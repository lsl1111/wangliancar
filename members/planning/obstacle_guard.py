"""Conservative path stop constraint for current Sensor API objects.

Only forward, approximately path-aligned traffic is supported. A moving
object is treated as if it could stop at its current position; that is a
conservative bound for a forward lead, not a prediction of its future path.
Small lateral motion needs verified route-lane geometry and an expanded
envelope; it is not silently discarded as sensor noise.
Vehicle dimensions must be supplied explicitly before a moving trajectory
with targets can be marked valid.
"""

import math

from core.geometry import project_polyline, polyline_prefix, projection_within_polyline
from core.route_motion import lateral_residual, lateral_drift_bound
from core.obstacle_geometry import footprint_entry, swept_footprint_intersects
from core.validation import number
from core.target_semantics import mapped_traffic_light
from core.route_obstacles import (motion_guard_distance, motion_guard_time,
                                 front_reach_reference, path_reference, RouteContext)


EPS = 1e-9


def _route_association(perception, target, settings, route=None):
    """A mapped successor uses its own segment tangent and measured width."""
    lane = perception.lane
    if target.same_lane_valid is not True:
        return None
    route = route if route is not None else RouteContext(perception, settings)
    if (route.ego_s is None or (target.lane_id != lane.lane_id
            and (target.lane_id not in route.forward_ids or target.lane_id not in route.spans))):
        return None
    width = (lane.lane_width if target.lane_id == lane.lane_id and lane.lane_width_valid
             else getattr(target, 'lane_width_m', None))
    if not number(width) or width <= 0.1:
        return None
    local = route.project(target.x, target.y, settings, target.lane_id)
    if local is None or local['distance'] > width*.5:
        return None
    local['s'] -= route.ego_s
    local['_coverage_checked']=True
    local['_source_points']=route.points
    return local, width


def _lateral_drift(perception, target, projection, velocity, settings, route_width=None):
    """Bound constant lateral drift within a verified route-lane corridor.

    This local assumption does not model a future lane change or acceleration.
    Require at least the trial observation window and current braking time.
    The full drift expands the obstacle envelope even when motion falls under
    an explicitly configured tolerance.
    """
    window = motion_guard_time(perception.ego.speed,settings.lateral_guard_time_s,
                               settings.deceleration)
    lane = perception.lane
    width = route_width
    if width is None and (target.same_lane_valid is True and target.lane_id == lane.lane_id
                          and lane.lane_width_valid is True):
        width = lane.lane_width
    return lateral_drift_bound(target, projection, velocity, width, window,
                               settings.lateral_margin_m, settings.motion_tolerance_mps)


def obstacle_stop(perception, reference, settings, clearance_m=None, clearances_m=None,
                  motion_stop_distance=None):
    """Return nearest conservative stop distance, or None for a clear path.

    `reference` starts at the GPS rear-axle projection. The oriented target
    is expanded laterally, then the front offset and decision's net gap are
    removed once. Unknown orientation retains a conservative circle.
    """
    if not perception.targets:
        return None
    if settings.front_offset_m is None or settings.half_width_m is None:
        raise ValueError("obstacle-aware planning requires measured vehicle front offset and half-width")
    status = perception.source_status.get("targets", {})
    if (perception.targets_valid is not True
            or not isinstance(perception.target_source, str)
            or not perception.target_source.startswith("sensor:")
            or not isinstance(status, dict) or status.get("usable") is not True):
        raise ValueError("obstacle-aware planning requires a usable Sensor API target frame")
    points = [point for _, point in reference]
    collision_reference=path_reference(front_reach_reference(points,settings.front_offset_m))
    route = RouteContext(perception, settings)
    nearest = None
    for target in perception.targets:
        if mapped_traffic_light(target, perception):
            continue
        values = (target.x, target.y, target.vx, target.vy,
                  target.length, target.width)
        if (target.valid is not True or not all(number(value) for value in values)
                or target.length <= 0.0 or target.width <= 0.0):
            raise ValueError("obstacle extent or position unavailable")
        radius = 0.5 * math.hypot(target.length, target.width)
        envelope = radius + settings.half_width_m + settings.lateral_margin_m
        if not number(envelope):
            raise ValueError("nonfinite obstacle envelope")
        projection = project_polyline(points, target.x, target.y)
        if projection is None:
            raise ValueError("obstacle projection unavailable")
        projection['_coverage_checked']=projection_within_polyline(projection,len(points))
        association = _route_association(perception, target, settings, route)
        route_width = None
        if association is not None:
            projection, route_width = association
        heading = projection["heading"]
        longitudinal_velocity = (math.cos(heading) * target.vx
                                 + math.sin(heading) * target.vy)
        lateral_velocity = (-math.sin(heading) * target.vx
                            + math.cos(heading) * target.vy)
        if association is not None:
            residual = lateral_residual(projection['_source_points'],
                                        projection, target, route_width)
            if residual is not None:
                lateral_velocity = residual
        guard_time = motion_guard_time(perception.ego.speed,settings.lateral_guard_time_s,
                                       settings.deceleration)
        motion_distance = motion_guard_distance(perception.ego.speed,
            settings.horizon,settings.deceleration,settings.front_offset_m)
        if number(motion_stop_distance) and motion_stop_distance > EPS:
            motion_distance = min(motion_distance, motion_stop_distance)
        ahead = polyline_prefix(points, motion_distance)
        ahead=front_reach_reference(ahead,settings.front_offset_m)
        ahead_reference = [(0.0, ahead[0])]
        for a, b in zip(ahead, ahead[1:]):
            ahead_reference.append((ahead_reference[-1][0] +
                                    math.hypot(b[0]-a[0], b[1]-a[1]), b))
        padding = settings.half_width_m + settings.lateral_margin_m
        intersects = swept_footprint_intersects(ahead_reference, target, padding,
                                                guard_time)
        if intersects:
            if longitudinal_velocity < -settings.motion_tolerance_mps:
                raise ValueError("crossing or oncoming obstacle motion unsupported")
            padding += _lateral_drift(perception, target, projection,
                                       lateral_velocity, settings, route_width)
        entry = footprint_entry(collision_reference, target, padding)
        if entry is None:
            continue
        gap = settings.obstacle_margin_m if clearance_m is None else clearance_m
        if clearances_m is not None:
            gap = clearances_m.get(str(target.id), gap)
        stop = max(0.0, entry - settings.front_offset_m - gap)
        nearest = stop if nearest is None else min(nearest, stop)
    return nearest
