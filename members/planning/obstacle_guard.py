"""Conservative path stop constraint for current Sensor API objects.

Only forward, approximately path-aligned traffic is supported. A moving
object is treated as if it could stop at its current position; that is a
conservative bound for a forward lead, not a prediction of its future path.
Small lateral motion needs verified current-lane geometry and an expanded
envelope; it is not silently discarded as sensor noise.
Vehicle dimensions must be supplied explicitly before a moving trajectory
with targets can be marked valid.
"""

import math

from core.geometry import project_polyline
from core.obstacle_geometry import footprint_entry
from core.validation import number


EPS = 1e-9


def _lateral_drift(perception, target, projection, velocity, settings):
    """Bound constant lateral drift within a verified current-lane corridor.

    This local assumption does not model a future lane change or acceleration.
    Require at least the trial observation window and current braking time.
    The full drift expands the obstacle envelope even when motion falls under
    an explicitly configured tolerance.
    """
    if abs(velocity) <= EPS:
        return 0.0
    window = max(settings.lateral_guard_time_s,
                 perception.ego.speed / settings.deceleration)
    drift = abs(velocity) * window
    if not number(drift):
        raise ValueError("nonfinite obstacle lateral drift")
    if abs(velocity) <= settings.motion_tolerance_mps:
        return drift
    lane = perception.lane
    if (target.same_lane_valid is not True or target.lane_id != lane.lane_id
            or lane.lane_width_valid is not True or not number(lane.lane_width)
            or lane.lane_width <= 0 or not number(target.heading)
            or not 0.0 <= projection["raw_ratio"] <= 1.0):
        raise ValueError("crossing or oncoming obstacle motion unsupported")
    angle = target.heading - projection["heading"]
    lateral_extent = 0.5 * (target.length * abs(math.sin(angle))
                            + target.width * abs(math.cos(angle)))
    clearance = lane.lane_width * 0.5 - lateral_extent - settings.lateral_margin_m
    if clearance <= 0 or projection["distance"] + drift > clearance:
        raise ValueError("crossing or oncoming obstacle motion unsupported")
    return drift


def obstacle_stop(perception, reference, settings, clearance_m=None, clearances_m=None):
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
    nearest = None
    for target in perception.targets:
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
        heading = projection["heading"]
        longitudinal_velocity = (math.cos(heading) * target.vx
                                 + math.sin(heading) * target.vy)
        lateral_velocity = (-math.sin(heading) * target.vx
                            + math.cos(heading) * target.vy)
        drift_bound = abs(lateral_velocity) * max(settings.lateral_guard_time_s,
                                                  perception.ego.speed/settings.deceleration)
        padding = settings.half_width_m + settings.lateral_margin_m
        # Filter only after bounding future lateral motion. A car wholly in
        # another lane is harmless; one predicted to enter this path is not.
        if footprint_entry(reference, target, padding + drift_bound) is None:
            continue
        if longitudinal_velocity < -settings.motion_tolerance_mps:
            raise ValueError("crossing or oncoming obstacle motion unsupported")
        padding += _lateral_drift(perception, target, projection,
                                  lateral_velocity, settings)
        entry = footprint_entry(reference, target, padding)
        if entry is None:
            continue
        gap = settings.obstacle_margin_m if clearance_m is None else clearance_m
        if clearances_m is not None:
            gap = clearances_m.get(str(target.id), gap)
        stop = max(0.0, entry - settings.front_offset_m - gap)
        nearest = stop if nearest is None else min(nearest, stop)
    return nearest
