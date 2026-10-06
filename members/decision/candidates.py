"""Route-relative target facts for the decision member.

The current lane anchors GPS progress. A verified continuous successor is used
only when its geometry has the unchanged current-lane prefix. Projection and
vehicle extent uncertainty remain explicit instead of becoming zero metres.
"""

import math
from core.route_motion import lateral_residual, lateral_drift_bound
from core.geometry import swept_path_distance
from core.obstacle_geometry import longitudinal_extent, swept_footprint_intersects, footprint_entry
from core.route_obstacles import (RouteContext, path_reference, motion_guard_distance,
                                  motion_guard_time, front_reach_reference)
from core.target_semantics import mapped_traffic_light
from core.traffic_quality import signal_stop_bound



CURRENT_LANE = "CURRENT_LANE"
FORWARD_ROUTE = "FORWARD_ROUTE"
OTHER_LANE = "OTHER_LANE"
UNKNOWN = "UNKNOWN"
EPS = 1e-6


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class Candidate(object):
    def __init__(self, target):
        self.target = target
        self.relation = UNKNOWN
        self.distance = None
        self.gap = None
        self.ttc = -1.0
        self.lead_speed = None
        self.lateral_speed = None
        self.static = False
        self.conflict = False
        self.stop_distance = None
        self.reason = ""
        self.motion_relevant = True
        self.motion_supported = False
        self.coverage_stop_distance = None


def build_candidate(target, ego, perception, route, settings):
    candidate = Candidate(target)
    if target.valid is not True:
        candidate.conflict, candidate.reason = True, "target record invalid"
        return candidate
    values = (target.x, target.y, target.vx, target.vy)
    if not all(_finite(value) for value in values):
        candidate.conflict, candidate.reason = True, "target pose or velocity invalid"
        return candidate
    if mapped_traffic_light(target, perception):
        candidate.reason = "mapped traffic-light fixture"
        return candidate
    lane = perception.lane
    if target.same_lane_valid is True and target.lane_id == lane.lane_id:
        candidate.relation = CURRENT_LANE
    elif (target.same_lane_valid is True and target.lane_id
          and target.lane_id in route.forward_ids):
        candidate.relation = FORWARD_ROUTE
    elif target.same_lane_valid is True and target.lane_id:
        # A candidate successor ID is not a selected forward route by itself.
        successors = getattr(lane, "successor_lane_ids", ())
        if target.lane_id not in successors:
            candidate.relation = OTHER_LANE
    projection = route.project(target.x, target.y, settings,
                               target.lane_id if target.same_lane_valid is True else None)
    if projection is not None:
        candidate.distance = projection["s"] - route.ego_s
        heading = projection["heading"]
    else:
        heading = ego.heading
    candidate.lead_speed = math.cos(heading) * target.vx + math.sin(heading) * target.vy
    candidate.lateral_speed = -math.sin(heading) * target.vx + math.cos(heading) * target.vy
    if projection is not None and candidate.relation in (CURRENT_LANE, FORWARD_ROUTE):
        width = (lane.lane_width if candidate.relation == CURRENT_LANE and lane.lane_width_valid
                 else getattr(target, 'lane_width_m', None))
        if candidate.relation == CURRENT_LANE or target.lane_id in route.spans:
            residual = lateral_residual(route.points, projection, target, width)
            if residual is not None:
                candidate.lateral_speed = residual
    candidate.static = math.hypot(target.vx, target.vy) <= settings.static_speed_threshold
    extent = None
    collision_extent = None
    if (_finite(target.length) and _finite(target.width)
            and target.length > 0.0 and target.width > 0.0):
        extent = longitudinal_extent(target, heading)
        collision_extent = 0.5 * math.hypot(target.length, target.width)
    dx, dy = target.x - ego.x, target.y - ego.y
    ego_longitudinal = math.cos(ego.heading) * dx + math.sin(ego.heading) * dy
    ego_lateral = -math.sin(ego.heading) * dx + math.cos(ego.heading) * dy
    lateral = projection["distance"] if projection is not None else abs(ego_lateral)
    lane_half = (lane.lane_width * 0.5 if lane.lane_width_valid and
                 _finite(lane.lane_width) and lane.lane_width > 0.0
                 else settings.projection_tolerance_m)
    corridor = lane_half + (collision_extent if collision_extent is not None else settings.min_gap)
    ahead_points = front_reach_reference(route.ahead(), settings.front_offset_m)
    padding = (settings.half_width_m if settings.half_width_m is not None else lane_half)
    padding += settings.motion_lateral_margin_m
    guard_time = motion_guard_time(ego.speed, settings.motion_guard_time_s,
                                   settings.motion_deceleration)
    if settings.front_offset_m is not None and extent is not None:
        distance = motion_guard_distance(ego.speed, settings.motion_horizon_m,
                                         settings.motion_deceleration, settings.front_offset_m)
        signal_bound = signal_stop_bound(perception, settings.front_offset_m,
                                        settings.traffic_stop_margin)
        if signal_bound is not None and signal_bound > EPS:
            distance = min(distance, signal_bound)
        candidate.motion_relevant = swept_footprint_intersects(
            path_reference(front_reach_reference(route.ahead(distance), settings.front_offset_m)),
            target, padding, guard_time)
    if (candidate.relation not in (CURRENT_LANE, FORWARD_ROUTE)
            and target.lane_id not in lane.successor_lane_ids):
        # Failed bounded projection is not evidence of a collision. Evaluate
        # whether this object's footprint/sweep can touch the known ahead road.
        if extent is not None:
            # The known orientation uses the same footprint sweep as planning.
            # Vehicle length must not become an arbitrary sideways radius.
            intersects = swept_footprint_intersects(path_reference(ahead_points),
                                                    target, padding, guard_time)
        else:
            endpoint = (target.x + target.vx*guard_time, target.y + target.vy*guard_time)
            intersects = swept_path_distance(ahead_points, (target.x,target.y), endpoint) <= corridor
        if not intersects:
            return candidate
        if extent is not None and not candidate.motion_relevant:
            # Planning retains the observed far-away footprint stop boundary;
            # unsupported motion outside its local range is not an emergency.
            return candidate
    if projection is None and ego_longitudinal < -corridor:
        return candidate
    entering = (candidate.lateral_speed is not None and
                abs(candidate.lateral_speed) > settings.static_speed_threshold and
                lateral <= corridor + abs(candidate.lateral_speed) * settings.conflict_horizon_s)
    ahead = (candidate.distance is not None and candidate.distance >= -corridor)
    if candidate.relation in (CURRENT_LANE, FORWARD_ROUTE):
        candidate.conflict = ahead or projection is None
    elif candidate.relation == OTHER_LANE:
        candidate.conflict = ahead or projection is None
    else:
        candidate.conflict = ((extent is not None or lateral <= corridor or entering)
                              and (ahead or projection is None or ego_longitudinal > -corridor))
    if not candidate.conflict:
        return candidate
    if projection is None:
        # A mapped vehicle beyond the known road may overlap only the front
        # overhang at its last rear-axle pose. Stop the whole ego body inside
        # coverage instead of inventing its lane or requiring standstill now.
        # Unmapped/ambiguous targets and overlap inside coverage stay unknown.
        known = path_reference(route.ahead())
        if (candidate.relation == OTHER_LANE and target.same_lane_valid is True
                and route.projection_reason == 'outside_route_start_or_end'
                and extent is not None and settings.front_offset_m is not None and known):
            entry = footprint_entry(path_reference(ahead_points), target, padding)
            if entry is not None and entry >= known[-1][0] - EPS:
                candidate.coverage_stop_distance = max(0.0, known[-1][0]
                    - settings.front_offset_m - settings.obstacle_stop_margin)
                return candidate
        candidate.reason = ("route projection unavailable or ambiguous: {0};"
                            "reference_status={1};target_lane={2}".format(
                                route.projection_reason, route.reference_status, target.lane_id))
        return candidate
    if candidate.relation == FORWARD_ROUTE:
        if projection["s"] < route.current_length - EPS:
            candidate.reason = "successor target projection contradicts route"
            return candidate
        if (route.forward_ids and target.lane_id != route.forward_ids[0]
                and target.lane_id not in route.spans):
            candidate.reason = "later successor segment boundary unavailable"
            return candidate
    if settings.front_offset_m is None or extent is None:
        candidate.reason = "vehicle front or target extent unknown"
        return candidate
    candidate.gap = max(0.0, candidate.distance - extent - settings.front_offset_m)
    closing = (math.cos(route.ego_heading) * ego.vx
               + math.sin(route.ego_heading) * ego.vy - candidate.lead_speed)
    if closing > 0.05 and candidate.gap > 0.0:
        candidate.ttc = candidate.gap / closing
    if (candidate.relation in (CURRENT_LANE, FORWARD_ROUTE)
            and candidate.distance >= 0.0):
        candidate.stop_distance = max(0.0, candidate.distance - extent
                                      - settings.front_offset_m
                                      - settings.obstacle_stop_margin)
        width = (lane.lane_width if candidate.relation == CURRENT_LANE and lane.lane_width_valid
                 else getattr(target, 'lane_width_m', None))
        try:
            projection['_coverage_checked'] = True
            lateral_drift_bound(target, projection, candidate.lateral_speed, width, guard_time,
                                settings.motion_lateral_margin_m, settings.motion_tolerance_mps)
            candidate.motion_supported = candidate.lead_speed >= -settings.motion_tolerance_mps
        except ValueError:
            pass
    return candidate


def select_lead(candidates):
    leads = [item for item in candidates if item.conflict and
             item.relation in (CURRENT_LANE, FORWARD_ROUTE) and item.gap is not None]
    return min(leads, key=lambda item: (item.gap, item.target.id)) if leads else None
