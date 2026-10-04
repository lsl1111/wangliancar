"""Verified road projection and local obstacle-check range, without the SDK.

Decision and independent safety consume the same road facts without calling
each other's implementation. No output intent is used as collision evidence.
"""
import math

from core.geometry import polyline_prefix, projection_within_polyline
from core.obstacle_geometry import longitudinal_extent, swept_footprint_intersects
from core.route_motion import lateral_residual
from core.route_segments import verified_spans
from core.validation import number


EPS = 1e-6


def clean_points(raw):
    if not isinstance(raw, (list, tuple)) or not 2 <= len(raw) <= 20000:
        return None
    points = []
    for item in raw:
        if (not isinstance(item, (list, tuple)) or len(item) < 2
                or not number(item[0]) or not number(item[1])):
            return None
        point = (float(item[0]), float(item[1]))
        if not points or math.hypot(point[0]-points[-1][0], point[1]-points[-1][1]) > EPS:
            points.append(point)
    return points if len(points) >= 2 else None


def path_length(points):
    return sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:]))


def project_unique(points, x, y, ambiguity_m, diagnostics=None):
    best, contenders, along = None, [], 0.0
    for index, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(dx, dy)
        raw = ((x-a[0])*dx + (y-a[1])*dy)/(length*length)
        ratio = max(0.0, min(1.0, raw))
        px, py = a[0]+ratio*dx, a[1]+ratio*dy
        distance, progress = math.hypot(x-px, y-py), along+ratio*length
        contenders.append((distance, progress))
        if best is None or distance < best['distance']:
            best = dict(s=progress, distance=distance, index=index, raw_ratio=raw,
                        heading=math.atan2(dy, dx), point=(px, py))
        along += length
    if best is None:
        if diagnostics is not None:
            diagnostics['reason'] = 'no_route_segment'
        return None
    if not projection_within_polyline(best, len(points)):
        if diagnostics is not None:
            diagnostics['reason'] = 'outside_route_start_or_end'
        return None
    for distance, progress in contenders:
        if (abs(distance-best['distance']) <= 1e-3
                and abs(progress-best['s']) > ambiguity_m):
            if diagnostics is not None:
                diagnostics['reason'] = 'multiple_route_positions'
            return None
    return best


class RouteContext(object):
    def __init__(self, perception, settings):
        lane = perception.lane
        self.lane_id = lane.lane_id
        self.current = clean_points(lane.center_line)
        self.points, self.forward_ids, self.spans = self.current, (), {}
        self.projection_reason, self.reason = '', ''
        self.reference_status = lane.forward_reference_status
        self.current_length = path_length(self.current) if self.current else 0.0
        self.ego_s, self.ego_heading, self.anchor = None, None, None
        if self.current is None:
            self.reason = 'current lane centre line unavailable'
            return
        ego = perception.ego
        anchor = project_unique(self.current, ego.x, ego.y, settings.route_ambiguity_m)
        if anchor is None or anchor['distance'] > settings.projection_tolerance_m:
            self.reason = 'ego route projection unavailable or ambiguous'
            return
        self.anchor, self.ego_s, self.ego_heading = anchor, anchor['s'], anchor['heading']
        if lane.forward_reference_valid is not True:
            return
        forward, ids = clean_points(lane.forward_reference), lane.forward_lane_ids
        if (forward is None or len(forward) < len(self.current)
                or forward[:len(self.current)] != self.current
                or not isinstance(ids, (list, tuple)) or not ids
                or ids[0] != lane.lane_id or not all(isinstance(i, str) for i in ids)):
            return
        self.points, self.forward_ids = forward, tuple(ids[1:])
        self.spans = verified_spans(len(self.current), len(forward), ids,
                                   getattr(lane, 'forward_lane_spans', []))

    def project(self, x, y, settings, lane_id=None):
        self.projection_reason = ''
        if self.ego_s is None or not (number(x) and number(y)):
            self.projection_reason = 'route_anchor_or_target_coordinates_unavailable'
            return None
        points, offset, index_offset = self.points, 0.0, 0
        if lane_id in self.spans:
            start, end = self.spans[lane_id]
            points = self.points[start:end+1]
            offset, index_offset = path_length(self.points[:start+1]), start
        elif lane_id == self.lane_id:
            # A current-lane record must not jump to a close return leg when
            # optional continuation metadata is missing or inconsistent.
            points = self.current
        diagnostics = {}
        result = project_unique(points, x, y, settings.route_ambiguity_m, diagnostics)
        if result is None or result['distance'] > settings.projection_tolerance_m:
            self.projection_reason = diagnostics.get('reason', 'outside_route_corridor')
            return None
        result['s'] += offset
        result['index'] += index_offset
        return result

    def ahead(self, distance=None):
        if self.anchor is None:
            return []
        points = [self.anchor['point']] + self.points[self.anchor['index']+1:]
        return polyline_prefix(points, distance) if distance is not None else points


def path_reference(points):
    reference = [(0.0, points[0])] if points else []
    for a, b in zip(points, points[1:]):
        length = math.hypot(b[0]-a[0], b[1]-a[1])
        if length > EPS:
            reference.append((reference[-1][0]+length, b))
    return reference


def front_reach_reference(points, front_offset):
    """Collision-only front reach at the last known rear-axle pose.

    This is the body overhang along the last supported tangent, not additional
    route coverage. Stop coordinates and route validity keep their old domain.
    """
    result = list(points)
    if len(points) < 2 or not number(front_offset) or front_offset <= 0:
        return result
    a, b = points[-2:]
    dx, dy = b[0]-a[0], b[1]-a[1]
    length = math.hypot(dx, dy)
    if length > EPS:
        result.append((b[0]+front_offset*dx/length, b[1]+front_offset*dy/length))
    return result


def motion_guard_distance(speed, horizon, deceleration, front_offset):
    if (not all(number(v) for v in (speed, horizon, deceleration, front_offset))
            or speed < 0 or min(horizon, deceleration, front_offset) <= 0):
        raise ValueError('invalid obstacle motion guard dimensions or limits')
    return max(horizon, speed*speed/(2*deceleration)+front_offset)


def motion_guard_time(speed, observation_time, deceleration):
    if (not all(number(v) for v in (speed, observation_time, deceleration))
            or speed < 0 or min(observation_time, deceleration) <= 0):
        raise ValueError('invalid obstacle motion guard time')
    return max(observation_time, speed/deceleration)


def route_footprint_relevant(perception, target, route, planning_settings):
    """Independent physical relevance, or None when evidence is incomplete.

    The broad ego-axis band is only a fallback. With a verified road and
    dimensions, a stationary roadside fixture must actually overlap the body
    corridor. Moving targets retain their full constant-velocity sweep.
    Include measured pose/heading error so off-centre ego motion is not erased.
    This is an exclusion proof, not a trajectory intention or signal exemption.
    """
    lane, ego = perception.lane, perception.ego
    if (route.anchor is None or lane.lane_width_valid is not True
            or not number(lane.lane_width) or lane.lane_width <= 0
            or not all(number(v) for v in (target.x, target.y, target.vx, target.vy,
                                           target.length, target.width))
            or min(target.length, target.width) <= 0
            or not number(ego.heading)):
        return None
    distance = motion_guard_distance(ego.speed, planning_settings.horizon,
                                     planning_settings.deceleration,
                                     planning_settings.front_offset_m)
    seconds = motion_guard_time(ego.speed, planning_settings.lateral_guard_time_s,
                                planning_settings.deceleration)
    points = route.ahead(distance)
    # A lateral strip exactly encloses straight forward body travel. For an
    # unassociated object on a bend it does not certify the front-corner sweep;
    # keep the conservative TTC fallback rather than claim a clearance proof.
    headings = [math.atan2(b[1]-a[1], b[0]-a[0]) for a,b in zip(points,points[1:])]
    if not headings or any(abs(math.atan2(math.sin(h-headings[0]),
                                         math.cos(h-headings[0]))) > EPS for h in headings):
        return None
    reference = path_reference(front_reach_reference(points,
                                                     planning_settings.front_offset_m))
    if len(reference) < 2:
        return None
    heading_error = ego.heading-route.ego_heading
    padding = (planning_settings.half_width_m + planning_settings.lateral_margin_m
               + route.anchor['distance']
               + planning_settings.front_offset_m*abs(math.sin(heading_error)))
    return swept_footprint_intersects(reference, target, padding, seconds)


def mapped_route_motion(perception, target, route, settings, front_offset):
    """Independent along-road collision facts, or None without verified evidence.

    The local projection is usable for an oncoming target too: its signed
    speed stays negative. Only supported road turn is removed from lateral
    velocity; actual sideways slip is retained.
    """
    lane = perception.lane
    if (lane.valid is not True or target.same_lane_valid is not True
            or not number(front_offset) or front_offset <= 0
            or not all(number(v) for v in (target.x, target.y, target.vx, target.vy,
                                           target.length, target.width, target.heading))
            or min(target.length, target.width) <= 0 or route.ego_s is None):
        return None
    current = target.lane_id == lane.lane_id
    if not current and (target.lane_id not in route.forward_ids or target.lane_id not in route.spans):
        return None
    width = (lane.lane_width if current and lane.lane_width_valid
             else getattr(target, 'lane_width_m', None))
    if not number(width) or width <= 0.1:
        return None
    projection = route.project(target.x, target.y, settings, target.lane_id)
    if projection is None or projection['distance'] > width*.5:
        return None
    angle = target.heading-projection['heading']
    body_half_width = .5*(target.length*abs(math.sin(angle))
                           + target.width*abs(math.cos(angle)))
    if projection['distance']+body_half_width >= width*.5:
        return None
    heading = projection['heading']
    lead_speed = math.cos(heading)*target.vx + math.sin(heading)*target.vy
    lateral_speed = -math.sin(heading)*target.vx + math.cos(heading)*target.vy
    residual = lateral_residual(route.points, projection, target, width)
    if residual is not None:
        lateral_speed = residual
    distance = projection['s']-route.ego_s
    gap = max(0.0, distance-longitudinal_extent(target, heading)-front_offset)
    closing = (math.cos(route.ego_heading)*perception.ego.vx
               + math.sin(route.ego_heading)*perception.ego.vy-lead_speed)
    ttc = gap/closing if closing > .05 and distance > 0 else -1.0
    return dict(distance=distance, gap=gap, closing=closing, ttc=ttc,
                lead_speed=lead_speed, lateral_speed=lateral_speed,
                projection=projection, width=width)
