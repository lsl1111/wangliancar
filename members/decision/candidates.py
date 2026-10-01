"""Route-relative target facts for the decision member.

The current lane anchors GPS progress. A verified continuous successor is used
only when its geometry has the unchanged current-lane prefix. Projection and
vehicle extent uncertainty remain explicit instead of becoming zero metres.
"""

import math



CURRENT_LANE = "CURRENT_LANE"
FORWARD_ROUTE = "FORWARD_ROUTE"
OTHER_LANE = "OTHER_LANE"
UNKNOWN = "UNKNOWN"
EPS = 1e-6


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _points(raw):
    if not isinstance(raw, (list, tuple)) or not 2 <= len(raw) <= 20000:
        return None
    points = []
    for item in raw:
        if (not isinstance(item, (list, tuple)) or len(item) < 2
                or not _finite(item[0]) or not _finite(item[1])):
            return None
        point = (float(item[0]), float(item[1]))
        if not points or math.hypot(point[0] - points[-1][0],
                                    point[1] - points[-1][1]) > EPS:
            points.append(point)
    return points if len(points) >= 2 else None


def _length(points):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(points, points[1:]))


def _project_unique(points, x, y, ambiguity_m):
    """Find one bounded route projection, rejecting distant equal matches."""
    best = None
    contenders = []
    along = 0.0
    for index, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        raw = ((x - a[0]) * dx + (y - a[1]) * dy) / (length * length)
        ratio = max(0.0, min(1.0, raw))
        px, py = a[0] + ratio * dx, a[1] + ratio * dy
        distance = math.hypot(x - px, y - py)
        progress = along + ratio * length
        contenders.append((distance, progress))
        if best is None or distance < best["distance"]:
            best = {"s": progress, "distance": distance,
                    "index": index, "raw_ratio": raw,
                    "heading": math.atan2(dy, dx)}
        along += length
    if best is None:
        return None
    if ((best["index"] == 0 and best["raw_ratio"] < -EPS)
            or (best["index"] == len(points) - 2 and
                best["raw_ratio"] > 1.0 + EPS)):
        return None
    for distance, progress in contenders:
        if (abs(distance - best["distance"]) <= 1e-3
                and abs(progress - best["s"]) > ambiguity_m):
            return None
    return best


class RouteContext(object):
    def __init__(self, perception, settings):
        lane = perception.lane
        self.current = _points(lane.center_line)
        self.points = self.current
        self.forward_ids = ()
        self.current_length = _length(self.current) if self.current else 0.0
        self.ego_s = None
        self.ego_heading = None
        self.reason = ""
        if self.current is None:
            self.reason = "current lane centre line unavailable"
            return
        ego = perception.ego
        anchor = _project_unique(self.current, ego.x, ego.y,
                                 settings.route_ambiguity_m)
        if anchor is None or anchor["distance"] > settings.projection_tolerance_m:
            self.reason = "ego route projection unavailable or ambiguous"
            return
        self.ego_s = anchor["s"]
        self.ego_heading = anchor["heading"]
        if lane.forward_reference_valid is not True:
            return
        forward = _points(lane.forward_reference)
        ids = lane.forward_lane_ids
        if (forward is None or len(forward) < len(self.current)
                or forward[:len(self.current)] != self.current
                or not isinstance(ids, (list, tuple)) or not ids
                or ids[0] != lane.lane_id or not all(isinstance(i, str) for i in ids)):
            return
        self.points = forward
        self.forward_ids = tuple(ids[1:])

    def project(self, x, y, settings):
        if self.ego_s is None or not (_finite(x) and _finite(y)):
            return None
        result = _project_unique(self.points, x, y, settings.route_ambiguity_m)
        if result is None or result["distance"] > settings.projection_tolerance_m:
            return None
        return result


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


def build_candidate(target, ego, perception, route, settings):
    candidate = Candidate(target)
    if target.valid is not True:
        candidate.conflict, candidate.reason = True, "target record invalid"
        return candidate
    values = (target.x, target.y, target.vx, target.vy)
    if not all(_finite(value) for value in values):
        candidate.conflict, candidate.reason = True, "target pose or velocity invalid"
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
    projection = route.project(target.x, target.y, settings)
    if projection is not None:
        candidate.distance = projection["s"] - route.ego_s
        heading = projection["heading"]
    else:
        heading = ego.heading
    candidate.lead_speed = math.cos(heading) * target.vx + math.sin(heading) * target.vy
    candidate.lateral_speed = -math.sin(heading) * target.vx + math.cos(heading) * target.vy
    candidate.static = math.hypot(target.vx, target.vy) <= settings.static_speed_threshold
    extent = None
    if (_finite(target.length) and _finite(target.width)
            and target.length > 0.0 and target.width > 0.0):
        # The circumscribed circle bounds any unpublished orientation.
        extent = 0.5 * math.hypot(target.length, target.width)
    dx, dy = target.x - ego.x, target.y - ego.y
    ego_longitudinal = math.cos(ego.heading) * dx + math.sin(ego.heading) * dy
    ego_lateral = -math.sin(ego.heading) * dx + math.cos(ego.heading) * dy
    lateral = projection["distance"] if projection is not None else abs(ego_lateral)
    lane_half = (lane.lane_width * 0.5 if lane.lane_width_valid and
                 _finite(lane.lane_width) and lane.lane_width > 0.0
                 else settings.projection_tolerance_m)
    corridor = lane_half + (extent if extent is not None else settings.min_gap)
    if projection is None and ego_longitudinal < -corridor:
        return candidate
    entering = (candidate.lateral_speed is not None and
                abs(candidate.lateral_speed) > settings.static_speed_threshold and
                lateral <= corridor + abs(candidate.lateral_speed) * settings.conflict_horizon_s)
    ahead = (candidate.distance is not None and candidate.distance >= -corridor)
    if candidate.relation in (CURRENT_LANE, FORWARD_ROUTE):
        candidate.conflict = ahead or projection is None
    elif candidate.relation == OTHER_LANE:
        overlap = extent is not None and lateral <= extent
        candidate.conflict = (overlap or entering) and (ahead or projection is None)
    else:
        candidate.conflict = (lateral <= corridor or entering) and (
            ahead or projection is None or ego_longitudinal > -corridor)
    if not candidate.conflict:
        return candidate
    if projection is None:
        candidate.reason = "route projection unavailable or ambiguous"
        return candidate
    if candidate.relation == FORWARD_ROUTE:
        if projection["s"] < route.current_length - EPS:
            candidate.reason = "successor target projection contradicts route"
            return candidate
        if route.forward_ids and target.lane_id != route.forward_ids[0]:
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
    return candidate


def select_lead(candidates):
    leads = [item for item in candidates if item.conflict and
             item.relation in (CURRENT_LANE, FORWARD_ROUTE) and item.gap is not None]
    return min(leads, key=lambda item: (item.gap, item.target.id)) if leads else None
