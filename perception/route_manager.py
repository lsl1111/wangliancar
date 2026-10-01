"""HD map lookup and lane-context normalization."""

import math
import copy
import time

from core.geometry import nearest_path_error, normalize_angle, project_polyline
from core.interfaces import LaneContext


def _sdk_string(value):
    if value is None:
        return ""
    if hasattr(value, "GetString"):
        return value.GetString()
    return str(value)


MAX_REFERENCE_POINTS = 20000
FORWARD_HORIZON_M = 120.0  # Beyond the planner's 60 m horizon; never a goal.
MAX_FORWARD_LANES = 32
JOIN_POSITION_TOLERANCE_M = 0.1
JOIN_HEADING_TOLERANCE_RAD = 0.35
ROUTE_REFERENCE_VERSION = "successor-continuation-v1"


def _vector_points(vector):
    points = []
    size = int(vector.Size())
    if not 0 <= size <= MAX_REFERENCE_POINTS:
        raise ValueError("map sample point budget exceeded")
    for index in range(size):
        item = vector.GetElement(index)
        points.append((float(item.x), float(item.y), float(item.z)))
    return points


def _clean_points(points):
    clean = []
    for point in points:
        if not all(math.isfinite(v) for v in point):
            raise ValueError("nonfinite map sample")
        if not clean or math.hypot(point[0] - clean[-1][0], point[1] - clean[-1][1]) > 1e-6:
            clean.append(point)
    if len(clean) < 2:
        raise ValueError("degenerate map sample")
    return clean


def _length(points):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def _heading(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _connected_segment(reference, points):
    """Orient at the join, not against the ego heading far before a curve.

    Endpoints differing by at most map-rounding tolerance share the existing
    endpoint. No gap, junction chord or extrapolated lane is manufactured.
    """
    tail = reference[-1]
    previous_heading = _heading(reference[-2], tail)
    candidates = []
    for segment, reversed_direction in ((points, False), (list(reversed(points)), True)):
        start = segment[0]
        if (math.hypot(start[0] - tail[0], start[1] - tail[1]) > JOIN_POSITION_TOLERANCE_M
                or abs(start[2] - tail[2]) > JOIN_POSITION_TOLERANCE_M):
            continue
        next_heading = _heading(start, segment[1])
        joined_heading = _heading(tail, segment[1])
        if (math.hypot(segment[1][0] - tail[0], segment[1][1] - tail[1]) <= 1e-6
                or abs(normalize_angle(next_heading - previous_heading)) > JOIN_HEADING_TOLERANCE_RAD
                or abs(normalize_angle(joined_heading - previous_heading)) > JOIN_HEADING_TOLERANCE_RAD
                or abs(normalize_angle(joined_heading - next_heading)) > JOIN_HEADING_TOLERANCE_RAD):
            continue
        candidates.append((segment, reversed_direction))
    return candidates[0] if len(candidates) == 1 else None


def _forward_links(link_info, reversed_direction):
    """Keep native ID objects for SDK calls; publish strings separately."""
    if link_info is None or not link_info.exists:
        return None
    field = "predecessorLaneIds" if reversed_direction else "successorLaneIds"
    vector = getattr(link_info.laneLink, field, None)
    if vector is None or not 0 <= int(vector.Size()) <= MAX_FORWARD_LANES:
        return None
    result, seen = [], set()
    for i in range(int(vector.Size())):
        native = vector.GetElement(i)
        identity = _sdk_string(native)
        if not identity:
            return None
        if identity not in seen:
            result.append((identity, native))
            seen.add(identity)
    return result


def _orient_forward(points, heading, x=None, y=None):
    if len(points) < 2:
        return points
    projection = project_polyline(points, points[0][0] if x is None else x,
                                  points[0][1] if y is None else y)
    if projection is None:
        return []
    path_heading = projection["heading"]
    if math.cos(path_heading - heading) < 0.0:
        return list(reversed(points))
    return points


class RouteManager(object):
    def __init__(self, adapter, logger, refresh_sec=0.2):
        self.adapter = adapter
        self.logger = logger
        self.refresh_sec = refresh_sec
        self.last_update = 0.0
        self.last_lane = LaneContext()

    def update(self, ego, force=False):
        if not ego.valid or not self.adapter.map_loaded:
            return LaneContext()
        now = time.monotonic()
        cached_projection = project_polyline(self.last_lane.center_line, ego.x, ego.y)
        crossed_end = (cached_projection is not None
                       and cached_projection["index"] == len(self.last_lane.center_line) - 2
                       and cached_projection["raw_ratio"] >= 1.0)
        if (not force and not crossed_end and self.last_lane.valid
                and now - self.last_update < self.refresh_sec):
            lane = copy.deepcopy(self.last_lane)
            lane.lateral_offset, lane.heading_error = nearest_path_error(
                lane.center_line, ego.x, ego.y, ego.heading
            )
            return lane
        try:
            lane = self._lookup(ego)
            self.last_lane = copy.deepcopy(lane)
            self.last_update = now
            return lane
        except Exception as exc:
            self.logger.warning("车道查询失败: %s", exc)
            return LaneContext()

    def _lookup(self, ego):
        hdmap = self.adapter.hdmap
        position = hdmap.pySimPoint3D(ego.x, ego.y, ego.z)
        nearest = hdmap.getNearMostLane(position)
        if not nearest.exists:
            return LaneContext()
        lane = LaneContext()
        lane.lane_id = _sdk_string(nearest.laneId)
        sample = hdmap.getLaneSample(nearest.laneId)
        if not sample.exists:
            return lane
        points = _clean_points(_vector_points(sample.laneInfo.centerLine))
        lane.center_line = _orient_forward(points, ego.heading, ego.x, ego.y)
        reversed_direction = bool(points and lane.center_line and lane.center_line[0] != points[0])
        lane.left_boundary = _vector_points(sample.laneInfo.leftBoundary)
        lane.right_boundary = _vector_points(sample.laneInfo.rightBoundary)
        for name in ("left_boundary", "right_boundary"):
            if not all(math.isfinite(v) for point in getattr(lane, name) for v in point):
                setattr(lane, name, [])
        link_info = None
        if hasattr(hdmap, "getLaneLink"):
            try:
                link_info = hdmap.getLaneLink(nearest.laneId)
            except Exception as exc:
                self.logger.warning("车道连通查询失败: %s", exc)
            if link_info is not None and link_info.exists:
                lane.left_lane_id = _sdk_string(link_info.laneLink.leftNeighborLaneId)
                lane.right_lane_id = _sdk_string(link_info.laneLink.rightNeighborLaneId)
                for name, field in (("predecessor_lane_ids", "predecessorLaneIds"),
                                    ("successor_lane_ids", "successorLaneIds")):
                    vector = getattr(link_info.laneLink, field, None)
                    if vector is not None:
                        setattr(lane, name, [_sdk_string(vector.GetElement(i)) for i in range(vector.Size())])
        if hasattr(hdmap, "getLaneWidth"):
            width_info = hdmap.getLaneWidth(nearest.laneId, position)
            if width_info.exists and math.isfinite(float(width_info.width)) and float(width_info.width) > 0.1:
                lane.lane_width = float(width_info.width)
                lane.lane_width_valid = True
        if hasattr(hdmap, "getRoadMark"):
            try:
                mark_info = hdmap.getRoadMark(position, nearest.laneId)
                if mark_info.exists:
                    lane.left_mark_type = _sdk_string(mark_info.left.type)
                    lane.right_mark_type = _sdk_string(mark_info.right.type)
            except (AttributeError, TypeError, ValueError) as exc:
                self.logger.warning("车道标线查询失败: %s", exc)
        if reversed_direction:
            lane.left_boundary, lane.right_boundary = list(reversed(lane.right_boundary)), list(reversed(lane.left_boundary))
            lane.left_mark_type, lane.right_mark_type = lane.right_mark_type, lane.left_mark_type
            lane.left_lane_id, lane.right_lane_id = lane.right_lane_id, lane.left_lane_id
            lane.predecessor_lane_ids, lane.successor_lane_ids = lane.successor_lane_ids, lane.predecessor_lane_ids
        lane.lateral_offset, lane.heading_error = nearest_path_error(
            lane.center_line, ego.x, ego.y, ego.heading
        )
        lane.valid = project_polyline(lane.center_line, ego.x, ego.y) is not None
        lane.source = "hdmap"
        if lane.valid:
            self._extend_reference(lane, ego, link_info, reversed_direction)
        return lane

    def _extend_reference(self, lane, ego, link_info, reversed_direction):
        """Bounded map expansion; only a unique, connected forward lane is used.

        A partial reference is still usable geometry. Its end remains a stop
        boundary until the map/route supplies a verified continuation.
        """
        reference = list(lane.center_line)
        lane.forward_reference = reference
        lane.forward_lane_ids = [lane.lane_id]
        lane.forward_reference_valid = True
        projection = project_polyline(reference, ego.x, ego.y)
        remaining = _length(reference) - projection["s"]
        visited = {lane.lane_id}
        hdmap = self.adapter.hdmap
        while True:
            try:
                links = _forward_links(link_info, reversed_direction)
                if links is None:
                    lane.forward_reference_status = "link_unavailable"
                    return
                if not links:
                    lane.forward_reference_status = "map_end"
                    return
                if remaining >= FORWARD_HORIZON_M:
                    lane.forward_reference_status = "lookahead_limit"
                    return
                if len(links) != 1:
                    lane.forward_reference_status = "ambiguous_successor"
                    return
                identity, native = links[0]
                if identity in visited:
                    lane.forward_reference_status = "cycle"
                    return
                if len(visited) >= MAX_FORWARD_LANES:
                    lane.forward_reference_status = "lane_limit"
                    return
                sample = hdmap.getLaneSample(native)
                if not sample.exists:
                    lane.forward_reference_status = "successor_unavailable"
                    return
                points = _clean_points(_vector_points(sample.laneInfo.centerLine))
                connection = _connected_segment(reference, points)
                if connection is None:
                    lane.forward_reference_status = "disconnected_successor"
                    return
                points, reversed_direction = connection
                if len(reference) + len(points) - 1 > MAX_REFERENCE_POINTS:
                    lane.forward_reference_status = "point_limit"
                    return
                previous_end = reference[-1]
                reference.extend(points[1:])
                remaining += _length([previous_end] + points[1:])
                lane.forward_lane_ids.append(identity)
                visited.add(identity)
                link_info = hdmap.getLaneLink(native) if hasattr(hdmap, "getLaneLink") else None
            except Exception as exc:
                # Optional successor failure must not discard the usable
                # current lane, its traffic association or target membership.
                lane.forward_reference_status = "successor_query_failed"
                self.logger.warning("前方车道接续查询失败: %s", exc)
                return

    def locate_target(self, target):
        """Map membership requires bounded projection and a measured lane width."""
        if not self.adapter.map_loaded:
            return "", False
        try:
            hdmap = self.adapter.hdmap
            pos = hdmap.pySimPoint3D(target.x, target.y, target.z)
            nearest = hdmap.getNearMostLane(pos)
            if not nearest.exists:
                return "", False
            sample = hdmap.getLaneSample(nearest.laneId)
            width = hdmap.getLaneWidth(nearest.laneId, pos)
            if not sample.exists or not width.exists or not math.isfinite(width.width) or width.width <= 0:
                return "", False
            points = _vector_points(sample.laneInfo.centerLine)
            if not all(math.isfinite(v) for p in points for v in p):
                return "", False
            projection = project_polyline(points, target.x, target.y)
            if (projection is None or not 0 <= projection["raw_ratio"] <= 1 or
                    projection["distance"] > width.width * 0.5):
                return "", False
            i, r = projection["index"], projection["ratio"]
            road_z = points[i][2] + r * (points[i + 1][2] - points[i][2])
            if abs(target.z - road_z) > max(2.0, target.height):
                return "", False
            return _sdk_string(nearest.laneId), True
        except (AttributeError, TypeError, ValueError, RuntimeError):
            return "", False
