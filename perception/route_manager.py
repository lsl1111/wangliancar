"""HD map lookup and lane-context normalization."""

import math
import time

from core.geometry import nearest_path_error
from core.interfaces import LaneContext


def _sdk_string(value):
    if value is None:
        return ""
    if hasattr(value, "GetString"):
        return value.GetString()
    return str(value)


def _vector_points(vector):
    points = []
    for index in range(int(vector.Size())):
        item = vector.GetElement(index)
        points.append((float(item.x), float(item.y), float(item.z)))
    return points


def _orient_forward(points, heading):
    if len(points) < 2:
        return points
    path_heading = math.atan2(points[-1][1] - points[0][1], points[-1][0] - points[0][0])
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
        now = time.time()
        if not force and self.last_lane.valid and now - self.last_update < self.refresh_sec:
            lane = self.last_lane
            lane.lateral_offset, lane.heading_error = nearest_path_error(
                lane.center_line, ego.x, ego.y, ego.heading
            )
            return lane
        try:
            lane = self._lookup(ego)
            self.last_lane = lane
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
        lane.center_line = _orient_forward(_vector_points(sample.laneInfo.centerLine), ego.heading)
        if hasattr(hdmap, "getLaneLink"):
            link_info = hdmap.getLaneLink(nearest.laneId)
            if link_info.exists:
                lane.left_lane_id = _sdk_string(link_info.laneLink.leftNeighborLaneId)
                lane.right_lane_id = _sdk_string(link_info.laneLink.rightNeighborLaneId)
        if hasattr(hdmap, "getLaneWidth"):
            width_info = hdmap.getLaneWidth(nearest.laneId, position)
            if width_info.exists and float(width_info.width) > 0.1:
                lane.lane_width = float(width_info.width)
        lane.lateral_offset, lane.heading_error = nearest_path_error(
            lane.center_line, ego.x, ego.y, ego.heading
        )
        lane.valid = len(lane.center_line) >= 2
        return lane

