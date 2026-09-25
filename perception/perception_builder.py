"""Convert raw official API data into the captain's stable team interface."""

import math

from core.geometry import calculate_ttc, speed_2d, world_to_ego
from core.interfaces import EgoState, Perception, Target
from simone_platform.case_resolver import resolve_scene_id


class PerceptionBuilder(object):
    def __init__(self, adapter, route_manager, scene_override=0):
        self.adapter = adapter
        self.route_manager = route_manager
        self.scene_override = scene_override
        self.case_name = ""
        self.scene_id = int(scene_override)

    def update_case_info(self):
        info = self.adapter.get_case_info()
        self.case_name = info.get("case_name", "")
        self.scene_id = resolve_scene_id(self.case_name, self.scene_override)
        return info

    def build(self):
        return self.build_from_raw(self.adapter.read_raw_snapshot())

    def build_from_raw(self, raw):
        result = Perception()
        result.case_name = self.case_name
        result.scene_id = self.scene_id
        gps = raw.get("gps") if raw else None
        if not gps:
            result.errors.append("GPS_UNAVAILABLE")
            return result
        result.ego = self._build_ego(gps)
        result.frame_id = result.ego.frame_id
        result.timestamp = result.ego.timestamp
        if not result.ego.valid:
            result.errors.append("GPS_LOST")
            return result
        result.lane = self.route_manager.update(result.ego)
        if not result.lane.valid:
            result.errors.append("LANE_UNAVAILABLE")
        lane_half_width = max(1.4, result.lane.lane_width * 0.5)
        for item in raw.get("targets", []):
            target = self._build_target(item, result.ego, lane_half_width)
            if target.valid:
                result.targets.append(target)
        result.targets.sort(key=lambda item: item.distance)
        result.valid = True
        return result

    @staticmethod
    def _build_ego(gps):
        ego = EgoState()
        ego.frame_id = int(gps.get("frame", 0))
        ego.timestamp = int(gps.get("timestamp", 0))
        ego.x = float(gps.get("x", 0.0))
        ego.y = float(gps.get("y", 0.0))
        ego.z = float(gps.get("z", 0.0))
        ego.heading = float(gps.get("heading", 0.0))
        ego.speed = speed_2d(float(gps.get("vx", 0.0)), float(gps.get("vy", 0.0)))
        ego.acceleration = speed_2d(float(gps.get("ax", 0.0)), float(gps.get("ay", 0.0)))
        ego.throttle = float(gps.get("throttle", 0.0))
        ego.brake = float(gps.get("brake", 0.0))
        ego.steering = float(gps.get("steering", 0.0))
        ego.gear = int(gps.get("gear", 0))
        ego.valid = not bool(gps.get("is_lost", False))
        return ego

    @staticmethod
    def _build_target(item, ego, lane_half_width):
        target = Target()
        target.id = int(item.get("id", -1))
        target.type = int(item.get("type", 0))
        target.x = float(item.get("x", 0.0))
        target.y = float(item.get("y", 0.0))
        target.z = float(item.get("z", 0.0))
        target.vx = float(item.get("vx", 0.0))
        target.vy = float(item.get("vy", 0.0))
        target.vz = float(item.get("vz", 0.0))
        target.length = float(item.get("length", 0.0))
        target.width = float(item.get("width", 0.0))
        target.height = float(item.get("height", 0.0))
        target.longitudinal_distance, target.lateral_distance = world_to_ego(
            ego.x, ego.y, ego.heading, target.x, target.y
        )
        target.distance = math.sqrt(
            target.longitudinal_distance * target.longitudinal_distance
            + target.lateral_distance * target.lateral_distance
        )
        target.ttc, target.relative_speed = calculate_ttc(
            target.longitudinal_distance, ego.speed, target.vx, target.vy, ego.heading
        )
        target.same_lane = (
            target.longitudinal_distance > -target.length
            and abs(target.lateral_distance) <= lane_half_width
        )
        target.valid = target.id >= 0 and float(item.get("probability", 1.0)) >= 0.05
        return target

