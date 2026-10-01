import ctypes
import json
import unittest
from types import SimpleNamespace

from core.interfaces import LaneContext
from core.serialization import perception_to_dict
from perception.perception_builder import PerceptionBuilder
from perception.route_manager import RouteManager
from simone_platform.simone_adapter import SimOneAdapter


class FakeAdapter(object):
    def get_case_info(self):
        return {"case_name": "06.车道居中控制-测试"}

    def read_traffic(self, lane_id):
        return [
            {"opendrive_id": 20, "status": 1, "count_down": 8, "x": 50.0, "y": 0.0,
             "stop_line_x": 48.0, "stop_line_y": 0.0},
            {"opendrive_id": 21, "status": 2, "count_down": 5, "x": 200.0, "y": 0.0,
             "stop_line_x": 198.0, "stop_line_y": 0.0},
        ]


class FakeRouteManager(object):
    def update(self, ego):
        lane = LaneContext()
        lane.lane_id = "1_0_-1"
        lane.center_line = [(0.0, 0.0, 0.0), (100.0, 0.0, 0.0)]
        lane.lane_width = 3.5
        lane.valid = True
        return lane


class PerceptionTests(unittest.TestCase):
    def setUp(self):
        self.builder = PerceptionBuilder(FakeAdapter(), FakeRouteManager())
        self.builder.update_case_info()

    def test_builds_stable_team_interface(self):
        raw = {
            "gps": {
                "frame": 100,
                "timestamp": 123456,
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "heading": 0.0,
                "vx": 10.0,
                "vy": 0.0,
                "ax": 0.0,
                "ay": 0.0,
                "gear": 1,
                "is_lost": False,
            },
            "targets": [
                {
                    "id": 8,
                    "type": 1,
                    "x": 20.0,
                    "y": 0.5,
                    "z": 0.0,
                    "vx": 5.0,
                    "vy": 0.0,
                    "vz": 0.0,
                    "length": 4.5,
                    "width": 1.8,
                    "height": 1.5,
                    "probability": 1.0,
                }
            ],
            "targets_valid": True,
            "targets_frame": 100,
            "targets_timestamp": 123456,
            "targets_age_ms": 0,
        }
        value = self.builder.build_from_raw(raw)
        self.assertTrue(value.valid)
        self.assertEqual(6, value.scene_id)
        self.assertEqual("1_0_-1", value.lane.lane_id)
        self.assertEqual(1, len(value.targets))
        self.assertFalse(value.targets[0].same_lane_valid)
        self.assertTrue(value.targets[0].lateral_band_match)
        self.assertAlmostEqual(4.0, value.targets[0].ttc)
        # Traffic is filled from the adapter-provided traffic lights: the nearest
        # lit signal is a RED light 50 m ahead, so it drives TrafficControl.
        self.assertTrue(value.traffic.valid)
        self.assertEqual("RED", value.traffic.signal_state)
        self.assertAlmostEqual(50.0, value.traffic.signal_distance)
        self.assertAlmostEqual(48.0, value.traffic.stop_line_distance)
        self.assertEqual(8, value.traffic.count_down)
        self.assertEqual(20, value.traffic.signal_id)
        self.assertEqual(-1, value.ego.age_ms)
        self.assertIn("traffic_source", perception_to_dict(value))

    def test_no_traffic_lights_keeps_unknown(self):
        class NoLightsAdapter(FakeAdapter):
            def read_traffic(self, lane_id):
                return []

        self.builder = PerceptionBuilder(NoLightsAdapter(), FakeRouteManager())
        self.builder.update_case_info()
        raw = {
            "gps": {"frame": 1, "x": 0.0, "y": 0.0, "z": 0.0, "heading": 0.0,
                    "vx": 0.0, "vy": 0.0, "ax": 0.0, "ay": 0.0, "gear": 1,
                    "is_lost": False},
            "targets": [],
        }
        value = self.builder.build_from_raw(raw)
        self.assertTrue(value.valid)
        self.assertFalse(value.traffic.valid)
        self.assertEqual("UNKNOWN", value.traffic.signal_state)
        self.assertEqual(-1.0, value.traffic.signal_distance)

    def test_gps_missing_is_invalid(self):
        value = self.builder.build_from_raw({"gps": None, "targets": []})
        self.assertFalse(value.valid)
        self.assertIn("GPS_UNAVAILABLE", value.errors)

    def test_rear_light_and_missing_stop_line_are_not_selected(self):
        class Lights(FakeAdapter):
            def read_traffic(self, lane_id):
                return [
                    {"status": 1, "x": -2.0, "y": 0.0,
                     "stop_line_x": -2.0, "stop_line_y": 0.0},
                    {"status": 2, "x": 40.0, "y": 0.0,
                     "stop_line_x": 38.0, "stop_line_y": 0.0},
                    {"status": 1, "x": 10.0, "y": 0.0},
                ]
        self.builder.adapter = Lights()
        value = self.builder.build_from_raw({"gps": self._gps(), "targets": []})
        self.assertEqual("GREEN", value.traffic.signal_state)
        self.assertAlmostEqual(38.0, value.traffic.stop_line_distance)
        self.assertIn("TRAFFIC_RECORD_INVALID", value.errors)

    def test_missing_light_coordinates_do_not_crash(self):
        class Lights(FakeAdapter):
            def read_traffic(self, lane_id):
                return [{"status": 1, "x": None, "y": None,
                         "stop_line_x": 25.0, "stop_line_y": 0.0}]
        self.builder.adapter = Lights()
        value = self.builder.build_from_raw({"gps": self._gps(), "targets": []})
        self.assertTrue(value.traffic.valid)
        self.assertEqual(-1.0, value.traffic.signal_distance)
        self.assertEqual(25.0, value.traffic.stop_line_distance)

    def test_failed_targets_distinguished_from_empty_and_signed_acceleration(self):
        gps = self._gps()
        gps["ax"] = -3.0
        raw = {"gps": gps, "targets": [], "targets_valid": False,
               "target_source": "none", "route_points": [(1.0, 2.0)],
               "route_valid": True, "environment": {"rainDensity": 0.3},
               "environment_valid": True}
        value = self.builder.build_from_raw(raw)
        self.assertFalse(value.targets_valid)
        self.assertNotIn("TARGETS_UNAVAILABLE", value.errors)
        self.builder.scene_id = 1
        self.assertIn("TARGETS_UNAVAILABLE", self.builder.build_from_raw(raw).errors)
        self.builder.scene_id = 6
        self.assertEqual(-3.0, value.ego.acceleration)
        snapshot = perception_to_dict(value)
        self.assertEqual([[1.0, 2.0]], [list(p) for p in snapshot["route_points"]])
        self.assertEqual(0.3, snapshot["environment"]["rainDensity"])
        raw["targets_valid"] = True
        raw["targets_frame"] = 1
        raw["targets_timestamp"] = 123
        raw["targets_age_ms"] = 0
        self.assertTrue(self.builder.build_from_raw(raw).targets_valid)

    def test_nonfinite_optional_data_is_not_published(self):
        raw = {"gps": self._gps(), "targets": [],
               "route_points": [(float("nan"), 0.0)], "route_valid": True,
               "radar_detections": [{"range": float("inf")}],
               "environment": {"rainDensity": float("nan")},
               "environment_valid": True}
        value = self.builder.build_from_raw(raw)
        self.assertTrue(value.valid)
        self.assertFalse(value.route_valid)
        self.assertEqual([], value.route_points)
        self.assertEqual([], value.radar_detections)
        self.assertFalse(value.environment_valid)
        self.assertIn("RADAR_DETECTIONS_INVALID", value.sensor_errors)
        json.dumps(perception_to_dict(value), allow_nan=False)

    def test_sdk_ctypes_enum_target_conversion(self):
        class ObstacleKind(ctypes.c_int):
            pass
        class Entry(ctypes.Structure):
            _fields_ = [("id", ctypes.c_int), ("type", ObstacleKind),
                        ("posX", ctypes.c_float), ("posY", ctypes.c_float),
                        ("posZ", ctypes.c_float), ("velX", ctypes.c_float),
                        ("velY", ctypes.c_float), ("velZ", ctypes.c_float),
                        ("length", ctypes.c_float), ("width", ctypes.c_float),
                        ("height", ctypes.c_float)]
        item = Entry()
        item.id, item.type = 8, 6
        value = SimOneAdapter._target_dict(item, 0.8)
        self.assertEqual(6, value["type"])
        self.assertEqual(0.8, value["probability"])

    def test_map_lane_boundaries_and_marks_are_populated(self):
        class Vector(list):
            def Size(self):
                return len(self)

            def GetElement(self, index):
                return self[index]

        left = Vector([SimpleNamespace(x=0.0, y=1.75, z=0.0),
                       SimpleNamespace(x=20.0, y=1.75, z=0.0)])
        right = Vector([SimpleNamespace(x=0.0, y=-1.75, z=0.0),
                        SimpleNamespace(x=20.0, y=-1.75, z=0.0)])
        center = Vector([SimpleNamespace(x=0.0, y=0.0, z=0.0),
                         SimpleNamespace(x=20.0, y=0.0, z=0.0)])
        lane_id = "1_0_-1"
        map_api = SimpleNamespace(
            pySimPoint3D=lambda x, y, z: (x, y, z),
            getNearMostLane=lambda pos: SimpleNamespace(exists=True, laneId=lane_id),
            getLaneSample=lambda lane: SimpleNamespace(
                exists=True, laneInfo=SimpleNamespace(
                    centerLine=center, leftBoundary=left, rightBoundary=right)),
            getRoadMark=lambda pos, lane: SimpleNamespace(
                exists=True, left=SimpleNamespace(type="solid"),
                right=SimpleNamespace(type="broken")))
        adapter = SimpleNamespace(map_loaded=True, hdmap=map_api)
        logger = SimpleNamespace(warning=lambda *args: None)
        context = RouteManager(adapter, logger).update(self._build_ego(), force=True)
        self.assertTrue(context.valid)
        self.assertEqual("solid", context.left_mark_type)
        self.assertEqual("broken", context.right_mark_type)
        self.assertEqual(2, len(context.left_boundary))

    @staticmethod
    def _build_ego():
        return PerceptionBuilder._build_ego(PerceptionTests._gps())

    @staticmethod
    def _gps():
        return {"frame": 1, "timestamp": 123, "x": 0.0, "y": 0.0,
                "z": 0.0, "heading": 0.0, "vx": 0.0, "vy": 0.0,
                "ax": 0.0, "ay": 0.0, "gear": 1, "is_lost": False}


if __name__ == "__main__":
    unittest.main()
