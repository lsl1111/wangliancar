import unittest

from core.interfaces import LaneContext
from perception.perception_builder import PerceptionBuilder


class FakeAdapter(object):
    def get_case_info(self):
        return {"case_name": "06.车道居中控制-测试"}


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
        }
        value = self.builder.build_from_raw(raw)
        self.assertTrue(value.valid)
        self.assertEqual(6, value.scene_id)
        self.assertEqual("1_0_-1", value.lane.lane_id)
        self.assertEqual(1, len(value.targets))
        self.assertTrue(value.targets[0].same_lane)
        self.assertAlmostEqual(4.0, value.targets[0].ttc)

    def test_gps_missing_is_invalid(self):
        value = self.builder.build_from_raw({"gps": None, "targets": []})
        self.assertFalse(value.valid)
        self.assertIn("GPS_UNAVAILABLE", value.errors)


if __name__ == "__main__":
    unittest.main()

