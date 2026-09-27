import math
import unittest

from core.geometry import calculate_ttc, normalize_angle, world_to_ego
from core.interfaces import ControlOut
from core.scene_requirements import requires_targets
from simone_platform.case_resolver import resolve_scene_id


class CoreTests(unittest.TestCase):
    def test_target_stream_is_required_only_by_relevant_scenes(self):
        for scene_id in (4, 5, 6, 7, 8, 9, 10, 19, 20, 29):
            self.assertFalse(requires_targets(scene_id))
        for scene_id in (1, 3, 11, 18, 21, 28, 30, 41):
            self.assertTrue(requires_targets(scene_id))
        self.assertTrue(requires_targets(0))

    def test_scene_id_from_chinese_case_name(self):
        self.assertEqual(4, resolve_scene_id("04.直道车道偏离抑制-测试"))
        self.assertEqual(6, resolve_scene_id("06_车道居中控制"))
        self.assertEqual(0, resolve_scene_id("未编号场景"))
        self.assertEqual(9, resolve_scene_id("04.场景", 9))

    def test_world_to_ego(self):
        longitudinal, lateral = world_to_ego(0.0, 0.0, math.pi / 2.0, 0.0, 10.0)
        self.assertAlmostEqual(10.0, longitudinal, places=5)
        self.assertAlmostEqual(0.0, lateral, places=5)

    def test_ttc(self):
        ttc, closing = calculate_ttc(20.0, 10.0, 5.0, 0.0, 0.0)
        self.assertAlmostEqual(4.0, ttc)
        self.assertAlmostEqual(5.0, closing)

    def test_control_clamp(self):
        output = ControlOut()
        output.throttle = 2.0
        output.brake = -1.0
        output.steering = -3.0
        output.clamp()
        self.assertEqual(1.0, output.throttle)
        self.assertEqual(0.0, output.brake)
        self.assertEqual(-1.0, output.steering)


if __name__ == "__main__":
    unittest.main()
