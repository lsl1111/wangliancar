"""Decision integration contracts; no SDK calls or actuator sends."""

import time
import unittest

from core.interfaces import DecisionMode, Perception, Target
from members.decision.engine import DecisionEngine
from members.decision.settings import DecisionSettings


def sample(frame=10, speed=0.0, scene=1, targets_usable=True):
    p = Perception()
    p.frame_id = p.ego.frame_id = frame
    p.timestamp = p.ego.timestamp = frame * 100
    p.valid_until = time.monotonic() + 20.0
    p.valid = p.ego.valid = True
    p.ego.speed = p.ego.vx = speed
    p.scene_id = scene
    p.case_id = "case-a"
    p.task_id = "task-a"
    p.lane.valid = True
    p.lane.lane_id = "lane-a"
    p.lane.center_line = [(0.0, 0.0), (100.0, 0.0)]
    p.targets_valid = targets_usable
    p.target_source = "sensor:front" if targets_usable else "none"
    p.source_status = {
        "gps": {"usable": True, "frame_id": frame, "timestamp": p.timestamp},
        "targets": {"usable": targets_usable, "frame_id": frame,
                    "timestamp": p.timestamp},
    }
    return p


def lead(p, speed=6.0):
    target = Target()
    target.valid = True
    target.id = 1
    target.lane_id = p.lane.lane_id
    target.same_lane_valid = target.same_lane = True
    target.longitudinal_distance = target.x = 80.0
    target.length = 4.0
    target.width = 1.8
    target.vx = speed
    p.targets = [target]
    return p


class DecisionCompatibilityTests(unittest.TestCase):
    def setUp(self):
        self.engine = DecisionEngine(DecisionSettings(cruise_speed=3.0,
                                                        front_offset_m=3.5))

    def test_raw_gps_gear_is_not_command_enum(self):
        for raw in (-1, 2, 4):
            p = sample(scene=6)
            p.ego.gear = raw
            self.assertEqual(DecisionMode.KEEP_LANE, self.engine.run(p).mode)
        p.ego.gear = True
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, self.engine.run(p).mode)

    def test_gps_source_gate_is_required_even_when_overall_valid(self):
        p = sample(speed=3.0)
        p.source_status["gps"]["usable"] = False
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, self.engine.run(p).mode)

    def test_optional_missing_targets_allow_repeated_lane_progress(self):
        for frame in range(10, 15):
            p = sample(frame, scene=6, targets_usable=False)
            self.assertEqual(DecisionMode.KEEP_LANE, self.engine.run(p).mode)
        self.assertFalse(self.engine._blind_stop)

    def test_startup_invalid_gps_can_recover_without_optional_targets(self):
        p = sample(scene=6, targets_usable=False)
        p.ego.valid = False
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, self.engine.run(p).mode)
        for frame in (11, 12):
            self.assertEqual(DecisionMode.STOP, self.engine.run(
                sample(frame, scene=6, targets_usable=False)).mode)
        self.assertEqual(DecisionMode.KEEP_LANE, self.engine.run(
            sample(13, scene=6, targets_usable=False)).mode)

    def test_repeated_gps_frame_does_not_count_multiple_target_faults(self):
        p = sample(targets_usable=False)
        for _ in range(5):
            self.assertEqual(DecisionMode.STOP, self.engine.run(p).mode)
        self.assertEqual(1, self.engine._blind_fault_count)
        p = sample(11, targets_usable=False)
        self.assertEqual(DecisionMode.STOP, self.engine.run(p).mode)
        self.assertTrue(self.engine._blind_stop)

    def test_good_empty_sensor_frame_clears_consecutive_fault_count(self):
        self.engine.run(sample(10, targets_usable=False))
        self.engine.run(sample(11))
        result = self.engine.run(sample(12, targets_usable=False))
        self.assertEqual(DecisionMode.STOP, result.mode)
        self.assertEqual(1, self.engine._blind_fault_count)
        self.assertFalse(self.engine._blind_stop)

    def test_unusable_target_list_is_not_used_for_following(self):
        for source in ("none", "ground_truth", "sensor:front"):
            engine = DecisionEngine(DecisionSettings(cruise_speed=3.0))
            p = lead(sample(scene=6))
            p.target_source = source
            p.source_status["targets"]["usable"] = False
            self.assertEqual(DecisionMode.KEEP_LANE, engine.run(p).mode)

    def test_ground_truth_is_never_promoted_to_sensor_evidence(self):
        for frame in (10, 11):
            p = lead(sample(frame))
            p.target_source = "ground_truth"
            result = self.engine.run(p)
        self.assertEqual(DecisionMode.STOP, result.mode)

    def test_recovering_blind_stop_cannot_be_bypassed_by_lead(self):
        self.engine.run(sample(10, speed=3.0, targets_usable=False))
        self.engine.run(sample(11, speed=3.0, targets_usable=False))
        moving = self.engine.run(lead(sample(12, speed=3.0)))
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, moving.mode)
        for frame in (13, 14):
            held = self.engine.run(lead(sample(frame, speed=0.0)))
            self.assertEqual(DecisionMode.STOP, held.mode)
        recovered = self.engine.run(lead(sample(15, speed=0.0)))
        self.assertEqual(DecisionMode.FOLLOW, recovered.mode)
        self.assertFalse(self.engine._blind_stop)

    def test_new_case_task_scene_reset_previous_blind_stop(self):
        for field, value in (("case_id", "case-b"), ("task_id", "task-b"),
                             ("scene_id", 2)):
            engine = DecisionEngine(DecisionSettings(cruise_speed=3.0))
            engine.run(sample(10, speed=3.0, targets_usable=False))
            engine.run(sample(11, speed=3.0, targets_usable=False))
            p = sample(12, speed=3.0)
            setattr(p, field, value)
            self.assertEqual(DecisionMode.KEEP_LANE, engine.run(p).mode)

    def test_frame_rollback_resets_previous_run_state(self):
        self.engine.run(sample(10, speed=3.0, targets_usable=False))
        self.engine.run(sample(11, speed=3.0, targets_usable=False))
        self.assertEqual(DecisionMode.KEEP_LANE,
                         self.engine.run(sample(1, speed=3.0)).mode)

    def test_following_speed_caps_cruise_and_published_limit(self):
        p = lead(sample(), speed=20.0)
        p.lane.speed_limit = 2.0
        self.assertEqual(2.0, self.engine.run(p).target_speed)
        p.lane.speed_limit = -1.0
        self.assertEqual(3.0, self.engine.run(p).target_speed)


if __name__ == "__main__":
    unittest.main()
