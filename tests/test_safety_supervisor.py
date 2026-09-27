import time
import unittest
from types import SimpleNamespace

from core.interfaces import (DecisionMode, DecisionTarget, Perception, Target, Trajectory,
                             TrajectoryPoint)
from core.safety_supervisor import SafetySupervisor


def chain(frame=1):
    p = Perception()
    p.valid, p.frame_id, p.timestamp = True, frame, frame * 1000
    p.valid_until = time.monotonic() + 20
    p.case_name, p.case_id = "06.车道居中控制", "case-06"
    p.scene_id = 6
    p.ego.valid, p.ego.frame_id, p.ego.gear = True, frame, 1
    p.ego.age_ms = 0
    p.targets_valid, p.lane.valid = True, True
    p.target_source = "sensor:perfectPerception1"
    p.source_status = {"gps": {"usable": True}, "targets": {"usable": True}}
    d = DecisionTarget().bind(p)
    d.valid = True
    t = Trajectory().bind(d)
    t.valid = True
    t.points = [TrajectoryPoint(0, 0, 1, 0, 0),
                TrajectoryPoint(1, 0, 1, 0, 1)]
    return p, d, t


class SafetySupervisorTests(unittest.TestCase):
    def setUp(self):
        self.now = time.monotonic()
        config = SimpleNamespace(pipeline_timeout_ms=200, sensor_timeout_ms=500,
                                 safety_recovery_frames=3,
                                 safety_controlled_brake=0.3,
                                 safety_emergency_brake=1.0)
        self.safety = SafetySupervisor(config, clock=lambda: self.now)

    def test_optional_sensor_error_does_not_block_normal_chain(self):
        p, d, t = chain()
        p.sensor_errors = ["imu:timing_unknown", "radar:unavailable"]
        self.assertEqual("normal", self.safety.evaluate(p, d, t).mode)

    def test_target_and_lane_loss_request_controlled_brake(self):
        for field in ("targets", "lane"):
            self.safety.reset()
            p, d, t = chain()
            if field == "targets":
                p.scene_id = 1
                p.targets_valid = False
            else:
                p.lane.valid = False
            result = self.safety.evaluate(p, d, t)
            self.assertEqual("controlled_stop", result.mode)
            self.assertTrue(result.control.valid)
            self.assertEqual(0.0, result.control.throttle)
            self.assertEqual(0.3, result.control.brake)

    def test_unconfigured_target_sensor_does_not_stop_lane_scene(self):
        p, d, t = chain()
        p.targets_valid = False
        p.source_status["targets"] = {"usable": False,
                                       "sensor_presence": "not_configured"}
        self.assertEqual("normal", self.safety.evaluate(p, d, t).mode)

    def test_ground_truth_diagnostic_does_not_trigger_collision_override(self):
        p, d, t = chain()
        p.target_source = "ground_truth"
        target = Target()
        target.valid, target.lateral_band_match = True, True
        target.longitudinal_distance, target.ttc = 4.0, 1.0
        p.targets = [target]
        self.assertEqual("normal", self.safety.evaluate(p, d, t).mode)

    def test_collision_request_uses_emergency_brake(self):
        p, d, t = chain()
        d.mode = DecisionMode.EMERGENCY_BRAKE
        result = self.safety.evaluate(p, d, t)
        self.assertEqual("emergency_stop", result.mode)
        self.assertEqual(1.0, result.control.brake)

    def test_imminent_target_still_emergency_when_map_lane_is_lost(self):
        p, d, t = chain()
        p.lane.valid = False
        target = Target()
        target.valid, target.lateral_band_match = True, True
        target.longitudinal_distance, target.ttc = 4.0, 1.0
        p.targets = [target]
        result = self.safety.evaluate(p, d, t)
        self.assertEqual("emergency_stop", result.mode)
        self.assertEqual("imminent_target_collision", result.reason)

    def test_malformed_target_does_not_crash_safety_gate(self):
        p, d, t = chain()
        p.scene_id = 1
        p.targets = [SimpleNamespace(valid=True, lateral_band_match=True,
                                     longitudinal_distance=4.0, ttc=None)]
        self.assertEqual("controlled_stop", self.safety.evaluate(p, d, t).mode)

    def test_invalid_trajectory_and_gps_use_bounded_fault_candidate(self):
        p, d, t = chain()
        self.safety.evaluate(p, d, t)
        p2, d2, t2 = chain(2)
        t2.valid = False
        result = self.safety.evaluate(p2, d2, t2)
        self.assertEqual("fault_stop", result.mode)
        self.assertIsNotNone(result.control)
        p3, d3, t3 = chain(3)
        p3.valid, p3.ego.valid = False, False
        result = self.safety.evaluate(p3, d3, t3)
        self.assertEqual("gps_invalid_or_expired", result.reason)
        self.assertEqual(2, result.control.frame_id)
        self.now += 0.49
        late = self.safety.evaluate(p3, d3, t3)
        self.assertLessEqual(late.control.valid_until, self.safety.last_good_at + 0.5)
        self.now += 0.6
        self.assertIsNone(self.safety.evaluate(p3, d3, t3).control)

    def test_repeated_frame_fault_and_recovery_need_distinct_fresh_frames(self):
        p, d, t = chain()
        self.assertEqual("normal", self.safety.evaluate(p, d, t).mode)
        self.now += 0.21
        result = self.safety.evaluate(p, d, t)
        self.assertEqual("fault_stop", result.mode)
        self.assertEqual("gps_frame_stalled", result.reason)
        for frame in (2, 3):
            p, d, t = chain(frame)
            self.assertEqual("controlled_stop", self.safety.evaluate(p, d, t).mode)
        p, d, t = chain(4)
        self.assertEqual("normal", self.safety.evaluate(p, d, t).mode)

    def test_invalid_brake_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            SafetySupervisor(SimpleNamespace(safety_controlled_brake=float("nan")))

    def test_case_change_discards_old_header_and_requires_recovery(self):
        p, d, t = chain()
        self.assertEqual("normal", self.safety.evaluate(p, d, t).mode)
        p2, d2, t2 = chain(2)
        p2.case_id = "new-case"
        self.assertEqual("controlled_stop", self.safety.evaluate(p2, d2, t2).mode)
        self.assertEqual(2, self.safety.last_good_header[0])


if __name__ == "__main__":
    unittest.main()
