"""Regressions for rolling trajectories, repeated GPS and stop transitions."""

import math
import time
import unittest

from core.interfaces import DecisionTarget
from members.control.controller import ControlEngine
from members.control.parameters import ControllerSettings
from members.control.trajectory import PreparedPath
from members.planning_stub import plan
from tests.test_control import inputs, calibration


class ControlRegressionTests(unittest.TestCase):
    def setUp(self):
        self.now = time.monotonic()
        self.engine = ControlEngine(calibration(), clock=lambda: self.now)

    def advance(self):
        self.now += 0.05


    def test_hold_releases_for_planner_with_future_terminal_stop(self):
        p, t = inputs(speed=0)
        t.target_speed = 0.0
        self.assertEqual("HOLD", self.engine.compute(p, t).diagnostics["state"])
        outputs = []
        for frame in range(2, 16):
            self.advance()
            p, _ = inputs(frame=frame, speed=0)
            p.lane.valid, p.lane.lane_id = True, "lane"
            p.scene_id = 6
            p.lane.center_line = [(float(i), 0.0, 0.0) for i in range(51)]
            d = DecisionTarget().bind(p)
            d.valid, d.target_speed, d.target_lane_id = True, 2.0, "lane"
            t = plan(p, d)
            self.assertTrue(t.valid, t.errors)
            self.assertTrue(t.stop_required)
            outputs.append(self.engine.compute(p, t))
        self.assertTrue(all(o.valid for o in outputs))
        self.assertGreater(outputs[-1].throttle, 0.0)
        self.assertEqual("TRACK", outputs[-1].diagnostics["state"])


    def test_duplicate_frame_does_not_reset_control_memory(self):
        p, t = inputs(speed=4.5)
        self.engine.compute(p, t)
        self.advance()
        p, t = inputs(frame=2, speed=4.5)
        previous = self.engine.compute(p, t)
        memory = (self.engine.integral, self.engine.state, self.engine.last_time)
        for advance in (0.0, 0.02):
            self.now += advance
            duplicate = self.engine.compute(p, t)
            self.assertFalse(duplicate.valid)
            self.assertEqual(memory, (self.engine.integral, self.engine.state,
                                      self.engine.last_time))
            self.assertIs(previous, self.engine.last_output)
        p.valid_until = time.monotonic() - 1.0
        self.assertFalse(self.engine.compute(p, t).valid)
        self.assertEqual("FAULT_STOP", self.engine.state)


    def test_stop_intent_never_accelerates_from_stored_integral(self):
        self.engine.integral = 0.0
        p, t = inputs(speed=3)
        self.engine.compute(p, t)
        self.engine.integral = 2.0
        self.advance()
        p, t = inputs(frame=2, speed=3)
        t.stop_required, t.target_speed, t.stop_distance = True, 0.0, 50.0
        result = self.engine.compute(p, t)
        self.assertTrue(result.valid)
        self.assertEqual(0.0, result.throttle)
        self.assertLessEqual(self.engine.integral, 0.0)


    def test_near_path_end_still_produces_braking_command(self):
        p, t = inputs(speed=0.7, x=79.83)
        t.stop_required, t.target_speed, t.stop_distance = True, 0.0, 0.17
        result = self.engine.compute(p, t)
        self.assertTrue(result.valid, result.errors)
        self.assertEqual(0.0, result.throttle)
        self.assertGreater(result.brake, 0.0)
        self.assertEqual("STOPPING", result.diagnostics["state"])


    def test_interior_zero_speed_constraint_is_not_used_as_launch(self):
        points = [(float(i), 0.0, 0.0 if i == 5 else 2.0) for i in range(21)]
        p, t = inputs(speed=0, x=5, points=points)
        t.target_speed = 2.0
        result = self.engine.compute(p, t)
        self.assertTrue(result.valid, result.errors)
        self.assertEqual(0.0, result.throttle)
        self.assertEqual("HOLD", result.diagnostics["state"])


    def test_integral_does_not_grow_while_hold_brake_is_releasing(self):
        p, t = inputs(speed=0)
        t.target_speed = 0.0
        self.engine.compute(p, t)
        outputs = []
        for frame in range(2, 5):
            self.advance()
            p, t = inputs(frame=frame, speed=0)
            outputs.append(self.engine.compute(p, t))
        self.assertGreater(outputs[-1].brake, 0.0)
        self.assertEqual(0.0, self.engine.integral)


    def test_positive_global_cap_does_not_create_false_deceleration(self):
        p, t = inputs(speed=2.8, x=0)
        t.target_speed = 3.0  # Flat point speeds of 5; global cap of 3.
        result = self.engine.compute(p, t)
        self.assertTrue(result.valid, result.errors)
        self.assertEqual(0.0, result.diagnostics["reference_acceleration_mps2"])
        self.assertGreater(result.throttle, 0.0)
        self.assertEqual(0.0, result.brake)

    def test_curve_speed_floor_cannot_override_lateral_acceleration(self):
        points = [(0.2*math.cos(i*0.1), 0.2*math.sin(i*0.1), 5.0)
                  for i in range(21)]
        p, t = inputs(points=points)
        path = PreparedPath(t.points, 12.0)
        limit = path.curve_speed_limit(0.0, ControllerSettings())
        self.assertLessEqual(limit*limit*5.0, 1.5 + 1e-6)

    def test_invalid_algorithm_settings_are_rejected(self):
        for field, value in (("pi_kp", float("nan")), ("hold_release_frames", 0),
                             ("steering_preview_s", 1.0), ("acceleration_preview_m", 0.0)):
            settings = ControllerSettings()
            setattr(settings, field, value)
            with self.assertRaises(ValueError):
                ControlEngine(calibration(), settings)

    def test_feedforward_respects_curve_speed_envelope(self):
        points = [(10*math.cos(i*0.05), 10*math.sin(i*0.05), 8.0)
                  for i in range(32)]
        p, t = inputs(speed=3.0, x=10*math.cos(0.5), y=10*math.sin(0.5),
                      heading=0.5+math.pi/2, points=points)
        t.target_speed = 8.0
        result = self.engine.compute(p, t)
        self.assertTrue(result.valid, result.errors)
        self.assertLessEqual(result.diagnostics["reference_acceleration_mps2"], 1e-6)


if __name__ == "__main__":
    unittest.main()
