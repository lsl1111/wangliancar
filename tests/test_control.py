"""Offline controller checks use explicit test-only vehicle calibration."""

import math
import os
import tempfile
import time
import unittest
from types import SimpleNamespace

from core.config import load_config
from core.interfaces import Perception, Trajectory, TrajectoryPoint
from members.control.controller import ControlEngine
from members.control.longitudinal import pedal_request
from members.control.parameters import ControllerSettings, VehicleCalibration
from members.control_stub import compute_control, configure_control


def calibration():
    # Exercise the mapping mathematically; these are not SimOne vehicle values.
    return VehicleCalibration(2.7, 0.5, 1, 0.3, 0.4, 0.2, 0.9, 0.4, 0.6)


def inputs(frame=1, speed=2.0, heading=0.0, x=5.0, y=0.0,
           points=None):
    p = Perception()
    p.valid, p.frame_id, p.timestamp = True, frame, frame * 1000
    p.valid_until = time.monotonic() + 20
    p.case_id, p.case_name, p.scene_id = "case-06", "06.车道居中控制", 6
    p.ego.valid, p.ego.frame_id, p.ego.age_ms = True, frame, 0
    p.ego.gear = 1
    p.ego.x, p.ego.y, p.ego.heading, p.ego.speed = x, y, heading, speed
    p.source_status = {"gps": {"usable": True},
                       "targets": {"usable": False,
                                   "sensor_presence": "not_configured"}}
    t = Trajectory().bind(p)
    t.valid, t.target_speed = True, 5.0
    if points is None:
        points = [(float(i), 0.0, 5.0) for i in range(81)]
    t.points = [TrajectoryPoint(px, py, v, 0.0, index + 0.1)
                for index, (px, py, v) in enumerate(points)]
    return p, t


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.now = time.monotonic()
        self.engine = ControlEngine(calibration(), clock=lambda: self.now)

    def advance(self):
        self.now += 0.05

    def test_left_right_and_rotated_path_steering_sign(self):
        left = [(float(i), 1.0, 5.0) for i in range(81)]
        right = [(float(i), -1.0, 5.0) for i in range(81)]
        p, t = inputs(points=left)
        left_output = self.engine.compute(p, t)
        self.assertTrue(left_output.valid, left_output.errors)
        self.assertGreater(left_output.steering, 0)
        other = ControlEngine(calibration(), clock=lambda: self.now)
        p, t = inputs(points=right)
        self.assertLess(other.compute(p, t).steering, 0)
        rotated = [(-1.0, float(i), 5.0) for i in range(81)]
        p, t = inputs(heading=math.pi / 2, x=0, y=5, points=rotated)
        self.assertGreater(ControlEngine(calibration(), clock=lambda: self.now)
                           .compute(p, t).steering, 0)

    def test_accelerate_brake_and_pedal_exclusivity(self):
        p, t = inputs(speed=0)
        first = self.engine.compute(p, t)
        self.assertTrue(first.valid)
        self.assertGreater(first.throttle, 0)
        self.assertEqual(0, first.brake)
        self.assertLessEqual(first.throttle, 0.4)
        self.advance()
        p, t = inputs(frame=2, speed=10)
        second = self.engine.compute(p, t)
        self.assertTrue(second.valid)
        self.assertEqual(0, second.throttle)
        self.assertGreater(second.brake, 0)
        self.assertLessEqual(second.brake, 0.6)

    def test_old_positive_integral_cannot_accelerate_while_overspeed(self):
        throttle, brake, integral, error = pedal_request(
            3.2, 3.0, 3.0, 0.05, ControllerSettings(), calibration())
        self.assertEqual(0, throttle)
        self.assertGreater(brake, 0)
        self.assertEqual(0, integral)

    def test_stop_brakes_then_holds_and_releases_with_hysteresis(self):
        p, t = inputs(speed=3)
        t.stop_required, t.target_speed, t.stop_distance = True, 0.0, 1.0
        stopping = self.engine.compute(p, t)
        self.assertTrue(stopping.valid)
        self.assertEqual("STOPPING", stopping.diagnostics["state"])
        self.assertEqual(0, stopping.throttle)
        self.assertGreater(stopping.brake, 0)
        self.advance()
        p, t = inputs(frame=2, speed=0)
        t.stop_required, t.target_speed, t.stop_distance = True, 0.0, 0.0
        hold = self.engine.compute(p, t)
        self.assertEqual("HOLD", hold.diagnostics["state"])
        self.assertEqual(0.2, hold.brake)
        self.advance()
        p, t = inputs(frame=3, speed=0)
        pending = self.engine.compute(p, t)
        self.assertEqual("HOLD", pending.diagnostics["state"])
        self.advance()
        p, t = inputs(frame=4, speed=0)
        released = self.engine.compute(p, t)
        self.assertEqual("TRACK", released.diagnostics["state"])
        self.assertLess(released.brake, hold.brake)
        self.assertEqual(0, released.throttle)

    def test_path_end_and_future_slowdown_lower_speed_reference(self):
        p, t = inputs(speed=4, x=78)
        result = self.engine.compute(p, t)
        self.assertTrue(result.valid)
        self.assertLess(result.diagnostics["reference_speed_mps"], 4)
        self.assertGreater(result.brake, 0)
        points = [(float(i), 0.0, 5.0 if i < 8 else 0.5) for i in range(81)]
        p, t = inputs(points=points, speed=5)
        result = ControlEngine(calibration()).compute(p, t)
        self.assertLess(result.diagnostics["reference_speed_mps"], 5)

    def test_emergency_bypasses_degenerate_path_and_cuts_throttle(self):
        p, t = inputs(speed=4, points=[(5.0, 0.0, 0.0),
                                       (5.0, 0.0, 0.0)])
        t.emergency_stop, t.stop_required, t.target_speed = True, True, 0.0
        result = self.engine.compute(p, t)
        self.assertTrue(result.valid, result.errors)
        self.assertEqual("EMERGENCY", result.diagnostics["state"])
        self.assertEqual((0.0, 0.9, 0.0),
                         (result.throttle, result.brake, result.steering))

    def test_unconfigured_vehicle_has_diagnostics_but_no_valid_command(self):
        p, t = inputs()
        result = ControlEngine().compute(p, t)
        self.assertFalse(result.valid)
        self.assertIn("vehicle calibration required", result.errors)
        self.assertIn("curvature_m_inv", result.diagnostics)
        with self.assertRaises(ValueError):
            VehicleCalibration.from_app_config(
                SimpleNamespace(control_calibrated=True))

    def test_fixed_entry_uses_safe_default_configuration(self):
        project = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config = load_config(project)
        self.assertFalse(config.control_calibrated)
        self.assertFalse(config.send_control)
        configure_control(config)
        p, t = inputs()
        result = compute_control(p, t)
        self.assertFalse(result.valid)
        self.assertIn("reference_speed_mps", result.diagnostics)

    def test_calibrated_local_config_makes_candidate_valid_without_sending(self):
        project = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        values = {"wheelbase_m": 2.7, "front_steer_max_rad": 0.5,
                  "steering_sign": 1, "throttle_per_mps": 0.3,
                  "brake_per_mps": 0.4, "hold_brake": 0.2,
                  "emergency_brake": 0.9, "max_throttle": 0.4,
                  "max_brake": 0.6}
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "local.ini")
            with open(path, "w", encoding="utf-8") as stream:
                stream.write("[app]\ncontrol_calibrated = true\n")
                for name, value in values.items():
                    stream.write("control_{0} = {1}\n".format(name, value))
            config = load_config(project, path)
        self.assertFalse(config.send_control)
        configure_control(config)
        p, t = inputs()
        result = compute_control(p, t)
        self.assertTrue(result.valid, result.errors)

    def test_zero_target_speed_holds_without_explicit_stop_flag(self):
        p, t = inputs(speed=0)
        t.target_speed = 0.0
        result = self.engine.compute(p, t)
        self.assertEqual("HOLD", result.diagnostics["state"])
        self.assertEqual(0.2, result.brake)

    def test_simple_bicycle_model_reduces_straight_path_offset(self):
        x, y, heading, speed = 5.0, 0.8, 0.0, 2.0
        vehicle = calibration()
        for frame in range(1, 101):
            p, t = inputs(frame=frame, speed=speed, x=x, y=y, heading=heading)
            result = self.engine.compute(p, t)
            self.assertTrue(result.valid, result.errors)
            front_angle = result.steering * vehicle.front_steer_max_rad
            heading += speed / vehicle.wheelbase_m * math.tan(front_angle) * 0.05
            x += speed * math.cos(heading) * 0.05
            y += speed * math.sin(heading) * 0.05
            self.advance()
        self.assertLess(abs(y), 0.2)

    def test_simple_speed_plant_approaches_reference(self):
        x, speed = 5.0, 0.0
        for frame in range(1, 201):
            p, t = inputs(frame=frame, speed=speed, x=x)
            t.target_speed = 3.0
            result = self.engine.compute(p, t)
            self.assertTrue(result.valid, result.errors)
            speed = max(0.0, speed +
                        (1.5 * result.throttle - 2.5 * result.brake) * 0.05)
            x += speed * 0.05
            self.advance()
        self.assertLess(abs(speed - 3.0), 0.5)

    def test_bad_path_direction_crossing_gap_and_nonfinite_input(self):
        cases = (
            ([(float(i), 0.0, 5.0) for i in range(80, -1, -1)], "opposes"),
            ([(0, 0, 5), (30, 0, 5)], "gap"),
            ([(0, 0, 5), (8, 8, 5), (0, 8, 5), (8, 0, 5)], "ambiguous"),
        )
        for points, reason in cases:
            p, t = inputs(x=4, y=4, heading=math.pi / 4, points=points)
            if reason == "opposes":
                p.ego.x, p.ego.y, p.ego.heading = 5, 0, 0
            if reason == "gap":
                p.ego.x, p.ego.y, p.ego.heading = 5, 0, 0
            result = ControlEngine(calibration()).compute(p, t)
            self.assertFalse(result.valid)
            self.assertTrue(any(reason in item for item in result.errors), result.errors)
        p, t = inputs()
        p.ego.speed = float("nan")
        self.assertFalse(ControlEngine(calibration()).compute(p, t).valid)

    def test_mismatch_stale_reverse_repeat_and_time_regression_rejected(self):
        p, t = inputs()
        t.frame_id = 99
        self.assertFalse(self.engine.compute(p, t).valid)
        self.advance()
        p, t = inputs(frame=2)
        p.ego.gear = 2
        self.assertFalse(self.engine.compute(p, t).valid)
        self.advance()
        p, t = inputs(frame=3)
        p.valid_until = time.monotonic() - 1
        self.assertFalse(self.engine.compute(p, t).valid)
        self.advance()
        p, t = inputs(frame=4)
        self.assertTrue(self.engine.compute(p, t).valid)
        self.assertFalse(self.engine.compute(p, t).valid)
        self.advance()
        p, t = inputs(frame=3)
        self.assertFalse(self.engine.compute(p, t).valid)
        self.now += 1.0
        p, t = inputs(frame=5)
        self.assertFalse(self.engine.compute(p, t).valid)


if __name__ == "__main__":
    unittest.main()
