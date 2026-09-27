"""Offline controller checks use explicit test-only vehicle calibration."""

import math
import os
import tempfile
import time
import unittest
from types import SimpleNamespace

from core.config import load_config
from core.interfaces import DecisionMode, DecisionTarget, Perception, Trajectory, TrajectoryPoint
from members.control.controller import ControlEngine
from members.control.longitudinal import pedal_request
from members.control.parameters import ControllerSettings, VehicleCalibration
from members.control.trajectory import PreparedPath
from members.control_stub import compute_control, configure_control
from members.planning_stub import plan


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

    def test_curve_geometry_shortens_lookahead_and_limits_speed(self):
        radius = 10.0
        arc = [(radius * math.cos(i * 0.05),
                radius * math.sin(i * 0.05), 8.0) for i in range(32)]
        p, t = inputs(speed=6, x=radius, y=0, heading=math.pi / 2,
                      points=arc)
        t.target_speed = 8.0
        curved = self.engine.compute(p, t)
        self.assertTrue(curved.valid, curved.errors)
        self.assertAlmostEqual(0.1, curved.diagnostics["path_curvature_m_inv"],
                               delta=0.03)
        self.assertLess(curved.diagnostics["lookahead_m"], 6.2)
        self.assertLess(curved.diagnostics["curve_speed_limit_mps"], 4.2)
        self.assertEqual(curved.diagnostics["curve_speed_limit_mps"],
                         curved.diagnostics["reference_speed_mps"])
        self.assertGreater(curved.brake, 0)
        straight = [(float(i), 0.0, 8.0) for i in range(81)]
        p, t = inputs(speed=6, points=straight)
        t.target_speed = 8.0
        direct = ControlEngine(calibration()).compute(p, t)
        self.assertTrue(direct.valid, direct.errors)
        self.assertAlmostEqual(6.2, direct.diagnostics["lookahead_m"])
        self.assertEqual(8.0, direct.diagnostics["reference_speed_mps"])
        self.assertGreater(direct.throttle, 0)

    def test_upcoming_corner_limits_speed_before_turn(self):
        points = ([(float(i), 0.0, 8.0) for i in range(11)] +
                  [(10.0, float(i), 8.0) for i in range(1, 21)])
        p, t = inputs(speed=7, x=0, points=points)
        t.target_speed = 8.0
        result = self.engine.compute(p, t)
        self.assertTrue(result.valid, result.errors)
        self.assertLess(result.diagnostics["curve_speed_limit_mps"], 7.0)
        self.assertGreater(result.brake, 0)
        self.assertEqual(0, result.throttle)

    def test_curve_window_ignores_small_centerline_wobble(self):
        points = [(float(i), 0.02 * math.sin(2.0 * i), 8.0)
                  for i in range(81)]
        p, t = inputs(speed=6, x=0, points=points)
        t.target_speed = 8.0
        result = self.engine.compute(p, t)
        self.assertTrue(result.valid, result.errors)
        self.assertEqual(8.0, result.diagnostics["curve_speed_limit_mps"])

    def test_adaptive_lookahead_reduces_circle_tracking_error_in_bicycle_model(self):
        radius = 10.0
        arc = [(radius * math.cos(i * 0.05),
                radius * math.sin(i * 0.05), 8.0) for i in range(32)]
        vehicle = calibration()

        def tracking_error(gain):
            settings = ControllerSettings()
            settings.curve_lookahead_gain_m = gain
            now = [0.0]
            engine = ControlEngine(vehicle, settings, clock=lambda: now[0])
            x, y, heading = radius, 0.0, math.pi / 2.0
            for frame in range(1, 61):
                p, t = inputs(frame=frame, speed=3.5, x=x, y=y,
                              heading=heading, points=arc)
                t.target_speed = 8.0
                result = engine.compute(p, t)
                self.assertTrue(result.valid, result.errors)
                front_angle = result.steering * vehicle.front_steer_max_rad
                heading += (3.5 / vehicle.wheelbase_m *
                            math.tan(front_angle) * 0.05)
                x += 3.5 * math.cos(heading) * 0.05
                y += 3.5 * math.sin(heading) * 0.05
                now[0] += 0.05
            return abs(math.hypot(x, y) - radius)

        fixed_error = tracking_error(0.0)
        adaptive_error = tracking_error(4.0)
        self.assertLess(adaptive_error, fixed_error - 0.005)

    def test_emergency_bypasses_degenerate_path_and_cuts_throttle(self):
        p, t = inputs(speed=4, points=[(5.0, 0.0, 0.0),
                                       (5.0, 0.0, 0.0)])
        t.emergency_stop, t.stop_required, t.target_speed = True, True, 0.0
        result = self.engine.compute(p, t)
        self.assertTrue(result.valid, result.errors)
        self.assertEqual("EMERGENCY", result.diagnostics["state"])
        self.assertEqual((0.0, 0.9, 0.0),
                         (result.throttle, result.brake, result.steering))

    def test_stationary_stop_brakes_then_holds_without_forward_geometry(self):
        stationary_points = [(5.0, 0.0, 0.0), (5.0, 0.0, 0.0)]
        p, t = inputs(speed=3, points=stationary_points)
        t.stop_required, t.target_speed, t.stop_distance = True, 0.0, 0.0
        stopping = self.engine.compute(p, t)
        self.assertTrue(stopping.valid, stopping.errors)
        self.assertEqual("STOPPING", stopping.diagnostics["state"])
        self.assertEqual((0.0, 0.6, 0.0),
                         (stopping.throttle, stopping.brake, stopping.steering))
        self.assertEqual(0.0, stopping.diagnostics["reference_speed_mps"])
        self.advance()
        p, t = inputs(frame=2, speed=0, points=stationary_points)
        t.stop_required, t.target_speed, t.stop_distance = True, 0.0, 0.0
        holding = self.engine.compute(p, t)
        self.assertTrue(holding.valid, holding.errors)
        self.assertEqual("HOLD", holding.diagnostics["state"])
        self.assertEqual((0.0, 0.2, 0.0),
                         (holding.throttle, holding.brake, holding.steering))

    def test_stationary_stop_requires_nearby_position_and_calibration(self):
        points = [(20.0, 0.0, 0.0), (20.0, 0.0, 0.0)]
        p, t = inputs(speed=0, points=points)
        t.stop_required, t.target_speed = True, 0.0
        result = self.engine.compute(p, t)
        self.assertFalse(result.valid)
        self.assertIn("too far", result.errors[0])
        p, t = inputs(speed=0, points=[(5.0, 0.0, 0.0),
                                      (5.0, 0.0, 0.0)])
        t.stop_required, t.target_speed = True, 0.0
        result = ControlEngine().compute(p, t)
        self.assertFalse(result.valid)
        self.assertIn("vehicle calibration required", result.errors)

    def test_planner_stationary_stop_reaches_valid_control(self):
        p, _ = inputs(speed=0)
        p.lane.valid = True
        p.lane.lane_id = "lane-1"
        p.lane.lane_width = 3.5
        p.lane.center_line = [(float(i), 0.0, 0.0) for i in range(81)]
        decision = DecisionTarget().bind(p)
        decision.valid = True
        decision.mode = DecisionMode.STOP
        decision.target_speed = 0.0
        decision.target_lane_id = p.lane.lane_id
        decision.stop_distance = 0.0
        trajectory = plan(p, decision)
        self.assertTrue(trajectory.valid, trajectory.errors)
        self.assertFalse(trajectory.emergency_stop)
        result = self.engine.compute(p, trajectory)
        self.assertTrue(result.valid, result.errors)
        self.assertEqual("HOLD", result.diagnostics["state"])
        self.assertEqual(0.2, result.brake)

    def test_duplicate_path_point_preserves_lower_speed_limit(self):
        points = [(0.0, 0.0, 5.0), (1.0, 0.0, 5.0),
                  (1.0, 0.0, 0.0), (2.0, 0.0, 5.0)]
        p, t = inputs(speed=2, x=0, points=points)
        path = PreparedPath(t.points, ControllerSettings().max_segment_m)
        self.assertEqual(0.0, path.points[1][2])
        reference, _ = path.speed_reference(0.0, t, ControllerSettings())
        self.assertLess(reference, 5.0)

    def test_unconfigured_vehicle_has_diagnostics_but_no_valid_command(self):
        p, t = inputs()
        result = ControlEngine().compute(p, t)
        self.assertFalse(result.valid)
        self.assertIn("vehicle calibration required", result.errors)
        self.assertIn("curvature_m_inv", result.diagnostics)
        with self.assertRaises(ValueError):
            VehicleCalibration.from_app_config(
                SimpleNamespace(control_calibrated=True))

    def test_fixed_entry_uses_observe_only_trial_configuration(self):
        project = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config = load_config(project)
        self.assertTrue(config.control_calibrated)
        self.assertFalse(config.send_control)
        self.assertFalse(config.safety_brake_enabled)
        self.assertAlmostEqual(2.9187, config.control_wheelbase_m)
        self.assertEqual(-1, config.control_steering_sign)
        configure_control(config)
        p, t = inputs()
        result = compute_control(p, t)
        self.assertTrue(result.valid, result.errors)
        self.assertIn("reference_speed_mps", result.diagnostics)
        self.assertLessEqual(result.throttle, config.control_max_throttle)
        left_path = [(float(i), 1.0, 2.0) for i in range(81)]
        p, t = inputs(points=left_path)
        left = ControlEngine(VehicleCalibration.from_app_config(config)).compute(p, t)
        self.assertTrue(left.valid, left.errors)
        self.assertLess(left.steering, 0.0)

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
