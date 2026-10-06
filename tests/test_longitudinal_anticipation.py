"""Brake preview must survive accumulated propulsion demand on replans."""
import math
import unittest

from core.interfaces import Trajectory, TrajectoryPoint
from members.control.controller import ControlEngine
from members.control.longitudinal import pedal_request
from members.control.parameters import ControllerSettings
from tests.test_control import calibration, inputs


class LongitudinalAnticipationTests(unittest.TestCase):
    def test_captured_pre_curve_brake_demand_is_not_cancelled_by_old_integral(self):
        # Exact diagnostic values from scene 15 frame 1050, PID 13728.
        speed, integral, acceleration = 6.5796392398436545, 3.4383475565878467, -1.516876342621341
        throttle, brake, next_integral, error = pedal_request(
            speed, speed, integral, .11, ControllerSettings(), calibration(), acceleration)
        self.assertEqual(0, throttle)
        self.assertGreater(brake, 0)
        self.assertEqual(0, next_integral)
        self.assertEqual(0, error)

    def test_braking_reset_survives_saturation_but_steady_grade_compensation_remains(self):
        settings = ControllerSettings()
        throttle, brake, integral, _ = pedal_request(
            6, 5, 6, .1, settings, calibration(), -2)
        self.assertEqual(0, throttle)
        self.assertEqual(calibration().max_brake, brake)
        self.assertEqual(0, integral)
        throttle, brake, integral, _ = pedal_request(
            5, 5, 6, .1, settings, calibration(), 0)
        self.assertGreater(throttle, 0)
        self.assertEqual(0, brake)
        self.assertEqual(6, integral)

    def test_well_below_falling_reference_does_not_need_unnecessary_braking(self):
        throttle, brake, integral, _ = pedal_request(
            3, 6, 2, .1, ControllerSettings(), calibration(), -1)
        self.assertGreater(throttle, 0)
        self.assertEqual(0, brake)
        self.assertGreater(integral, 0)

    def test_falling_stop_profile_brakes_with_positive_target_speed_and_old_integral(self):
        # Public trajectory encoding permits positive target speed while
        # approaching a finite stop; braking must not require a zero target.
        speed = 6.5796392398436545
        acceleration = -1.516876342621341
        p, _ = inputs(speed=speed, x=0, y=0)
        p.ego.vx = speed
        now = [0.0]
        engine = ControlEngine(calibration(), clock=lambda: now[0])
        # Initialize the same case before seeding old propulsion state. A seed
        # before the first compute is cleared by case reset and misses the bug.
        warm = Trajectory().bind(p)
        warm.valid, warm.target_speed = True, speed
        warm.points = [TrajectoryPoint(i, 0, speed, 0, i / speed)
                       for i in range(101)]
        self.assertTrue(engine.compute(p, warm).valid)
        engine.integral = 3.4383475565878467
        now[0] = .11
        p.frame_id += 1
        p.ego.frame_id = p.frame_id
        p.timestamp += 110
        t = Trajectory().bind(p)
        t.valid, t.target_speed = True, speed
        t.stop_required = True
        t.stop_distance = speed * speed / (-2 * acceleration)
        positions = [float(i) for i in range(int(t.stop_distance) + 1)]
        positions.append(t.stop_distance)
        for distance in positions:
            point_speed = math.sqrt(max(0.0, speed * speed + 2 * acceleration * distance))
            elapsed = (speed - point_speed) / (-acceleration)
            t.points.append(TrajectoryPoint(distance, 0, point_speed, 0, elapsed))
        c = engine.compute(p, t)
        self.assertTrue(c.valid, c.errors)
        self.assertAlmostEqual(acceleration, c.diagnostics['reference_acceleration_mps2'])
        self.assertAlmostEqual(speed, c.diagnostics['reference_speed_mps'])
        self.assertEqual(0, engine.integral)
        self.assertEqual(0, c.throttle)
        self.assertGreater(c.brake, 0)
