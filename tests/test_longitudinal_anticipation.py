"""Brake preview must survive accumulated propulsion demand on replans."""
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
        p, _ = inputs(speed=6, x=0, y=0)
        t = Trajectory().bind(p)
        t.valid, t.target_speed = True, 6.0
        t.stop_required, t.stop_distance = True, 10.0
        t.points = [TrajectoryPoint(0, 0, 6, 0, 0),
                    TrajectoryPoint(1, 0, 5.8, 0, .17),
                    TrajectoryPoint(5, 0, 4, 0, 1),
                    TrajectoryPoint(10, 0, 0, 0, 3.5)]
        engine = ControlEngine(calibration())
        engine.integral = 3.4383475565878467
        c = engine.compute(p, t)
        self.assertTrue(c.valid, c.errors)
        self.assertLess(c.diagnostics['reference_acceleration_mps2'], 0)
        self.assertEqual(0, c.throttle)
        self.assertGreater(c.brake, 0)
