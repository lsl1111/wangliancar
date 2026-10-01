"""Closed-loop regressions with test-only actuator lag and a bicycle model."""

import math
import unittest

from members.control.controller import ControlEngine
from members.control.parameters import ControllerSettings
from tests.control_benchmark import Plant, frame_input, simulate, vehicle


class ClosedLoopTests(unittest.TestCase):
    def test_steering_preview_improves_curves_across_actuator_lags(self):
        for radius, speed in ((10.0, 3.0), (25.0, 6.0)):
            points = [(radius*math.cos(i*0.025), radius*math.sin(i*0.025), speed)
                      for i in range(121)]
            for lag in (0.05, 0.1, 0.3, 0.5):
                old = ControllerSettings()
                old.steering_preview_s = 0.0
                baseline = simulate("no_preview", points,
                                    (radius, 0.0, math.pi/2, speed), speed,
                                    duration=7.0, lag=lag, settings=old)
                improved = simulate("preview", points,
                                    (radius, 0.0, math.pi/2, speed), speed,
                                    duration=7.0, lag=lag)
                self.assertEqual([], improved["failures"])
                self.assertLess(improved["max_cross_track_m"],
                                baseline["max_cross_track_m"])

    def test_delayed_brakes_stop_before_end_with_profile_feedforward(self):
        points = [(i*0.25, 0.0, min(4.0, math.sqrt(max(0.0, 2*(20-i*0.25)))))
                  for i in range(81)]
        for lag in (0.1, 0.3, 0.5):
            result = simulate("stop", points, (0.0, 0.0, 0.0, 4.0), 0.0,
                              duration=15.0, stop_x=20.0, lag=lag)
            self.assertEqual([], result["failures"])
            self.assertEqual(0.0, result["final_speed_mps"])
            self.assertGreater(result["final_x_m"], 19.0)
            self.assertLess(result["final_x_m"], 20.0)
        without = ControllerSettings()
        without.acceleration_feedforward_s = 0.0
        baseline = simulate("no_ff", points, (0.0, 0.0, 0.0, 4.0), 0.0,
                            duration=15.0, stop_x=20.0, lag=0.3, settings=without)
        improved = simulate("ff", points, (0.0, 0.0, 0.0, 4.0), 0.0,
                            duration=15.0, stop_x=20.0, lag=0.3)
        self.assertLess(abs(improved["final_x_m"] - 19.5),
                        0.5 * abs(baseline["final_x_m"] - 19.5))

    def test_repeated_observations_do_not_destroy_speed_tracking(self):
        points = [(float(i), 0.0, 3.0) for i in range(151)]
        regular = simulate("regular", points, (0.0, 0.0, 0.0, 0.0), 3.0, duration=20.0)
        repeated = simulate("repeated", points, (0.0, 0.0, 0.0, 0.0), 3.0,
                            duration=20.0, duplicate=True)
        self.assertEqual([], repeated["failures"])
        self.assertLess(repeated["tail_mean_speed_error_mps"], 0.1)
        self.assertAlmostEqual(regular["final_speed_mps"],
                               repeated["final_speed_mps"], places=6)

    def test_irregular_loop_intervals_preserve_lateral_and_speed_tracking(self):
        plant = Plant(y=0.6, heading=0.05, lag=0.3)
        calibration = vehicle()
        now = [0.0]
        engine = ControlEngine(calibration, clock=lambda: now[0])
        points = [(float(i), 0.0, 3.0) for i in range(151)]
        previous = None
        for frame in range(1, 401):
            p, t = frame_input(plant, points, frame, 3.0)
            output = engine.compute(p, t)
            self.assertTrue(output.valid, output.errors)
            self.assertFalse(output.throttle > 0 and output.brake > 0)
            if previous is not None:
                elapsed = output.diagnostics["dt_s"]
                self.assertLessEqual(abs(output.steering - previous.steering),
                                     engine.settings.steering_slew_per_s * elapsed + 1e-8)
            dt = (0.02, 0.08, 0.035, 0.065)[(frame-1) % 4]
            plant.step(output, dt, calibration)
            now[0] += dt
            previous = output
        self.assertLess(abs(plant.y), 0.05)
        self.assertLess(abs(plant.speed - 3.0), 0.15)

    def test_rolling_profiles_stop_hold_and_restart_with_terminal_stop(self):
        plant = Plant(lag=0.3)
        calibration = vehicle()
        now = [0.0]
        engine = ControlEngine(calibration, clock=lambda: now[0])
        stop_x = None
        hold_seen = False
        resume_x = None
        for frame in range(1, 601):
            clock = (frame - 1) * 0.05
            stopping = 5.0 <= clock < 15.0
            if stopping and stop_x is None:
                stop_x = plant.x + 12.0
            if clock >= 15.0 and resume_x is None:
                resume_x = plant.x
                self.assertTrue(hold_seen)
                self.assertLess(plant.speed, 0.05)
            target = 0.0 if stopping else 3.0
            distance = max(0.0, stop_x - plant.x) if stopping else 60.0
            points = []
            horizon = min(60.0, distance)
            for index in range(61):
                i = index * horizon / 60.0
                desired = plant.speed if stopping else 3.0
                cap = math.sqrt(max(0.0, 2.0 * (distance - i)))
                acceleration_cap = math.sqrt(plant.speed**2 + 2.0*i)
                points.append((plant.x+i, 0.0, min(desired, cap, acceleration_cap)))
            if stopping and plant.speed <= 1e-6:
                points = [(plant.x, 0.0, 0.0), (plant.x, 0.0, 0.0)]
            p, t = frame_input(plant, points, frame, target, distance)
            output = engine.compute(p, t)
            self.assertTrue(output.valid, output.errors)
            self.assertFalse(output.throttle > 0 and output.brake > 0)
            if stopping:
                self.assertEqual(0.0, output.throttle)
                hold_seen = hold_seen or output.diagnostics["state"] == "HOLD"
            plant.step(output, 0.05, calibration)
            now[0] += 0.05
        self.assertGreater(plant.x - resume_x, 20.0)
        self.assertGreater(plant.speed, 2.5)


if __name__ == "__main__":
    unittest.main()
