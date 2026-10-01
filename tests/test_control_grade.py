"""Longitudinal grade disturbances with independent signed force dynamics.

Acceleration gains and +/-0.6 m/s^2 gravity are test parameters, not a
SimOne calibration. Upstream supplies speed/stop demand; no grade API or
case-specific control branch is used. Rolling backwards remains observable.
"""

import math
import unittest

from core.interfaces import ControlOut
from core.validation import validate_output
from members.control.controller import ControlEngine
from tests.control_benchmark import vehicle
from tests.test_control_maneuvers import BidirectionalPlant, maneuver_input


class GradePlant(BidirectionalPlant):
    def __init__(self, speed=0.0):
        super(GradePlant, self).__init__(speed=speed)
        self.grade_acceleration = 0.0
        self.velocity = speed

    def step(self, output, dt, calibration):
        if output.gear in (1, 2):
            requested = 1 if output.gear == 1 else -1
            if requested != self.direction and self.speed > 0.05:
                raise AssertionError("grade model gear changed while moving")
            self.direction = requested
        blend = 1.0 - math.exp(-dt / self.lag)
        self.throttle += blend * (output.throttle - self.throttle)
        self.brake += blend * (output.brake - self.brake)
        propulsion = self.direction * 3.0 * self.throttle if output.gear in (1, 2) else 0.0
        force = propulsion - 0.08*self.velocity - self.grade_acceleration
        braking = 6.0 * self.brake
        if abs(self.velocity) < 1e-9:
            # Brakes provide static holding force, but insufficient braking
            # permits downhill or backward roll; do not clamp speed to zero.
            acceleration = math.copysign(max(0.0, abs(force)-braking), force)
        else:
            acceleration = force - math.copysign(braking, self.velocity)
        next_velocity = self.velocity + dt*acceleration
        if self.velocity*next_velocity < 0 and abs(force) <= braking:
            next_velocity = 0.0
        if output.handbrake:
            next_velocity = 0.0
        self.x += dt*(self.velocity+next_velocity)/2
        self.velocity = next_velocity
        self.speed = abs(next_velocity)
        self.direction = -1 if next_velocity < 0 else self.direction


class GradeControlTests(unittest.TestCase):
    def setUp(self):
        self.now = [0.0]
        self.engine = ControlEngine(vehicle(), clock=lambda: self.now[0])
        self.frame = 0

    def step(self, plant, target, stop=False):
        self.frame += 1
        points = ([(plant.x, 0, 0), (plant.x, 0, 0)] if stop else
                  [(i*10, 0, target) for i in range(151)])
        p, trajectory = maneuver_input(plant, points, self.frame,
                                      target=target, stop_distance=0 if stop else -1)
        output = self.engine.compute(p, trajectory)
        self.assertTrue(output.valid, output.errors)
        validate_output(output, ControlOut, trajectory)
        self.assertFalse(output.throttle > 0 and output.brake > 0)
        self.assertLessEqual(output.throttle, vehicle().max_throttle)
        self.assertLessEqual(output.brake, vehicle().max_brake)
        plant.step(output, 0.05, vehicle())
        self.now[0] += 0.05
        return output

    def test_twenty_kph_reference_tracks_flat_uphill_downhill_and_flat(self):
        target = 20.0/3.6
        plant = GradePlant(speed=target)
        for grade in (0.0, 0.6, -0.6, 0.0):
            with self.subTest(gravity_acceleration=grade):
                plant.grade_acceleration = grade
                speeds = []
                for sample in range(800):
                    self.step(plant, target)
                    if sample >= 700:
                        speeds.append(plant.speed)
                self.assertLess(max(abs(speed-target) for speed in speeds), 0.15)
                self.assertGreater(plant.velocity, 0)
        self.assertGreater(plant.x, 800)

    def test_downhill_stop_holds_and_restarts_without_pedal_overlap(self):
        plant = GradePlant(speed=3.0)
        plant.grade_acceleration = -0.6
        for _ in range(200):
            stopped = self.step(plant, 0, stop=True)
        self.assertEqual("HOLD", stopped.diagnostics["state"])
        self.assertLess(plant.speed, 0.01)
        stopped_position = plant.x
        for _ in range(100):
            self.step(plant, 0, stop=True)
        self.assertLess(abs(plant.x-stopped_position), 0.01)
        for _ in range(500):
            self.step(plant, 3.0)
        self.assertGreater(plant.x, stopped_position+30)
        self.assertAlmostEqual(3.0, plant.speed, delta=0.15)

    def test_uphill_overspeed_cutoff_unwinds_integral_after_load_change(self):
        target = 20.0/3.6
        plant = GradePlant(speed=target)
        plant.grade_acceleration = 0.6
        for _ in range(800):
            self.step(plant, target)
        uphill_integral = self.engine.integral
        self.assertGreater(uphill_integral, 3.0)
        plant.grade_acceleration = -0.6
        cutoff_seen = False
        peak = plant.speed
        for _ in range(600):
            previous_speed = plant.speed
            output = self.step(plant, target)
            if previous_speed > target+0.08:
                self.assertEqual(0, output.throttle)
                cutoff_seen = True
            peak = max(peak, plant.speed)
        self.assertTrue(cutoff_seen)
        self.assertLess(peak, target+0.5)
        self.assertLess(self.engine.integral, uphill_integral)
        self.assertAlmostEqual(target, plant.speed, delta=0.15)


if __name__ == "__main__":
    unittest.main()
