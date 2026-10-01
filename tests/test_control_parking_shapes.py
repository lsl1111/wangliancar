"""Execute synthetic perpendicular/parallel parking paths through real control.

These are reference geometry fixtures, not the production parking planner.
No occupied-bay selection, vehicle envelope or collision certificate is
implied. Each path includes curved reverse, alignment, dwell and forward exit.
"""

import math
import unittest

from core.geometry import normalize_angle, project_polyline
from core.interfaces import ControlOut
from core.validation import validate_output
from members.control.controller import ControlEngine
from tests.control_benchmark import vehicle
from tests.test_control_maneuvers import BidirectionalPlant, maneuver_input


def parking_reference(kind):
    radius = 6.0
    angle = math.pi/2 if kind == "perpendicular" else 0.7
    coordinates, headings = [], []
    for i in range(101):
        turn = angle*i/100
        coordinates.append((-radius*math.sin(turn), radius*(1-math.cos(turn))))
        headings.append(-turn)
    if kind == "parallel":
        for i in range(1, 101):
            turn = angle*i/100
            coordinates.append((-2*radius*math.sin(angle)-radius*math.sin(turn-angle),
                                radius*(1-2*math.cos(angle)+math.cos(angle-turn))))
            headings.append(-angle+turn)
    terminal = coordinates[-1]
    yaw = headings[-1]
    for i in range(1, 21):
        distance = 0.1*i
        coordinates.append((terminal[0]-distance*math.cos(yaw),
                            terminal[1]-distance*math.sin(yaw)))
        headings.append(yaw)
    arc = [0.0]
    for a, b in zip(coordinates, coordinates[1:]):
        arc.append(arc[-1]+math.hypot(b[0]-a[0], b[1]-a[1]))
    points = [(x, y, min(0.8, math.sqrt(max(0, 2*(arc[-1]-s)))))
              for (x, y), s in zip(coordinates, arc)]
    return points, headings, arc[-1]


class ParkingShapeControlTests(unittest.TestCase):
    def test_perpendicular_and_parallel_reverse_dwell_and_forward_exit(self):
        for kind in ("perpendicular", "parallel"):
            with self.subTest(shape=kind):
                self.execute(kind)

    def execute(self, kind):
        now = [0.0]
        calibration = vehicle()
        engine = ControlEngine(calibration, clock=lambda: now[0])
        plant = BidirectionalPlant(lag=0.3)
        points, headings, length = parking_reference(kind)
        endpoint = points[-1][:2]
        parked_at = exit_at = None
        parking_errors = []
        for frame in range(1, 1701):
            if parked_at is None:
                projection = project_polyline(points, plant.x, plant.y)
                p, trajectory = maneuver_input(plant, points, frame, target=0.8,
                    direction=-1, stop_distance=max(0, length-projection["s"]),
                    precision=True, dwell=10, parking=True)
                for point, heading in zip(trajectory.points, headings):
                    point.heading = heading
                trajectory.right_signal = True
            else:
                # Submit the exit immediately: the control dwell must still
                # run for its full minimum, then arm D before propulsion.
                exit_points = [(x, y, 0.8) for x, y, _ in reversed(points)]
                p, trajectory = maneuver_input(plant, exit_points, frame, target=0.8)
                for point, heading in zip(trajectory.points, reversed(headings)):
                    point.heading = heading
                trajectory.left_signal = True
            output = engine.compute(p, trajectory)
            self.assertTrue(output.valid, (kind, frame, output.errors))
            validate_output(output, ControlOut, trajectory)
            self.assertFalse(output.throttle > 0 and output.brake > 0)
            if parked_at is None:
                parking_errors.append(output.diagnostics.get("cross_track_abs_m", 0))
                self.assertTrue(output.right_signal)
                if output.diagnostics["state"] == "DWELL":
                    parked_at = now[0]
                    self.assertLessEqual(plant.speed, 0.05)
                    self.assertLess(math.hypot(plant.x-endpoint[0], plant.y-endpoint[1]), 0.16)
                    self.assertLess(abs(normalize_angle(plant.heading-headings[-1])), 0.15)
            elif output.throttle > 0 and exit_at is None:
                exit_at = now[0]
                self.assertEqual(1, output.gear)
                self.assertFalse(output.handbrake)
            if parked_at is not None and now[0]-parked_at < 10:
                self.assertEqual(0, output.throttle)
                self.assertTrue(output.handbrake)
            plant.step(output, 0.05, calibration)
            now[0] += 0.05
            if exit_at is not None and math.hypot(plant.x-endpoint[0], plant.y-endpoint[1]) > 5:
                break
        self.assertIsNotNone(parked_at, (kind, plant.x, plant.y, plant.heading))
        self.assertIsNotNone(exit_at, (kind, "exit never armed"))
        self.assertGreaterEqual(exit_at-parked_at, 10)
        self.assertLess(max(parking_errors), 0.3)
        self.assertEqual(1, plant.direction)
        self.assertGreater(math.hypot(plant.x-endpoint[0], plant.y-endpoint[1]), 5)


if __name__ == "__main__":
    unittest.main()
