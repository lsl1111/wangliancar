"""Motion continuity and stop-pose contracts on genuinely new inputs."""

import math
import unittest

from core.interfaces import ControlOut
from core.validation import validate_output
from members.control.controller import ControlEngine
from members.control.parameters import ControllerSettings
from members.control.trajectory import PreparedPath
from tests.control_benchmark import vehicle
from tests.test_control import inputs, calibration
from tests.test_control_maneuvers import BidirectionalPlant, maneuver_input


class ContinuityTests(unittest.TestCase):
    def setUp(self):
        self.now = [0.0]
        self.engine = ControlEngine(calibration(), clock=lambda: self.now[0])

    def compute(self, p, t):
        result = self.engine.compute(p, t)
        self.now[0] += 0.05
        return result

    def test_frame_rollback_rearms_without_waiting_for_old_frame_number(self):
        p, t = inputs(frame=500, speed=0)
        self.assertTrue(self.compute(p, t).valid)
        p, t = inputs(frame=1, speed=0)
        discontinuity = self.compute(p, t)
        self.assertFalse(discontinuity.valid)
        self.assertEqual(1, self.engine.last_frame)
        outputs = []
        for frame in range(2, 23):
            p, t = inputs(frame=frame, speed=0)
            result = self.compute(p, t)
            self.assertTrue(result.valid, result.errors)
            outputs.append(result)
        self.assertEqual("GEAR_STOP", outputs[0].diagnostics["state"])
        self.assertTrue(any(output.throttle > 0 for output in outputs))

    def test_same_context_rollback_retains_unfinished_dwell(self):
        plant = BidirectionalPlant()
        for frame in range(100, 111):
            p, t = maneuver_input(plant, [(0, 0, 0), (0, 0, 0)], frame,
                target=0, stop_distance=0, precision=True, dwell=1, parking=True)
            self.assertTrue(self.compute(p, t).valid)
        p, t = maneuver_input(plant, [(0, 0, 1), (10, 0, 1)], 1)
        self.assertFalse(self.compute(p, t).valid)
        self.assertTrue(self.engine.standstill.active)
        self.assertEqual(0, self.engine.standstill.elapsed)
        for frame in range(2, 17):
            p, t = maneuver_input(plant, [(0, 0, 1), (10, 0, 1)], frame)
            result = self.compute(p, t)
            self.assertTrue(result.valid, result.errors)
            self.assertEqual("DWELL", result.diagnostics["state"])
            self.assertEqual(0, result.throttle)

    def test_unusable_rollback_does_not_rebase_trusted_frame_state(self):
        p, t = inputs(frame=100, speed=0)
        self.assertTrue(self.compute(p, t).valid)
        p, t = inputs(frame=1, speed=0)
        p.source_status["gps"]["usable"] = False
        self.assertFalse(self.compute(p, t).valid)
        self.assertEqual(100, self.engine.last_frame)

    def test_unusable_context_change_does_not_cancel_minimum_dwell(self):
        plant = BidirectionalPlant()
        p, t = maneuver_input(plant, [(0, 0, 0), (0, 0, 0)], 1,
            target=0, stop_distance=0, precision=True, dwell=10, parking=True)
        self.assertTrue(self.compute(p, t).valid)
        p, t = maneuver_input(plant, [(0, 0, 1), (10, 0, 1)], 2)
        p.task_id = "untrusted new context"
        p.source_status["gps"]["usable"] = False
        self.assertFalse(self.compute(p, t).valid)
        p, t = maneuver_input(plant, [(0, 0, 1), (10, 0, 1)], 3)
        result = self.compute(p, t)
        self.assertTrue(result.valid, result.errors)
        self.assertEqual("DWELL", result.diagnostics["state"])
        self.assertEqual(0, result.throttle)

    def test_replan_reanchors_progress_but_speed_updates_keep_continuity(self):
        p, t = inputs(speed=2, x=5)
        self.assertTrue(self.compute(p, t).valid)
        p, t = inputs(frame=2, speed=2, x=5)
        for point in t.points:
            point.speed = 4
        result = self.compute(p, t)
        self.assertTrue(result.valid, result.errors)
        self.assertTrue(result.diagnostics["projection_continuity_used"])
        points = [(float(i), 0, 4) for i in range(5, 30)]
        p, t = inputs(frame=3, speed=2, x=5, points=points)
        result = self.compute(p, t)
        self.assertTrue(result.valid, result.errors)
        self.assertFalse(result.diagnostics["projection_continuity_used"])
        self.assertEqual(0, result.diagnostics["path_progress_m"])

    def test_unreachable_progress_jump_cannot_select_a_future_segment(self):
        p, t = inputs(speed=2, x=5)
        self.assertTrue(self.compute(p, t).valid)
        p, t = inputs(frame=2, speed=2, x=25)
        result = self.compute(p, t)
        self.assertFalse(result.valid)
        self.assertEqual(0, result.throttle)

    def test_precision_stop_respects_requested_body_yaw(self):
        p, t = inputs(speed=0, x=4.92,
                      points=[(0, 0, 0.3), (4, 0, 0.3), (5, 0, 0)])
        t.target_speed, t.stop_required, t.stop_distance = 0.3, True, 0.08
        t.precision_stop, t.hold_duration_s = True, 10
        t.points[-1].heading = 1.0
        result = self.compute(p, t)
        self.assertTrue(result.valid, result.errors)
        self.assertEqual("HOLD", result.diagnostics["state"])
        self.assertFalse(result.diagnostics["stop_pose_arrived"])
        self.assertFalse(self.engine.standstill.active)
        self.assertAlmostEqual(1, result.diagnostics["stop_heading_error_rad"])

    def test_stop_boundary_uses_local_requested_yaw_before_path_end(self):
        p, t = inputs(speed=0, x=4.92,
                      points=[(0, 0, 0.3), (4, 0, 0.3), (5, 0, 0.3), (8, 0, 0)])
        t.target_speed, t.stop_required, t.stop_distance = 0.3, True, 0.08
        t.precision_stop, t.hold_duration_s = True, 1
        t.points[-1].heading = 1.0
        result = self.compute(p, t)
        self.assertTrue(result.valid, result.errors)
        self.assertEqual("DWELL", result.diagnostics["state"])
        self.assertTrue(result.diagnostics["stop_pose_arrived"])

    def test_requested_yaw_interpolation_wraps_at_pi(self):
        p, t = inputs(points=[(0, 0, 1), (10, 0, 1)])
        t.points[0].heading, t.points[1].heading = 3.1, -3.1
        path = PreparedPath(t.points, 12)
        self.assertAlmostEqual(math.pi, abs(path.heading_at(5)), places=6)

    def test_forward_crawl_restarts_from_hold_even_below_fixed_deadband(self):
        for target in (0.07, 0.3):
            with self.subTest(target=target):
                self.setUp()
                plant = BidirectionalPlant()
                p, t = maneuver_input(plant, [(0, 0, 0), (0, 0, 0)], 1,
                                      target=0, stop_distance=0)
                self.assertEqual("HOLD", self.compute(p, t).diagnostics["state"])
                for frame in range(2, 402):
                    points = [(i, 0, target) for i in range(21)]
                    p, t = maneuver_input(plant, points, frame, target=target)
                    result = self.compute(p, t)
                    self.assertTrue(result.valid, result.errors)
                    self.assertFalse(result.throttle > 0 and result.brake > 0)
                    plant.step(result, 0.05, vehicle())
                self.assertGreater(plant.x, target * 10)
                self.assertAlmostEqual(target, plant.speed, delta=0.04)

    def test_precision_creep_after_early_standstill_reaches_dwell(self):
        plant = BidirectionalPlant(x=4.88)
        points = [(0, 0, 0.3), (4.99, 0, 0.3), (5, 0, 0)]
        dwell_seen = False
        for frame in range(1, 81):
            p, t = maneuver_input(plant, points, frame, target=0.3,
                stop_distance=max(0, 5-plant.x), precision=True, dwell=1, parking=True)
            result = self.compute(p, t)
            self.assertTrue(result.valid, result.errors)
            dwell_seen = dwell_seen or result.diagnostics["state"] == "DWELL"
            plant.step(result, 0.05, vehicle())
        self.assertTrue(dwell_seen)
        self.assertGreater(plant.x, 4.88)
        self.assertLess(plant.x, 5)

    def test_global_speed_cap_does_not_block_low_point_speed_restart(self):
        plant = BidirectionalPlant()
        p, t = maneuver_input(plant, [(0, 0, 0), (0, 0, 0)], 1,
                              target=0, stop_distance=0)
        self.compute(p, t)
        for frame in range(2, 242):
            p, t = maneuver_input(plant, [(i, 0, 0.15) for i in range(21)],
                                  frame, target=5)
            result = self.compute(p, t)
            self.assertTrue(result.valid, result.errors)
            self.assertLessEqual(result.diagnostics["reference_speed_mps"], 0.15)
            plant.step(result, 0.05, vehicle())
        self.assertGreater(plant.x, 0.5)
        self.assertAlmostEqual(0.15, plant.speed, delta=0.03)

    def test_self_crossing_route_keeps_progress_in_forward_and_reverse(self):
        start = -0.5
        points = [(20*math.sin(start+i*0.015),
                   10*math.sin(2*(start+i*0.015)), 1.0)
                  for i in range(291)]
        tangent = math.atan2(20*math.cos(2*start), 20*math.cos(start))
        for direction in (1, -1):
            with self.subTest(direction=direction):
                self.setUp()
                plant = BidirectionalPlant(x=points[0][0], y=points[0][1],
                    heading=tangent if direction == 1 else tangent-math.pi,
                    lag=0.2)
                _, first = maneuver_input(plant, points, 1, direction=direction)
                path = PreparedPath(first.points, 12)
                # Without established progress the crossing is ambiguous and
                # must still be rejected, rather than guessing a future leg.
                with self.assertRaises(ValueError):
                    path.project(0, 0, math.pi/4, ControllerSettings())
                errors = []
                progress = []
                for frame in range(1, 501):
                    p, t = maneuver_input(plant, points, frame, direction=direction)
                    result = self.compute(p, t)
                    self.assertTrue(result.valid, result.errors)
                    validate_output(result, ControlOut, t)
                    if "path_progress_m" in result.diagnostics:
                        progress.append(result.diagnostics["path_progress_m"])
                        errors.append(result.diagnostics["cross_track_abs_m"])
                    plant.step(result, 0.05, vehicle())
                self.assertGreater(plant.x, 4)
                self.assertGreater(plant.y, 4)
                self.assertLess(max(errors), 0.35)
                self.assertLess(max(b-a for a, b in zip(progress, progress[1:])), 0.15)


if __name__ == "__main__":
    unittest.main()
