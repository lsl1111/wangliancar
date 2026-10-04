"""Impossible steering geometry needs a reachable boundary, not saturation."""
import math
import unittest

from core.interfaces import DecisionMode
from members.control.controller import ControlEngine
from members.planning.lane_planner import build_trajectory
from tests.test_planning import scene
from tests.test_planning_geometry_recovery import circle, capability
from tests.test_control_maneuvers import BidirectionalPlant
from tests.control_benchmark import vehicle


def road():
    points = [(float(x), 0.) for x in range(-10, 41)]
    points.extend((40.+4.*math.sin(i*.02), 4.*(1.-math.cos(i*.02))) for i in range(1, 121))
    return points


class PlanningSteeringEnvelopeTests(unittest.TestCase):
    def test_current_untrackable_curve_requests_braking_even_at_low_speed(self):
        p, d = circle(speed=1., radius=4., theta=.4)
        t = build_trajectory(p, d, capability())
        self.assertTrue(t.valid, t.reason)
        self.assertTrue(t.emergency_stop, t.reason)
        self.assertIn('steering', t.reason)

    def test_future_untrackable_curve_has_a_finite_stop_before_its_entry(self):
        p, d = scene(speed=8., target=8.)
        p.lane.center_line = road()
        ps = capability()
        t = build_trajectory(p, d, ps)
        self.assertTrue(t.valid and not t.emergency_stop, t.reason)
        self.assertIn('steering', t.reason)
        self.assertTrue(t.stop_required)
        self.assertGreater(t.stop_distance, 8.**2/(2*ps.deceleration))
        self.assertLessEqual(t.stop_distance+math.hypot(ps.front_offset_m, ps.half_width_m), 40.)
        self.assertEqual(0., t.points[-1].speed)
        self.assertTrue(all(abs(q.y) < 1e-9 for q in t.points))

    def test_prior_stop_and_short_view_do_not_check_unreachable_future_curve(self):
        for earlier_stop in (True, False):
            p, d = scene(speed=2., target=8.)
            p.lane.center_line = road()
            ps = capability(horizon=10.)
            if earlier_stop:
                d.mode, d.stop_distance, d.target_speed = DecisionMode.STOP, 5., 0.
            t = build_trajectory(p, d, ps)
            self.assertTrue(t.valid and not t.emergency_stop, t.reason)
            self.assertNotIn('steering', t.reason)
            if earlier_stop:
                self.assertLessEqual(t.stop_distance, 5.)

    def test_trackable_dense_curve_is_preserved(self):
        p, d = circle(speed=1., radius=10., theta=.4)
        t = build_trajectory(p, d, capability())
        self.assertTrue(t.valid and not t.emergency_stop, t.reason)
        self.assertNotIn('untrackable', t.reason)

    def test_impossible_future_curve_without_body_extent_cannot_claim_safe_stop(self):
        p, d = scene(speed=2., target=8.)
        p.lane.center_line = road()
        t = build_trajectory(p, d, capability(front_offset_m=None, half_width_m=None))
        self.assertFalse(t.valid, t.reason)
        self.assertIn('steering', t.reason)

    def test_real_control_stops_before_impossible_curve_and_keeps_holding(self):
        plant = BidirectionalPlant(speed=8.)
        now = [0.]
        ps, calibration = capability(), vehicle()
        calibration.wheelbase_m = ps.wheelbase_m
        control = ControlEngine(calibration, clock=lambda: now[0])
        for frame in range(1, 601):
            p, d = scene(speed=plant.speed, target=8.)
            p.frame_id, p.ego.frame_id, p.ego.age_ms = frame, frame, 0
            d.bind(p)
            p.ego.x, p.ego.y, p.ego.heading = plant.x, plant.y, plant.heading
            p.ego.vx, p.ego.vy = plant.speed*math.cos(plant.heading), plant.speed*math.sin(plant.heading)
            p.ego.yaw_rate = plant.yaw_rate
            p.source_status['gps'] = {'usable': True}
            p.lane.center_line = road()
            t = build_trajectory(p, d, ps)
            self.assertTrue(t.valid and not t.emergency_stop, (frame, t.reason))
            c = control.compute(p, t)
            self.assertTrue(c.valid, (frame, c.errors))
            self.assertLess(plant.x+ps.front_offset_m, 40.)
            plant.step(c, .05, calibration)
            now[0] += .05
        self.assertGreater(plant.x, 25.)
        self.assertLess(plant.speed, .05)
        self.assertEqual('HOLD', c.diagnostics['state'])


if __name__ == '__main__':
    unittest.main()
