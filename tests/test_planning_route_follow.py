"""Planning must accept verified successor traffic and retain motion guards."""
import json
import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from core.interfaces import DecisionMode, Perception, Target
from core.serialization import to_dict
from members.decision.engine import DecisionEngine
from members.planning.lane_planner import PlannerSettings, build_trajectory
from scripts.replay_decision import _assign
from tests import test_curve_route_targets as curve_fixtures
from tests.test_curve_route_targets import hairpin
from tests.test_route_continuation import Map, manager


class PlanningRouteFollowTests(unittest.TestCase):
    def observation(self):
        fixture = curve_fixtures.CurveRouteTargetTests()
        fixture.setUp()
        p = fixture.observation()
        target = p.targets[0]
        a, b = hairpin().segments['c'][4:6]
        target.x, target.y = (a[0]+b[0])/2, (a[1]+b[1])/2
        angle = math.atan2(b[1]-a[1], b[0]-a[0])
        target.heading = angle
        target.vx = 5*math.cos(angle) - 0.02*math.sin(angle)
        target.vy = 5*math.sin(angle) + 0.02*math.cos(angle)
        target.lane_width_m = 3.97
        decision = DecisionEngine(fixture.settings).run(p)
        self.assertEqual(DecisionMode.FOLLOW, decision.mode, decision.reason)
        return p, decision

    @staticmethod
    def settings():
        return PlannerSettings(front_offset_m=3.9187, half_width_m=0.9)

    def test_follow_intent_reaches_moving_trajectory_across_hairpin_segments(self):
        p, d = self.observation()
        with patch('members.planning.obstacle_guard._route_association', return_value=None):
            before = build_trajectory(p, d, self.settings())
        self.assertFalse(before.valid)
        self.assertIn('motion unsupported', before.reason)
        after = build_trajectory(p, d, self.settings())
        self.assertTrue(after.valid, after.reason)
        self.assertFalse(after.emergency_stop)
        self.assertGreater(after.points[1].speed, 0)
        self.assertGreater(after.stop_distance, 0)

    def test_successor_needs_its_own_measured_width_and_verified_range(self):
        for change in ('width', 'spans', 'membership', 'prefix'):
            p, d = self.observation()
            if change == 'width': p.targets[0].lane_width_m = None
            if change == 'spans': p.lane.forward_lane_spans[2]['start_index'] = 0
            if change == 'membership': p.targets[0].same_lane_valid = False
            if change == 'prefix': p.lane.forward_reference[0] = (-60, 0.01)
            result = build_trajectory(p, d, self.settings())
            self.assertFalse(result.valid, change)

    def test_narrow_successor_does_not_borrow_wider_ego_lane(self):
        p, d = self.observation()
        p.targets[0].lane_width_m = 1.9
        self.assertFalse(build_trajectory(p, d, self.settings()).valid)

    def test_crossing_and_oncoming_remain_rejected_on_verified_successor(self):
        for crossing in (False, True):
            p, d = self.observation()
            target = p.targets[0]
            angle = target.heading
            speed, lateral = (5, 1) if crossing else (-5, 0.02)
            target.vx = speed*math.cos(angle)-lateral*math.sin(angle)
            target.vy = speed*math.sin(angle)+lateral*math.cos(angle)
            result = build_trajectory(p, d, self.settings())
            self.assertFalse(result.valid)
            self.assertIn('motion unsupported', result.reason)

    def test_observed_lead_advancing_moves_conservative_stop_boundary_forward(self):
        p, d = self.observation()
        first = build_trajectory(p, d, self.settings())
        target = p.targets[0]
        a, b = hairpin().segments['c'][6:8]
        target.x, target.y = (a[0]+b[0])/2, (a[1]+b[1])/2
        target.heading = math.atan2(b[1]-a[1], b[0]-a[0])
        target.vx = 5*math.cos(target.heading)-0.02*math.sin(target.heading)
        target.vy = 5*math.sin(target.heading)+0.02*math.cos(target.heading)
        # New perception/decision frame; the target stays present throughout.
        p.frame_id += 1
        fixture = curve_fixtures.CurveRouteTargetTests()
        fixture.setUp()
        d = DecisionEngine(fixture.settings).run(p)
        second = build_trajectory(p, d, self.settings())
        self.assertTrue(first.valid and second.valid, second.reason)
        self.assertGreater(second.stop_distance, first.stop_distance)
        self.assertGreater(second.points[1].speed, 0)

    def test_map_membership_publishes_width_and_clears_unverified_measurement(self):
        api = Map({'a': [(0, 0), (80, 0)]})
        api.getLaneWidth = lambda *args: SimpleNamespace(exists=True, width=3.25)
        route = manager(api)
        target = Target()
        target.x, target.y = 20, 0
        self.assertEqual(('a', True), route.locate_target(target))
        self.assertEqual(3.25, target.lane_width_m)
        target.y = 5
        self.assertEqual(('', False), route.locate_target(target))
        self.assertIsNone(target.lane_width_m)

    def test_target_width_round_trip_and_old_snapshot_default(self):
        p, d = self.observation()
        payload = json.loads(json.dumps(to_dict(p), allow_nan=False))
        restored = _assign(Perception(), payload, 'perception')
        self.assertEqual(3.97, restored.targets[0].lane_width_m)
        del payload['targets'][0]['lane_width_m']
        restored = _assign(Perception(), payload, 'perception')
        self.assertIsNone(restored.targets[0].lane_width_m)


if __name__ == '__main__':
    unittest.main()
