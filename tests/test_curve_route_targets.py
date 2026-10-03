"""Continuous hairpin routes and map-associated targets across lane segments."""
import copy
import json
import math
import unittest
from unittest.mock import patch

from core.interfaces import DecisionMode, Perception
from core.serialization import to_dict
from members.decision.candidates import RouteContext
from members.decision.engine import DecisionEngine
from members.decision.settings import DecisionSettings
from scripts.replay_decision import _assign
from tests.test_decision import add_target, perception
from tests.test_route_continuation import Map, ego, manager


def hairpin(gap=0.158):
    radius = 12.0
    last_angle = math.pi / 2 - gap / radius
    first = [(radius * math.sin(last_angle * i / 18),
              radius * (1 - math.cos(last_angle * i / 18))) for i in range(19)]
    second = [(radius * math.sin(math.pi / 2 + math.pi * i / 36),
               radius * (1 - math.cos(math.pi / 2 + math.pi * i / 36)))
              for i in range(19)]
    return Map({'a': [(-60, 0), (0, 0)], 'b': first, 'c': second,
                'd': [(0, 24), (-120, 24)]},
               {'a': ['b'], 'b': ['c'], 'c': ['d']})


class CurveRouteTargetTests(unittest.TestCase):
    def setUp(self):
        self.settings = DecisionSettings(front_offset_m=3.9187)

    def observation(self, api=None):
        api = hairpin() if api is None else api
        p = perception(speed=0.0)
        p.scene_id = 15
        p.ego.x = -20.0
        p.lane = manager(api).update(ego(-20), force=True)
        p.lane.lane_width = 3.5
        p.lane.lane_width_valid = True
        target = add_target(p, speed=5.0, lane_id='c')
        angle = math.radians(110)
        target.x, target.y = 12 * math.sin(angle), 12 * (1 - math.cos(angle))
        target.vx, target.vy = 5 * math.cos(angle), 5 * math.sin(angle)
        return p

    def test_sdk_scale_sample_gap_no_longer_cuts_route_or_loses_lead(self):
        api = hairpin()
        with patch('perception.route_manager.JOIN_POSITION_TOLERANCE_M', 0.1):
            before = self.observation(api)
        self.assertEqual(['a', 'b'], before.lane.forward_lane_ids)
        stopped = DecisionEngine(self.settings).run(before)
        self.assertEqual(DecisionMode.STOP, stopped.mode)
        self.assertIn('TARGET_UNKNOWN', stopped.reason)
        after = self.observation()
        self.assertEqual(['a', 'b', 'c', 'd'], after.lane.forward_lane_ids)
        followed = DecisionEngine(self.settings).run(after)
        self.assertEqual(DecisionMode.FOLLOW, followed.mode, followed.reason)
        self.assertGreater(followed.target_speed, 0)

    def test_later_successor_without_spans_stays_protected(self):
        p = self.observation()
        p.lane.forward_lane_spans = []
        result = DecisionEngine(self.settings).run(p)
        self.assertEqual(DecisionMode.STOP, result.mode)
        self.assertIn('segment boundary unavailable', result.reason)

    def test_map_membership_disambiguates_close_hairpin_legs(self):
        p = perception(speed=0)
        p.lane.center_line = [(0, 0), (10, 0)]
        p.lane.forward_reference = [(0, 0), (10, 0), (10, 2), (0, 2)]
        p.lane.forward_reference_valid = True
        p.lane.forward_lane_ids = ['lane-1', 'bend', 'return']
        p.lane.forward_lane_spans = [
            {'lane_id': 'lane-1', 'start_index': 0, 'end_index': 1},
            {'lane_id': 'bend', 'start_index': 1, 'end_index': 2},
            {'lane_id': 'return', 'start_index': 2, 'end_index': 3}]
        target = add_target(p, speed=3, lane_id='return')
        target.x, target.y, target.vx = 5, 1, -3
        route = RouteContext(p, self.settings)
        self.assertIsNone(route.project(target.x, target.y, self.settings))
        projection = route.project(target.x, target.y, self.settings, target.lane_id)
        self.assertAlmostEqual(17, projection['s'])
        self.assertEqual(DecisionMode.FOLLOW, DecisionEngine(self.settings).run(p).mode)
        target.same_lane_valid = False
        self.assertIn('multiple_route_positions', DecisionEngine(self.settings).run(p).reason)

    def test_current_lane_target_does_not_jump_to_return_leg(self):
        p = self.observation()
        route = RouteContext(p, self.settings)
        projection = route.project(-10, 0, self.settings, 'a')
        self.assertAlmostEqual(50, projection['s'])
        self.assertIsNone(route.project(-10, 24, self.settings, 'a'))

    def test_wrong_map_lane_does_not_authorize_projection_to_other_span(self):
        p = self.observation()
        p.targets[0].lane_id = 'b'
        result = DecisionEngine(self.settings).run(p)
        self.assertEqual(DecisionMode.STOP, result.mode)
        self.assertIn('TARGET_UNKNOWN', result.reason)

    def test_invalid_span_metadata_keeps_later_successor_guard(self):
        p = self.observation()
        invalid = [None, [], [{'lane_id': 'a', 'start_index': 0, 'end_index': 1}]]
        for key, value in (('start_index', True), ('start_index', 100000),
                           ('end_index', 100000), ('lane_id', 'wrong')):
            spans = copy.deepcopy(p.lane.forward_lane_spans)
            spans[2][key] = value
            invalid.append(spans)
        spans = copy.deepcopy(p.lane.forward_lane_spans)
        spans[-1]['end_index'] -= 1
        invalid.append(spans)
        for spans in invalid:
            with self.subTest(spans=spans):
                p.lane.forward_lane_spans = spans
                result = DecisionEngine(self.settings).run(p)
                self.assertEqual(DecisionMode.STOP, result.mode)
                self.assertIn('segment boundary unavailable', result.reason)

    def test_target_outside_verified_end_and_corridor_remains_protected(self):
        p = self.observation()
        target = p.targets[0]
        target.lane_id, target.x, target.y = 'd', -125, 24
        route = RouteContext(p, self.settings)
        self.assertIsNone(route.project(target.x, target.y, self.settings, 'd'))
        self.assertEqual('outside_route_start_or_end', route.projection_reason)
        target.lane_id, target.x, target.y = 'c', 30, 15
        result = DecisionEngine(self.settings).run(p)
        self.assertEqual(DecisionMode.STOP, result.mode)
        self.assertIn('TARGET_UNKNOWN', result.reason)

    def test_spans_round_trip_and_legacy_snapshot_default(self):
        p = self.observation()
        payload = json.loads(json.dumps(to_dict(p), allow_nan=False))
        replay = _assign(Perception(), payload, 'perception')
        self.assertEqual(p.lane.forward_lane_spans, replay.lane.forward_lane_spans)
        self.assertEqual(DecisionMode.FOLLOW, DecisionEngine(self.settings).run(replay).mode)
        del payload['lane']['forward_lane_spans']
        replay = _assign(Perception(), payload, 'perception')
        self.assertEqual([], replay.lane.forward_lane_spans)
        self.assertEqual(DecisionMode.STOP, DecisionEngine(self.settings).run(replay).mode)

    def test_wider_xy_tolerance_does_not_relax_height_or_heading(self):
        for first in ((20.15, 0, 0.11), (20.15, 0, 0)):
            end = (80, 0, 0.11) if first[2] else (20.15, 60, 0)
            api = Map({'a': [(0, 0, 0), (20, 0, 0)], 'b': [first, end]}, {'a': ['b']})
            lane = manager(api).update(ego(), force=True)
            self.assertEqual('disconnected_successor', lane.forward_reference_status)
        api = Map({'a': [(0, 0), (20, 0)], 'b': [(20.21, 0), (80, 0)]}, {'a': ['b']})
        self.assertEqual('disconnected_successor', manager(api).update(ego(), force=True).forward_reference_status)

    def test_join_rejection_logs_geometry_without_repeating_each_refresh(self):
        api = Map({'a': [(0, 0), (20, 0)], 'b': [(21, 0), (80, 0)]}, {'a': ['b']})
        route = manager(api)
        calls = []
        route.logger.warning = lambda *args: calls.append(args)
        route.update(ego(), force=True)
        route.update(ego(19.9), force=True)
        self.assertEqual(1, len(calls))
        checks = calls[0][-1]
        self.assertEqual(1, checks[0]['xy_gap_m'])
        self.assertEqual('position_or_height_gap', checks[0]['reason'])


if __name__ == '__main__':
    unittest.main()
