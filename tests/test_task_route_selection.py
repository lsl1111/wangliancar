"""Task waypoints disambiguate only geometrically connected HDMap branches."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from tests.test_route_continuation import Map, NativeId, manager, ego


def branch_map():
    api = Map({'a': [(0, 0), (20, 0)],
               'b': [(20, 0), (21, .1), (40, 8)],
               'c': [(20, 0), (21, -.1), (40, -8)],
               'd': [(40, 8), (60, 16)]}, {'a': ['b', 'c'], 'b': ['d']})
    api.pySimString = NativeId
    api.getLaneWidth = lambda *args: SimpleNamespace(exists=True, width=3.5)
    return api


class TaskRouteSelectionTests(unittest.TestCase):
    def test_reachable_goal_selects_connected_path_across_multiple_lanes(self):
        route = manager(branch_map())
        route.set_route_hint([(0, 0), (55, 14)], True, ('case', 'task'))
        lane = route.update(ego(), force=True)
        self.assertEqual(['a', 'b', 'd'], lane.forward_lane_ids)
        self.assertEqual('map_end', lane.forward_reference_status)
        self.assertNotIn((55, 14), lane.forward_reference)

    def test_no_invalid_unreachable_or_equal_branch_hint_keeps_boundary(self):
        for hint, valid in (([], False), ([(0, 0), (float('nan'), 1)], True),
                            ([(0, 0), (100, 100)], True),
                            ([(0, 0), (20.1, 0)], True)):
            route = manager(branch_map())
            route.set_route_hint(hint, valid)
            lane = route.update(ego(), force=True)
            self.assertEqual(['a'], lane.forward_lane_ids)
            self.assertEqual('ambiguous_successor', lane.forward_reference_status)

    def test_hint_task_change_invalidates_positive_selection(self):
        route = manager(branch_map())
        route.set_route_hint([(0, 0), (40, 8)], True, ('one', 'task'))
        self.assertEqual(['a', 'b', 'd'], route.update(ego(), True).forward_lane_ids)
        route.set_route_hint([(0, 0), (40, -8)], True, ('two', 'task'))
        self.assertEqual(['a', 'c'], route.update(ego(), True).forward_lane_ids)
        route.set_route_hint([], False, ('two', 'task'))
        self.assertEqual('ambiguous_successor', route.update(ego(), True).forward_reference_status)

    def test_disconnected_or_unreadable_goal_branch_cannot_be_selected(self):
        for change in ('position', 'height', 'missing', 'links'):
            api = branch_map()
            if change == 'position': api.segments['b'][0] = (21, 0)
            if change == 'height': api.segments['b'] = [(x, y, 1) for x, y in api.segments['b']]
            if change == 'missing': del api.segments['b']
            if change == 'links': api.failed_links.add('b')
            route = manager(api)
            route.set_route_hint([(0, 0), (55, 14)], True)
            self.assertEqual('ambiguous_successor', route.update(ego(), True).forward_reference_status)

    def test_budget_exhaustion_and_competitive_query_failure_keep_boundary(self):
        api = branch_map()
        route = manager(api)
        route.set_route_hint([(0, 0), (55, 14)], True)
        with patch('perception.route_manager.MAX_ROUTE_SEARCH_STATES', 1):
            self.assertEqual('ambiguous_successor', route.update(ego(), True).forward_reference_status)
        del api.segments['c']
        self.assertEqual('ambiguous_successor', route.update(ego(), True).forward_reference_status)

    def test_delayed_native_ego_lane_at_join_promotes_only_verified_successor(self):
        api = branch_map()
        route = manager(api)
        route.set_route_hint([(0, 0), (40, 8)], True)
        route.update(ego(19), True)
        lane = route.update(ego(20.02, .002, .1), True)
        self.assertEqual('b', lane.lane_id)
        self.assertEqual((20., 0., 0.), lane.center_line[0])
        self.assertEqual(lane.center_line, lane.forward_reference[:len(lane.center_line)])

    def test_unselected_branch_vertical_error_and_unknown_branch_do_not_promote(self):
        for change in ('branch', 'height', 'unknown'):
            route = manager(branch_map())
            route.set_route_hint([(0, 0), (40, 8)], change != 'unknown')
            route.update(ego(19), True)
            value = ego(22, -.51, -.1) if change == 'branch' else ego(20.02, .002, .1)
            if change == 'height': value.z = 1
            self.assertEqual('a', route.update(value, True).lane_id)
