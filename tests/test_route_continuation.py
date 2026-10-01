"""Map segments, SDK ID types and conservative continuation boundaries."""

import json
import math
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from core.geometry import project_polyline
from core.interfaces import EgoState, Perception
from core.serialization import to_dict
from perception.route_manager import RouteManager
from scripts.replay_decision import _assign


class Vector(list):
    def Size(self):
        return len(self)

    def GetElement(self, index):
        return self[index]


class NativeId(object):
    def __init__(self, text):
        self.text = text

    def GetString(self):
        return self.text


class Map(object):
    def __init__(self, segments, successors=None, predecessors=None, current="a"):
        self.segments = segments
        self.successors = successors or {}
        self.predecessors = predecessors or {}
        self.current = current
        self.samples = []
        self.failed_links = set()

    @staticmethod
    def pySimPoint3D(*args):
        return args

    def getNearMostLane(self, position):
        return SimpleNamespace(exists=True, laneId=NativeId(self.current))

    @staticmethod
    def identity(native):
        # Actual pySimOneIO queries consume pySimString, not serialized str.
        if not isinstance(native, NativeId):
            raise TypeError("SDK requires a native lane ID")
        return native.GetString()

    def getLaneSample(self, native):
        identity = self.identity(native)
        self.samples.append(identity)
        if identity not in self.segments:
            return SimpleNamespace(exists=False)
        points = Vector([SimpleNamespace(x=float(p[0]), y=float(p[1]),
                                         z=float(p[2]) if len(p) > 2 else 0.0)
                         for p in self.segments[identity]])
        return SimpleNamespace(exists=True, laneInfo=SimpleNamespace(
            centerLine=points, leftBoundary=Vector(), rightBoundary=Vector()))

    def getLaneLink(self, native):
        identity = self.identity(native)
        if identity in self.failed_links:
            raise RuntimeError("link query failed")
        return SimpleNamespace(exists=True, laneLink=SimpleNamespace(
            leftNeighborLaneId=NativeId(""), rightNeighborLaneId=NativeId(""),
            successorLaneIds=Vector([NativeId(x) for x in self.successors.get(identity, [])]),
            predecessorLaneIds=Vector([NativeId(x) for x in self.predecessors.get(identity, [])])))


def ego(x=19.75, y=0.0, heading=0.0):
    value = EgoState()
    value.x, value.y, value.heading, value.valid = x, y, heading, True
    return value


def manager(map_api):
    return RouteManager(SimpleNamespace(map_loaded=True, hdmap=map_api),
                        SimpleNamespace(warning=lambda *args: None))


class RouteContinuationTests(unittest.TestCase):
    def test_unique_successor_reaches_curve_and_preserves_local_lane(self):
        curve = [(20 + 30 * math.sin(i * 0.02), 30 * (1 - math.cos(i * 0.02)))
                 for i in range(101)]
        api = Map({"a": [(0, 0), (20, 0)], "b": curve}, {"a": ["b"]})
        lane = manager(api).update(ego(), force=True)
        self.assertTrue(lane.valid and lane.forward_reference_valid)
        self.assertEqual("a", lane.lane_id)
        self.assertEqual(["a", "b"], lane.forward_lane_ids)
        self.assertEqual("map_end", lane.forward_reference_status)
        self.assertEqual([(0.0, 0.0, 0.0), (20.0, 0.0, 0.0)], lane.center_line)
        self.assertEqual(lane.center_line, lane.forward_reference[:2])
        self.assertGreater(lane.forward_reference[-1][1], 40.0)
        projection = project_polyline(lane.forward_reference, 19.75, 0)
        self.assertEqual(0, projection["index"])

    def test_reversed_successor_sample_uses_its_predecessors(self):
        api = Map({"a": [(0, 0), (20, 0)], "b": [(40, 0), (20, 0)],
                   "c": [(40, 0), (100, 0)]}, {"a": ["b"]}, {"b": ["c"]})
        lane = manager(api).update(ego(), force=True)
        self.assertEqual(["a", "b", "c"], lane.forward_lane_ids)
        self.assertEqual(100.0, lane.forward_reference[-1][0])
        self.assertEqual("map_end", lane.forward_reference_status)

    def test_reversed_current_lane_uses_forward_travel_links(self):
        api = Map({"a": [(0, 0), (20, 0)], "b": [(-50, 0), (0, 0)]},
                  predecessors={"a": ["b"]})
        lane = manager(api).update(ego(0.25, heading=math.pi), force=True)
        self.assertEqual(["a", "b"], lane.forward_lane_ids)
        self.assertEqual(["b"], lane.successor_lane_ids)
        self.assertEqual(-50.0, lane.forward_reference[-1][0])

    def test_successor_orients_at_join_after_current_lane_turns(self):
        # The join points north even though the ego is still heading east.
        api = Map({"a": [(0, 0), (20, 0), (20, 20)],
                   "b": [(20, 40), (20, 20)]}, {"a": ["b"]}, {"b": []})
        lane = manager(api).update(ego(5), force=True)
        self.assertEqual(["a", "b"], lane.forward_lane_ids)
        self.assertEqual((20.0, 40.0, 0.0), lane.forward_reference[-1])

    def test_join_rounding_does_not_create_tiny_spurious_turn(self):
        api = Map({"a": [(0, 0), (20, 0)],
                   "b": [(20.005, 0.005), (21, 0), (80, 0)]}, {"a": ["b"]})
        lane = manager(api).update(ego(), force=True)
        self.assertEqual(["a", "b"], lane.forward_lane_ids)
        self.assertEqual((21.0, 0.0, 0.0), lane.forward_reference[2])

    def test_missing_branch_or_bad_connection_keeps_current_coverage(self):
        cases = [({"a": ["missing"]}, {}, "successor_unavailable"),
                 ({"a": ["b", "c"]}, {}, "ambiguous_successor"),
                 ({"a": ["b"]}, {"b": [(21, 0), (80, 0)]}, "disconnected_successor"),
                 ({"a": ["b"]}, {"b": [(20, 0, 1), (80, 0, 1)]}, "disconnected_successor"),
                 ({"a": ["b"]}, {"b": [(20, 0), (20, 60)]}, "disconnected_successor"),
                 ({"a": ["b"]}, {"b": [(20, 0), (80, float("nan"))]}, "successor_query_failed")]
        for links, additional, status in cases:
            with self.subTest(status=status, additional=additional):
                api = Map(dict({"a": [(0, 0), (20, 0)]}, **additional), links)
                lane = manager(api).update(ego(), force=True)
                self.assertTrue(lane.valid and lane.forward_reference_valid)
                self.assertEqual(["a"], lane.forward_lane_ids)
                self.assertEqual(lane.center_line, lane.forward_reference)
                self.assertEqual(status, lane.forward_reference_status)

    def test_link_failure_is_distinct_from_a_confirmed_terminal(self):
        api = Map({"a": [(0, 0), (20, 0)]})
        api.failed_links.add("a")
        lane = manager(api).update(ego(), force=True)
        self.assertTrue(lane.valid)
        self.assertEqual("link_unavailable", lane.forward_reference_status)
        api.failed_links.clear()
        lane = manager(api).update(ego(), force=True)
        self.assertEqual("map_end", lane.forward_reference_status)

    def test_expansion_budget_and_cycles_cannot_loop_or_read_distant_maps(self):
        api = Map({"a": [(0, 0), (140, 0)], "b": [(140, 0), (200, 0)]},
                  {"a": ["b"]})
        lane = manager(api).update(ego(0), force=True)
        self.assertEqual(["a"], api.samples)
        self.assertEqual("lookahead_limit", lane.forward_reference_status)
        with patch("perception.route_manager.MAX_FORWARD_LANES", 2):
            api = Map({"a": [(0, 0), (20, 0)], "b": [(20, 0), (40, 0)]},
                      {"a": ["b"], "b": ["c"]})
            lane = manager(api).update(ego(), force=True)
            self.assertEqual("lane_limit", lane.forward_reference_status)
            self.assertEqual(["a", "b"], api.samples)
        api = Map({"a": [(0, 0), (20, 0)], "b": [(20, 0), (40, 0)]},
                  {"a": ["b"], "b": ["a"]})
        lane = manager(api).update(ego(), force=True)
        self.assertEqual("cycle", lane.forward_reference_status)
        self.assertEqual(["a", "b"], api.samples)

    def test_point_budget_stops_before_adding_an_oversized_successor(self):
        api = Map({"a": [(0, 0), (20, 0)],
                   "b": [(20, 0), (21, 0), (22, 0), (23, 0)]}, {"a": ["b"]})
        with patch("perception.route_manager.MAX_REFERENCE_POINTS", 4):
            lane = manager(api).update(ego(), force=True)
        self.assertEqual("point_limit", lane.forward_reference_status)
        self.assertEqual(lane.center_line, lane.forward_reference)

    def test_lane_crossing_refreshes_even_inside_cache_interval(self):
        api = Map({"a": [(0, 0), (20, 0)], "b": [(20, 0), (180, 0)]},
                  {"a": ["b"]})
        route = manager(api)
        now = time.monotonic()
        with patch("time.monotonic", return_value=now):
            first = route.update(ego(), force=True)
            cached = route.update(ego(19.9))
            self.assertEqual(first.forward_lane_ids, cached.forward_lane_ids)
            self.assertEqual(["a", "b"], api.samples)
            api.current = "b"
            crossed = route.update(ego(20.01))
        self.assertEqual("b", crossed.lane_id)
        self.assertEqual(["b"], crossed.forward_lane_ids)

    def test_serialization_and_replay_preserve_plain_contract_fields(self):
        api = Map({"a": [(0, 0), (20, 0)], "b": [(20, 0), (80, 0)]}, {"a": ["b"]})
        p = Perception()
        p.lane = manager(api).update(ego(), force=True)
        payload = json.loads(json.dumps(to_dict(p), allow_nan=False))
        restored = _assign(Perception(), payload, "perception")
        self.assertEqual(["a", "b"], restored.lane.forward_lane_ids)
        self.assertEqual(payload["lane"]["forward_reference"], restored.lane.forward_reference)
        self.assertTrue(restored.lane.forward_reference_valid)
        self.assertEqual("map_end", restored.lane.forward_reference_status)


if __name__ == "__main__":
    unittest.main()
