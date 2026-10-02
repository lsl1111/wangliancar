"""SDK-shaped static map data reaches the shared pipeline without scene logic."""

import json
import math
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

from core.interfaces import LaneContext
from core.serialization import perception_to_dict
from members import control_stub, decision_stub
from members.control.controller import ControlEngine
from members.decision.settings import DecisionSettings
from members.planning_stub import plan
from perception.perception_builder import PerceptionBuilder
from scripts.accept_scene import analyze_frames
from simone_platform.map_observations import MapObservationReader, signal_record, text
from simone_platform.simone_adapter import SimOneAdapter
from tests.control_benchmark import vehicle
from tests.test_scene_acceptance import frame


class Vector(list):
    def Size(self):
        return len(self)

    def GetElement(self, index):
        return self[index]


def pt(x, y, z=0.0):
    return NS(x=x, y=y, z=z)


def sign(identifier=9, value="20", unit="km/h", x=10, normal=None):
    return NS(id=identifier, type="1010203800001413", subType="", value=value,
              unit=unit, isDynamic=False, heading=normal or pt(-1, 0), pt=pt(x, 2),
              validities=Vector([NS(roadId=1, sectionIndex=0, fromLaneId=-1,
                                   toLaneId=-1, stopLineIds=Vector([4]),
                                   crosswalkIds=Vector([5]))]))


def parking(identifier=7):
    marker = NS(side="front", type="solid", color="white", width=0.13)
    return NS(id=identifier, pt=pt(20, 5), heading=pt(1, 0),
              boundaryKnots=Vector([pt(20, 3), pt(25, 3), pt(25, 6), pt(20, 6)]),
              front=marker, left=marker, rear=marker, right=marker)


def map_api(signs=None, parks=None):
    stop = NS(id=4, type="stopLine", pt=pt(40, 0),
              boundaryKnots=Vector([pt(40, -2), pt(40, 2)]))
    cross = NS(id=5, type="crosswalk", pt=pt(45, 0),
               boundaryKnots=Vector([pt(44, -2), pt(46, -2), pt(46, 2), pt(44, 2)]))
    return NS(getTrafficSignList=lambda: Vector([sign()] if signs is None else signs),
              getTrafficLightList=lambda: Vector([]),
              getParkingSpaceList=lambda: Vector([parking()] if parks is None else parks),
              pySimString=str, getStoplineList=lambda signal, lane: Vector([stop]),
              getCrosswalkList=lambda signal, lane: Vector([cross]))


class Lane(object):
    def update(self, ego):
        result = LaneContext()
        result.lane_id, result.valid = "1_0_-1", True
        result.center_line = [(0, 0, 0), (100, 0, 0)]
        result.left_boundary = [(0, 2, 0), (100, 2, 0)]
        result.right_boundary = [(0, -2, 0), (100, -2, 0)]
        result.lane_width, result.lane_width_valid = 4.0, True
        return result


class MapObservationTests(unittest.TestCase):
    def adapter(self, hdmap):
        adapter = SimOneAdapter(NS(vehicle_id="0", sensor_id="perfectPerception1"), NS())
        adapter.map_loaded, adapter.hdmap = True, hdmap
        adapter.pnc_api, adapter.sensor_api = NS(), NS()
        return adapter

    def perception(self, hdmap, x=20, scene=6):
        adapter = self.adapter(hdmap)
        raw = adapter._read_reference_data()
        raw.update(gps={"frame": 1, "timestamp": 50, "x": x, "y": 0.0,
                       "heading": 0.0, "vx": 0.0, "vy": 0.0, "age_ms": 0})
        builder = PerceptionBuilder(adapter, Lane(), scene_override=scene)
        return builder.build_from_raw(raw)

    def test_native_direction_vector_and_validity_ids_are_not_lost(self):
        record = signal_record(sign(normal=pt(0, 2, 0.25)))
        self.assertAlmostEqual(math.pi / 2, record["heading"])
        self.assertEqual([0.0, 2.0, 0.25], record["heading_vector"])
        self.assertEqual([4], record["validities"][0]["stop_line_ids"])
        self.assertEqual([5], record["validities"][0]["crosswalk_ids"])
        json.dumps(record, allow_nan=False)

    def test_bad_record_preserves_good_data_but_invalidates_catalog(self):
        reader = MapObservationReader(map_api(signs=[sign(), sign(10, normal=pt(float("nan"), 0))]))
        result = reader.catalog()
        self.assertEqual([9], [v["id"] for v in result["traffic_signs"]])
        self.assertFalse(result["traffic_signs_valid"])
        self.assertEqual(1, result["map_observation_status"]["traffic_signs"]["invalid_records"])

    def test_zero_heading_keeps_sign_but_does_not_invent_applicability(self):
        p = self.perception(map_api(signs=[sign(normal=pt(0, 0))]))
        self.assertTrue(p.traffic_signs_valid)
        self.assertEqual("heading_unresolved", p.speed_limit_observations[0]["reason"])
        self.assertEqual(-1, p.lane.speed_limit)

    def test_parking_geometry_order_and_unknown_occupancy_survive_json(self):
        p = self.perception(map_api())
        record = perception_to_dict(p)["parking_spaces"][0]
        self.assertTrue(p.parking_spaces_valid)
        self.assertEqual([[20.0, 3.0, 0.0], [20.0, 6.0, 0.0]], record["entrance_edge"])
        self.assertEqual("unknown", record["occupancy"])
        self.assertEqual("front", record["markings"]["front"]["side"])
        json.dumps(perception_to_dict(p), allow_nan=False)

    def test_invalid_parking_polygon_is_not_publishable_as_valid(self):
        invalid = parking(8)
        invalid.boundaryKnots = Vector([pt(1, 1)] * 4)
        result = MapObservationReader(map_api(parks=[parking(), invalid])).catalog()
        self.assertEqual(1, len(result["parking_spaces"]))
        self.assertFalse(result["parking_spaces_valid"])

    def test_static_objects_without_dynamic_lights_are_published_and_deduplicated(self):
        hdmap = map_api(signs=[sign(), sign(10)])
        del hdmap.getTrafficLightList
        p = self.perception(hdmap)
        self.assertTrue(p.map_stop_lines_valid)
        self.assertTrue(p.map_crosswalks_valid)
        self.assertEqual(1, len(p.map_stop_lines))
        self.assertEqual([9, 10], p.map_stop_lines[0]["association_signal_ids"])
        self.assertEqual(["1_0_-1"], p.map_crosswalks[0]["lane_ids"])
        meta = p.map_observation_status["map_stop_lines"]
        self.assertFalse(meta["coverage_complete"])
        self.assertFalse(meta["signal_catalog_complete"])
        self.assertEqual("static_map", meta["clock"])

    def test_empty_map_and_missing_api_are_distinguished_and_optional(self):
        empty = self.perception(map_api(signs=[], parks=[]))
        absent = self.perception(NS())
        self.assertTrue(empty.valid and absent.valid)
        self.assertTrue(empty.parking_spaces_valid)
        self.assertFalse(absent.parking_spaces_valid)
        self.assertEqual("empty", empty.map_observation_status["parking_spaces"]["reason"])
        self.assertEqual("api_missing", absent.map_observation_status["parking_spaces"]["reason"])

    def test_a_failed_association_query_keeps_other_observed_objects(self):
        hdmap = map_api(signs=[sign(10), sign(9)])
        original_query = hdmap.getStoplineList
        def query(signal, lane):
            if signal.id == 10:
                raise RuntimeError("one unavailable association")
            return original_query(signal, lane)
        hdmap.getStoplineList = query
        p = self.perception(hdmap)
        self.assertTrue(p.valid)
        self.assertEqual(1, len(p.map_stop_lines))
        self.assertFalse(p.map_stop_lines_valid)
        self.assertEqual("partial_read_failed", p.map_observation_status["map_stop_lines"]["reason"])
        self.assertTrue(p.map_crosswalks_valid)

    def test_map_text_retains_the_sdk_legacy_encoding(self):
        self.assertEqual("入口", text("入口".encode("gb18030") + b"\0extra"))

    def test_map_cache_is_detached_and_replacement_requeries(self):
        adapter = self.adapter(map_api())
        first = adapter._read_reference_data()
        first["parking_spaces"][0]["id"] = -1
        self.assertEqual(7, adapter._read_reference_data()["parking_spaces"][0]["id"])
        adapter.hdmap = map_api(parks=[parking(12)])
        adapter._reference_update = 0
        self.assertEqual(12, adapter._read_reference_data()["parking_spaces"][0]["id"])

    def test_limit_units_direction_lane_and_approach_are_checked(self):
        upcoming = self.perception(map_api(), x=5)
        self.assertEqual(-1, upcoming.lane.speed_limit)
        self.assertAlmostEqual(5, upcoming.speed_limit_observations[0]["along_distance"])
        self.assertEqual("upcoming_sign", upcoming.speed_limit_observations[0]["reason"])
        passed = self.perception(map_api(), x=20)
        self.assertAlmostEqual(20 / 3.6, passed.lane.speed_limit)
        for unit, normal, lane in (("", pt(-1, 0), -1), ("km/h", pt(1, 0), -1),
                                   ("km/h", pt(-1, 0), -2)):
            native = sign(unit=unit, normal=normal)
            native.validities[0].fromLaneId = native.validities[0].toLaneId = lane
            self.assertEqual(-1, self.perception(map_api(signs=[native])).lane.speed_limit)

    def test_api_map_facts_reach_real_decision_planning_and_control_in_multiple_scenes(self):
        for scene in (4, 6):
            p = self.perception(map_api(), scene=scene)
            with patch.object(decision_stub, "_ENGINE", decision_stub.DecisionEngine(DecisionSettings())), \
                    patch.object(control_stub, "_engine", ControlEngine(vehicle())):
                decision = decision_stub.decide(p)
                trajectory = plan(p, decision)
                control = control_stub.compute_control(p, trajectory)
            self.assertTrue(decision.valid, decision.errors)
            self.assertAlmostEqual(20 / 3.6, decision.target_speed)
            self.assertTrue(trajectory.valid, trajectory.reason)
            self.assertTrue(control.valid, control.diagnostics)
            self.assertGreater(control.throttle, 0)

    def test_acceptance_uses_delivered_map_data_instead_of_permanent_contract_gaps(self):
        for scene, key, count in ((7, "parking_spaces", 4), (20, "map_stop_lines", 2)):
            sample = frame(1)
            sample["scene_id"] = scene
            record = {"valid": True, "source": "hdmap", "lane_ids": ["1_0_-1"],
                      "boundary_knots": ([[0.0, 0.0, 0.0], [4.0, 0.0, 0.0],
                                          [4.0, 3.0, 0.0], [0.0, 3.0, 0.0]] if count == 4
                                         else [[0.0, 0.0, 0.0], [0.0, 4.0, 0.0]])}
            sample[key], sample[key + "_valid"] = [record], True
            if scene == 7:
                sample["lane"] = {}
            self.assertEqual("STRUCTURAL_PASS", analyze_frames([sample], scene, 1)["status"])
            sample[key] = []
            self.assertEqual("FAIL_OR_INCOMPLETE", analyze_frames([sample], scene, 1)["status"])
        sample = frame(1)
        sample["scene_id"] = 9
        sample["speed_limit_observations"] = [{"source": "hdmap", "applicable": True,
                                                 "speed_mps": 20 / 3.6}]
        self.assertEqual("STRUCTURAL_PASS", analyze_frames([sample], 9, 1)["status"])


if __name__ == "__main__":
    unittest.main()
