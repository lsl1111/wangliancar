"""P0 integration regressions using real algorithms, no SDK connection."""

import math
import unittest
from types import SimpleNamespace

from core.interfaces import DecisionTarget
from core.serialization import to_dict
from core.validation import validate_output
from core.safety_supervisor import SafetySupervisor
from core.obstacle_geometry import footprint_entry, longitudinal_extent
from members.decision.engine import DecisionEngine
from members.decision.settings import DecisionSettings
from members.planning.lane_planner import build_trajectory, PlannerSettings
from perception.perception_builder import PerceptionBuilder
from tests.test_decision import perception, add_target
from tests import test_perception as perception_fixtures
from tests.test_perception import FakeAdapter, FakeRouteManager


class P0HandoffTests(unittest.TestCase):
    def setUp(self):
        self.settings = DecisionSettings(front_offset_m=3.5)
        self.planner = PlannerSettings(front_offset_m=3.5, half_width_m=0.9)

    def chain(self, p, engine=None):
        d = (engine or DecisionEngine(self.settings)).run(p)
        validate_output(d, DecisionTarget, p)
        trajectory = build_trajectory(p, d, self.planner)
        self.assertTrue(trajectory.valid, trajectory.reason)
        return d, trajectory

    def test_aeb_static_boundary_is_bumper_clearance_once(self):
        p = perception(speed=0)
        p.scene_id = 1
        add_target(p, longitudinal=60, speed=0)
        d, t = self.chain(p)
        self.assertAlmostEqual(54, d.stop_distance)
        self.assertAlmostEqual(d.stop_distance, t.stop_distance)
        self.assertAlmostEqual(0.5, 60-2-3.5-t.points[-1].x)
        self.assertTrue(d.precision_stop and t.precision_stop)
        self.assertEqual(0.5, d.obstacle_clearance_m)

    def test_following_stopped_lead_keeps_task_clearance(self):
        p = perception(speed=0)
        p.scene_id = 25
        add_target(p, longitudinal=60, speed=0)
        d, t = self.chain(p)
        self.assertAlmostEqual(10.5, d.obstacle_clearances_m["1"])
        self.assertAlmostEqual(10.5, 60-2-3.5-t.points[-1].x)
        self.assertGreater(d.target_speed, 0)

    def test_generic_follow_retains_gap_when_lead_stops_and_resets_case(self):
        engine = DecisionEngine(self.settings)
        p = perception(speed=0, frame_id=10)
        p.scene_id = 11
        add_target(p, longitudinal=60, speed=3)
        d, _ = self.chain(p, engine)
        self.assertEqual(10.5, d.obstacle_clearances_m["1"])
        p.frame_id += 1
        p.targets[0].vx = 0
        d, t = self.chain(p, engine)
        self.assertEqual(10.5, d.obstacle_clearances_m["1"])
        self.assertAlmostEqual(10.5, 60-2-3.5-t.points[-1].x)
        p.case_id = "new-case"
        d, _ = self.chain(p, engine)
        self.assertEqual(0.5, d.obstacle_clearance_m)

    def test_adjacent_oncoming_car_does_not_reject_chain(self):
        p = perception(speed=0)
        car = add_target(p, longitudinal=35, speed=-5)
        car.y, car.lane_id, car.heading = 3.5, "other", math.pi
        d, t = self.chain(p)
        self.assertGreater(d.target_speed, 0)
        self.assertFalse(t.emergency_stop)
        self.assertFalse(t.stop_required)

    def test_emergency_static_stop_holds_and_releases_after_target_moves(self):
        engine = DecisionEngine(self.settings)
        p = perception(speed=3, frame_id=10)
        p.scene_id = 1
        target = add_target(p, longitudinal=7, speed=0)
        d, t = self.chain(p, engine)
        self.assertTrue(t.emergency_stop)
        for frame in (11, 12, 13):
            p.frame_id = frame
            p.ego.speed = p.ego.vx = 0
            d, t = self.chain(p, engine)
            self.assertEqual(0, d.target_speed)
            self.assertEqual(0, d.stop_distance)
            self.assertFalse(t.emergency_stop)
        target.x, target.vx = 30, 3
        p.frame_id = 14
        self.assertEqual(0, self.chain(p, engine)[0].target_speed)
        # Re-reading the same input cannot count as a second release frame.
        self.assertEqual(0, self.chain(p, engine)[0].target_speed)
        p.frame_id = 15
        self.assertGreater(self.chain(p, engine)[0].target_speed, 0)

    def test_emergency_static_stop_releases_after_target_disappears(self):
        engine = DecisionEngine(self.settings)
        p = perception(speed=3, frame_id=10)
        p.scene_id = 1
        add_target(p, longitudinal=7, speed=0)
        self.chain(p, engine)
        p.frame_id, p.ego.speed, p.ego.vx = 11, 0, 0
        self.chain(p, engine)
        p.targets = []
        p.frame_id = 12
        self.assertEqual(0, self.chain(p, engine)[0].target_speed)
        self.assertEqual(0, self.chain(p, engine)[0].target_speed)
        p.frame_id = 13
        self.assertGreater(self.chain(p, engine)[0].target_speed, 0)

    def test_interfering_oncoming_and_future_crossing_still_block(self):
        for y, vx, vy in ((0, -5, 0), (3.5, -5, -1), (5, 0, -2)):
            with self.subTest(y=y, vx=vx, vy=vy):
                p = perception(speed=0)
                car = add_target(p, longitudinal=35, speed=vx)
                car.y, car.vy, car.lane_id = y, vy, "other"
                d, _ = self.chain(p)
                self.assertEqual(0, d.target_speed)

    def test_planner_independently_rejects_predicted_intrusion(self):
        p = perception(speed=0)
        car = add_target(p, longitudinal=35, speed=-5)
        car.y, car.vy, car.lane_id = 3.5, -1, "other"
        d = DecisionTarget().bind(p)
        d.valid, d.target_speed = True, 5
        t = build_trajectory(p, d, self.planner)
        self.assertFalse(t.valid)
        self.assertIn("motion unsupported", t.reason)

    def test_oriented_contact_and_unknown_orientation_fallback(self):
        p = perception(speed=0)
        car = add_target(p, longitudinal=20, speed=0)
        reference = [(0, (0, 0)), (40, (40, 0))]
        self.assertAlmostEqual(18, footprint_entry(reference, car, 0.9))
        car.heading = math.pi/2
        self.assertAlmostEqual(19.1, footprint_entry(reference, car, 0.9))
        self.assertAlmostEqual(0.9, longitudinal_extent(car, 0))
        car.heading = None
        entry = footprint_entry(reference, car, 0.9)
        self.assertLess(entry, 18)
        self.assertAlmostEqual(math.hypot(4, 1.8)*0.5, longitudinal_extent(car, 0))

    def test_contract_fields_serialize_and_invalid_values_reject(self):
        p = perception(speed=0)
        add_target(p, longitudinal=60)
        d, _ = self.chain(p)
        self.assertTrue(to_dict(d)["precision_stop"])
        self.assertEqual(0.5, to_dict(d)["obstacle_clearance_m"])
        self.assertEqual({"1": 0.5}, to_dict(d)["obstacle_clearances_m"])
        for value in (-0.1, float("nan"), True):
            d.obstacle_clearance_m = value
            with self.assertRaises(ValueError):
                validate_output(d, DecisionTarget, p)
        d.obstacle_clearance_m = 0.5
        for values in ({"-1": 1}, {"01": 1}, {1: 1}, {"1": -0.1}, {"1": True}):
            d.obstacle_clearances_m = values
            with self.assertRaises(ValueError):
                validate_output(d, DecisionTarget, p)

    def test_multi_target_clearances_are_not_applied_to_other_objects(self):
        p = perception(speed=0)
        p.scene_id = 11
        add_target(p, longitudinal=30, speed=0)
        other = add_target(p, longitudinal=80, speed=3)
        other.id = 2
        d, t = self.chain(p)
        self.assertEqual({"1": 0.5, "2": 10.5}, d.obstacle_clearances_m)
        self.assertAlmostEqual(d.stop_distance, t.stop_distance)
        self.assertAlmostEqual(24, t.stop_distance)
        p.targets.reverse()
        swapped, path = self.chain(p)
        self.assertEqual(d.obstacle_clearances_m, swapped.obstacle_clearances_m)
        self.assertAlmostEqual(t.stop_distance, path.stop_distance)

    def test_signal_missing_blocks_required_but_not_optional_task(self):
        for scene_id, blocked in ((10, True), (20, True), (6, False)):
            p = perception(speed=0)
            p.scene_id = scene_id
            d, _ = self.chain(p)
            self.assertEqual(blocked, d.target_speed == 0)

    def test_confirmed_no_signal_does_not_block_signal_task_approach(self):
        p = perception(speed=0)
        p.scene_id = 10
        p.traffic.association_valid, p.traffic.signal_presence = True, "absent"
        d, _ = self.chain(p)
        self.assertGreater(d.target_speed, 0)

    def test_stale_green_is_rejected_by_decision_and_independent_safety(self):
        p = perception(speed=0)
        p.scene_id = 10
        p.traffic.valid = p.traffic.observed = True
        p.traffic.signal_state = "GREEN"
        p.source_status["traffic"] = {"usable": False, "quality": "stale"}
        d, t = self.chain(p)
        self.assertEqual(0, d.target_speed)
        # Even a bad decision claiming cruise cannot bypass the runtime gate.
        d.target_speed = 5
        assessment = SafetySupervisor(SimpleNamespace()).evaluate(p, d, t)
        self.assertEqual("required_traffic_unavailable", assessment.reason)

    def test_builder_retains_failed_nearest_signal_and_recovers(self):
        class Adapter(FakeAdapter):
            read_ok = False
            def read_traffic(self, lane):
                self.last_traffic_query = {"association_valid": True}
                return [{"opendrive_id": 10, "status": 2 if self.read_ok else 0,
                         "read_ok": self.read_ok, "stop_line_x": 25, "stop_line_y": 0},
                        {"opendrive_id": 11, "status": 2,
                         "stop_line_x": 50, "stop_line_y": 0}]
        adapter = Adapter()
        builder = PerceptionBuilder(adapter, FakeRouteManager())
        raw = {"gps": perception_fixtures.PerceptionTests._gps(), "targets": []}
        for ok in (False, True, False):
            adapter.read_ok = ok
            built = builder.build_from_raw(raw)
            self.assertTrue(built.traffic.required)
            self.assertEqual("present", built.traffic.signal_presence)
            self.assertEqual(ok, built.traffic.valid)
            self.assertEqual(ok, built.source_status["traffic"]["usable"])
            p = perception(speed=0)
            p.scene_id = 10
            p.traffic = built.traffic
            p.source_status["traffic"] = built.source_status["traffic"]
            d, _ = self.chain(p)
            self.assertEqual(not ok, d.target_speed == 0)

    def test_builder_confirmed_absence_and_behind_failed_lamp_are_optional(self):
        class Adapter(FakeAdapter):
            def read_traffic(self, lane):
                self.last_traffic_query = {"association_valid": True}
                return [{"opendrive_id": 10, "status": 0, "read_ok": False,
                         "stop_line_x": 25, "stop_line_y": 0}]
        builder = PerceptionBuilder(Adapter(), FakeRouteManager())
        raw = {"gps": dict(perception_fixtures.PerceptionTests._gps(), x=30), "targets": []}
        p = builder.build_from_raw(raw)
        self.assertEqual("absent", p.traffic.signal_presence)
        self.assertFalse(p.traffic.required)

    def test_builder_does_not_invent_an_orientation_for_missing_field(self):
        p = perception(speed=0)
        target = PerceptionBuilder._build_target({"id": 1, "x": 20, "y": 0,
                         "length": 4, "width": 1.8, "height": 1.5}, p.ego, 1.75)
        self.assertTrue(target.valid)
        self.assertIsNone(target.heading)
        self.assertIsNone(to_dict(target)["heading"])

    def test_signal_quality_cannot_be_overridden_by_usable_true(self):
        p = perception(speed=0)
        p.scene_id = 10
        p.traffic.valid = p.traffic.observed = True
        p.traffic.signal_state = "GREEN"
        p.source_status["traffic"] = {"usable": True, "quality": "stale"}
        d, t = self.chain(p)
        self.assertEqual(0, d.target_speed)
        safety = SafetySupervisor(SimpleNamespace()).evaluate(p, d, t)
        self.assertEqual("required_traffic_unavailable", safety.reason)

    def test_signal_fault_recovery_waits_for_distinct_good_frames(self):
        p = perception(speed=0, frame_id=10)
        p.scene_id = 10
        supervisor = SafetySupervisor(SimpleNamespace())
        d, t = self.chain(p)
        self.assertTrue(supervisor.evaluate(p, d, t).active)
        p.traffic.valid = p.traffic.observed = True
        p.traffic.signal_state = "GREEN"
        for frame, active in ((11, True), (11, True), (12, True), (13, False)):
            p.frame_id = frame
            d, t = self.chain(p)
            self.assertEqual(active, supervisor.evaluate(p, d, t).active)


if __name__ == "__main__":
    unittest.main()
