"""Multi-source decision acceptance cases, independent of SimOne and ROS."""

import copy
import os
import time
import unittest
from unittest.mock import patch

from core.interfaces import DecisionMode
from members.decision.engine import DecisionEngine
from members.decision.settings import DecisionSettings
from members.planning.lane_planner import PlannerSettings, build_trajectory
from tests.test_decision import add_red_light, add_target, perception


class FrameworkTests(unittest.TestCase):
    def setUp(self):
        self.settings = DecisionSettings(front_offset_m=3.5)
        self.engine = DecisionEngine(self.settings)

    def decide(self, p):
        result = self.engine.run(p)
        self.assertTrue(result.valid, result.reason)
        return result

    def test_static_target_approach_and_hold(self):
        p = perception(speed=0.0, frame_id=1)
        add_target(p, longitudinal=30.0, speed=0.0)
        first = self.decide(p)
        self.assertEqual(DecisionMode.KEEP_LANE, first.mode)
        self.assertGreater(first.target_speed, 0.0)
        self.assertGreater(first.stop_distance, 0.0)
        p = perception(speed=0.0, frame_id=2)
        p.ego.x = 20.0
        add_target(p, longitudinal=10.0, speed=0.0)
        near = self.decide(p)
        self.assertLess(near.stop_distance, first.stop_distance)
        p = perception(speed=0.0, frame_id=3)
        p.ego.x = 24.0
        add_target(p, longitudinal=6.0, speed=0.0)
        held = self.decide(p)
        self.assertEqual(DecisionMode.STOP, held.mode)
        self.assertEqual(0.0, held.stop_distance)

    def test_red_light_approaches_then_holds(self):
        p = perception(speed=0.0)
        add_red_light(p, 30.0)
        decision = self.decide(p)
        self.assertEqual(DecisionMode.KEEP_LANE, decision.mode)
        self.assertGreater(decision.target_speed, 0.0)
        self.assertAlmostEqual(26.2, decision.stop_distance)
        p = perception(speed=0.0, frame_id=8)
        add_red_light(p, 3.8)
        held = self.decide(p)
        self.assertEqual(DecisionMode.STOP, held.mode)
        self.assertEqual(0.0, held.stop_distance)

    def test_unknown_red_line_is_protection_not_remote_stop(self):
        for speed, mode, distance in ((0.0, DecisionMode.STOP, 0.0),
                                      (3.0, DecisionMode.EMERGENCY_BRAKE, -1.0)):
            p = perception(speed=speed)
            add_red_light(p, -1.0)
            result = DecisionEngine(self.settings).run(p)
            self.assertEqual(mode, result.mode)
            self.assertEqual(distance, result.stop_distance)
            self.assertIn("TRAFFIC_DISTANCE", result.reason)

    def test_multiple_constraints_do_not_depend_on_target_order(self):
        p = perception(speed=0.0)
        add_target(p, longitudinal=30.0, speed=5.0)
        near = add_target(p, longitudinal=3.0, speed=0.0,
                          same_lane_valid=False, lane_id="")
        near.id = 2
        first = DecisionEngine(self.settings).run(p)
        p.targets.reverse()
        second = DecisionEngine(self.settings).run(p)
        self.assertEqual(DecisionMode.STOP, first.mode)
        self.assertEqual((first.mode, first.target_speed, first.stop_distance),
                         (second.mode, second.target_speed, second.stop_distance))
        self.assertIn("TARGET_CONFLICT", first.reason)

    def test_static_red_and_follow_constraints_merge(self):
        p = perception(speed=0.0)
        add_red_light(p, 50.0)
        add_target(p, longitudinal=30.0, speed=5.0)
        other = add_target(p, longitudinal=20.0, speed=0.0)
        other.id = 2
        result = self.decide(p)
        self.assertEqual(DecisionMode.FOLLOW, result.mode)
        self.assertGreater(result.target_speed, 0.0)
        self.assertLess(result.stop_distance, 30.0)
        self.assertIn("FOLLOW:target", result.reason)
        self.assertIn("STOP:traffic", result.reason)

    def test_following_at_desired_gap_matches_lead_speed(self):
        p = perception(speed=6.0)
        # Desired net gap plus the front offset and aligned target half-length.
        add_target(p, longitudinal=self.settings.min_gap + 1.2*6 + 3.5 + 2,
                   speed=6.0)
        result = self.decide(p)
        self.assertEqual(DecisionMode.FOLLOW, result.mode)
        self.assertAlmostEqual(6.0, result.target_speed, places=6)

    def test_oncoming_while_ego_stopped_is_emergency(self):
        p = perception(speed=0.0)
        add_target(p, longitudinal=6.0, speed=-5.0)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, self.decide(p).mode)

    def test_verified_successor_target_keeps_current_lane_id(self):
        p = perception(speed=0.0)
        p.ego.x = 19.0
        p.lane.center_line = [(0.0, 0.0), (20.0, 0.0)]
        p.lane.forward_reference = [(0.0, 0.0), (20.0, 0.0), (100.0, 0.0)]
        p.lane.forward_reference_valid = True
        p.lane.forward_lane_ids = ["lane-1", "lane-2"]
        add_target(p, longitudinal=20.0, speed=5.0, lane_id="lane-2")
        result = self.decide(p)
        self.assertEqual(DecisionMode.FOLLOW, result.mode)
        self.assertEqual("lane-1", result.target_lane_id)
        p.lane.forward_reference[1] = (20.0, 0.1)
        self.assertNotEqual(DecisionMode.FOLLOW, DecisionEngine(self.settings).run(p).mode)

    def test_far_lateral_unknown_and_parallel_neighbor_do_not_block(self):
        p = perception(speed=0.0)
        t = add_target(p, longitudinal=3.0, speed=0.0,
                       same_lane_valid=False, lane_id="")
        t.y = 100.0  # Cached lateral field is intentionally inconsistent.
        self.assertEqual(DecisionMode.KEEP_LANE, self.decide(p).mode)
        p = perception(speed=0.0)
        t = add_target(p, longitudinal=5.0, speed=0.0, lane_id="lane-2")
        t.y = t.lateral_distance = 4.0
        self.assertEqual(DecisionMode.KEEP_LANE, DecisionEngine(self.settings).run(p).mode)

    def test_crossing_target_is_protection(self):
        p = perception(speed=2.0)
        t = add_target(p, longitudinal=15.0, speed=0.0,
                       same_lane_valid=False, lane_id="")
        t.y = t.lateral_distance = 4.0
        t.vy = -2.0
        result = self.decide(p)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, result.mode)
        self.assertIn("TARGET", result.reason)

    def test_required_source_first_fault_and_three_frame_recovery(self):
        first = perception(speed=4.0, targets_valid=False, frame_id=1)
        d = self.decide(first)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, d.mode)
        self.assertEqual(1, self.engine._blind_fault_count)
        self.decide(first)
        self.assertEqual(1, self.engine._blind_fault_count)
        self.decide(perception(speed=4.0, targets_valid=False, frame_id=2))
        self.assertTrue(self.engine._blind_stop)
        for frame in (3, 4):
            d = self.decide(perception(speed=0.0, frame_id=frame))
            self.assertEqual(DecisionMode.STOP, d.mode)
        d = self.decide(perception(speed=0.0, frame_id=5))
        self.assertEqual(DecisionMode.KEEP_LANE, d.mode)
        self.assertFalse(self.engine._blind_stop)

    def test_optional_targets_absent_still_cruise(self):
        for frame in (1, 2, 3):
            p = perception(speed=0.0, targets_valid=False, frame_id=frame)
            p.scene_id = 6
            self.assertEqual(DecisionMode.KEEP_LANE, self.decide(p).mode)

    def test_ground_truth_cannot_replace_sensor(self):
        p = perception(speed=0.0)
        add_target(p, longitudinal=20.0, speed=0.0)
        p.target_source = "ground_truth"
        p.source_status["targets"]["sensor_read_ok"] = False
        result = self.decide(p)
        self.assertEqual(DecisionMode.STOP, result.mode)
        self.assertIn("ground_truth_only", result.reason)

    def test_hold_requires_confirmed_clear_frames(self):
        p = perception(speed=0.0, frame_id=1)
        add_red_light(p, 3.8)
        self.assertEqual(DecisionMode.STOP, self.decide(p).mode)
        p = perception(speed=0.0, frame_id=2)
        self.assertEqual(DecisionMode.STOP, self.decide(p).mode)
        p = perception(speed=0.0, frame_id=3)
        self.assertEqual(DecisionMode.KEEP_LANE, self.decide(p).mode)

    def test_moving_static_moving_transition_keeps_stop_until_confirmed(self):
        outputs = []
        for frame, speed in ((1, 5.0), (2, 0.0), (3, 5.0), (4, 5.0)):
            p = perception(speed=0.0, frame_id=frame)
            add_target(p, longitudinal=30.0, speed=speed)
            outputs.append(self.decide(p))
        self.assertEqual([DecisionMode.FOLLOW, DecisionMode.KEEP_LANE,
                          DecisionMode.KEEP_LANE, DecisionMode.FOLLOW],
                         [item.mode for item in outputs])
        self.assertGreater(outputs[2].stop_distance, 0.0)

    def test_safe_moving_lead_disappearance_does_not_request_a_stop(self):
        p = perception(speed=0.0, frame_id=1)
        add_target(p, longitudinal=30.0, speed=5.0)
        self.assertEqual(DecisionMode.FOLLOW, self.decide(p).mode)
        for index in (2, 3):
            output = self.decide(perception(speed=0.0, frame_id=index))
            self.assertIn(output.mode, (DecisionMode.FOLLOW, DecisionMode.KEEP_LANE))
            self.assertGreater(output.target_speed, 0.0)
            self.assertFalse(self.engine._blind_stop)

    def test_target_source_failure_reasons_are_distinct(self):
        cases = (({"sensor_read_ok": False}, "sensor_read_failed"),
                 ({"sensor_presence": "not_configured"}, "not_configured"),
                 ({"quality": "stale", "usable": False}, "stale"),
                 ({"quality": "invalid", "usable": False}, "record_invalid"))
        for overrides, expected in cases:
            p = perception(speed=0.0, targets_valid=False)
            p.source_status["targets"].update(overrides)
            result = DecisionEngine(self.settings).run(p)
            self.assertEqual(DecisionMode.STOP, result.mode)
            self.assertIn(expected, result.reason)

    def test_real_planner_consumes_approach_hold_and_unknown_emergency(self):
        settings = PlannerSettings(front_offset_m=3.5, half_width_m=0.9)
        p = perception(speed=0.0)
        add_red_light(p, 30.0)
        d = DecisionEngine(self.settings).run(p)
        t = build_trajectory(p, d, settings)
        self.assertTrue(t.valid, t.reason)
        self.assertGreater(t.points[1].speed, 0.0)
        self.assertTrue(t.stop_required)
        p = perception(speed=0.0)
        add_red_light(p, -1.0)
        d = DecisionEngine(self.settings).run(p)
        t = build_trajectory(p, d, settings)
        self.assertEqual(DecisionMode.STOP, d.mode)
        self.assertTrue(t.valid, t.reason)
        self.assertTrue(all(point.speed == 0.0 for point in t.points))
        p = perception(speed=3.0)
        add_red_light(p, -1.0)
        d = DecisionEngine(self.settings).run(p)
        t = build_trajectory(p, d, settings)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, d.mode)
        self.assertTrue(t.emergency_stop)

    def test_sensor_read_failure_cannot_use_a_stale_valid_target_list(self):
        p = perception(speed=0.0)
        add_target(p, longitudinal=30.0, speed=5.0)
        p.source_status["targets"]["sensor_read_ok"] = False
        result = self.decide(p)
        self.assertEqual(DecisionMode.STOP, result.mode)
        self.assertIn("sensor_read_failed", result.reason)
        self.assertNotIn("FOLLOW_TARGET", result.reason)

    def test_hold_hysteresis_ignores_small_stop_point_jitter(self):
        for frame, distance, expected in ((1, 6.0, DecisionMode.STOP),
                                          (2, 6.3, DecisionMode.STOP),
                                          (3, 8.3, DecisionMode.STOP),
                                          (4, 8.3, DecisionMode.KEEP_LANE)):
            p = perception(speed=0.0, frame_id=frame)
            add_target(p, longitudinal=distance, speed=0.0)
            self.assertEqual(expected, self.decide(p).mode)

    def test_static_decision_is_consumed_by_real_planner(self):
        p = perception(speed=0.0)
        add_target(p, longitudinal=30.0, speed=0.0)
        d = self.decide(p)
        trajectory = build_trajectory(
            p, d, PlannerSettings(front_offset_m=3.5, half_width_m=0.9))
        self.assertTrue(trajectory.valid, trajectory.reason)
        self.assertTrue(trajectory.stop_required)
        self.assertGreater(trajectory.points[1].speed, 0.0)

    def test_successor_id_without_verified_route_is_unknown(self):
        p = perception(speed=0.0)
        p.ego.x = 19.0
        p.lane.center_line = [(0.0, 0.0), (20.0, 0.0)]
        p.lane.successor_lane_ids = ["lane-2"]
        add_target(p, longitudinal=20.0, speed=5.0, lane_id="lane-2")
        result = self.decide(p)
        self.assertEqual(DecisionMode.STOP, result.mode)
        self.assertIn("TARGET_UNKNOWN", result.reason)

    def test_later_successor_without_segment_boundary_is_protected(self):
        p = perception(speed=0.0)
        p.ego.x = 19.0
        p.lane.center_line = [(0.0, 0.0), (20.0, 0.0)]
        p.lane.forward_reference = [(0.0, 0.0), (20.0, 0.0),
                                    (40.0, 0.0), (100.0, 0.0)]
        p.lane.forward_reference_valid = True
        p.lane.forward_lane_ids = ["lane-1", "lane-2", "lane-3"]
        add_target(p, longitudinal=50.0, speed=5.0, lane_id="lane-3")
        result = self.decide(p)
        self.assertEqual(DecisionMode.STOP, result.mode)
        self.assertIn("segment boundary unavailable", result.reason)

    def test_hairpin_forward_target_is_not_discarded_by_ego_axis(self):
        p = perception(speed=0.0)
        p.lane.center_line = [(0.0, 0.0), (10.0, 0.0),
                              (10.0, 10.0), (0.0, 10.0)]
        target = add_target(p, longitudinal=0.0, speed=0.0)
        target.x, target.y = 0.0, 10.0
        target.longitudinal_distance = 0.0
        target.lateral_distance = 10.0
        target.vx = -3.0
        result = self.decide(p)
        self.assertEqual(DecisionMode.FOLLOW, result.mode)
        self.assertEqual("lane-1", result.target_lane_id)

    def test_target_type_and_equivalent_scene_do_not_change_behavior(self):
        outputs = []
        for scene_id, target_type in ((14, 1), (15, 2)):
            p = perception(speed=2.0)
            p.scene_id = scene_id
            target = add_target(p, longitudinal=30.0, speed=5.0)
            target.type = target_type
            result = DecisionEngine(self.settings).run(p)
            outputs.append((result.mode, result.target_speed,
                            result.stop_distance))
        self.assertEqual(outputs[0], outputs[1])

    def test_optional_data_does_not_create_stop_rules(self):
        p = perception(speed=0.0)
        p.scene_id = 6
        baseline = self.decide(p)
        p.environment = {"rain": 0.7}
        p.imu = {"yaw_rate": 1.0}
        p.radar_detections = [{"range": 3.0}]
        p.traffic_signs = [{"type": "unknown"}]
        p.environment_valid = p.imu_valid = p.traffic_signs_valid = False
        changed = self.decide(p)
        self.assertEqual((baseline.mode, baseline.target_speed,
                          baseline.stop_distance),
                         (changed.mode, changed.target_speed,
                          changed.stop_distance))

    def test_zero_and_unknown_limits_have_distinct_meanings(self):
        p = perception(speed=0.0)
        p.lane.speed_limit = 0.0
        held = self.decide(p)
        self.assertEqual(DecisionMode.STOP, held.mode)
        p.lane.speed_limit = -1.0
        moving = DecisionEngine(self.settings).run(p)
        self.assertEqual(DecisionMode.KEEP_LANE, moving.mode)
        self.assertGreater(moving.target_speed, 0.0)
        p.lane.speed_limit = float("nan")
        self.assertEqual(DecisionMode.KEEP_LANE,
                         DecisionEngine(self.settings).run(p).mode)

    def test_same_gps_id_with_changed_timestamp_is_not_new_fault_frame(self):
        p = perception(speed=0.0, targets_valid=False, frame_id=1)
        self.decide(p)
        p.timestamp += 1
        self.decide(p)
        self.assertEqual(1, self.engine._blind_fault_count)
        self.assertFalse(self.engine._blind_stop)

    def test_repeated_gps_frame_still_checks_expiry(self):
        p = perception(speed=0.0, frame_id=1)
        self.assertEqual(DecisionMode.KEEP_LANE, self.decide(p).mode)
        p.valid_until = time.monotonic() - 1.0
        expired = self.engine.run(p)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, expired.mode)
        self.assertIn("EGO_SOURCE", expired.reason)

    def test_equal_stop_candidates_are_order_independent(self):
        p = perception(speed=0.0)
        first = add_target(p, longitudinal=30.0, speed=0.0)
        first.id = 5
        second = add_target(p, longitudinal=30.0, speed=0.0)
        second.id = 3
        original = DecisionEngine(self.settings).run(p)
        p.targets.reverse()
        swapped = DecisionEngine(self.settings).run(p)
        self.assertEqual((original.mode, original.target_speed,
                          original.stop_distance, original.reason),
                         (swapped.mode, swapped.target_speed,
                          swapped.stop_distance, swapped.reason))
        self.assertIn("STOP_TARGET:id=3", original.reason)

    def test_front_offset_changes_stop_boundary_once(self):
        p = perception(speed=0.0)
        add_target(p, longitudinal=30.0, speed=0.0)
        short = DecisionEngine(DecisionSettings(front_offset_m=3.0)).run(p)
        long = DecisionEngine(DecisionSettings(front_offset_m=5.0)).run(p)
        self.assertAlmostEqual(2.0, short.stop_distance - long.stop_distance)
        p = perception(speed=0.0)
        add_red_light(p, 30.0)
        short = DecisionEngine(DecisionSettings(front_offset_m=3.0)).run(p)
        long = DecisionEngine(DecisionSettings(front_offset_m=5.0)).run(p)
        self.assertAlmostEqual(2.0, short.stop_distance - long.stop_distance)

    def test_small_residual_speed_stop_is_upgraded_by_real_planner(self):
        p = perception(speed=0.08)
        add_red_light(p, 3.8)
        decision = DecisionEngine(self.settings).run(p)
        self.assertEqual(DecisionMode.STOP, decision.mode)
        self.assertEqual(0.0, decision.stop_distance)
        trajectory = build_trajectory(
            p, decision, PlannerSettings(front_offset_m=3.5,
                                         half_width_m=0.9))
        self.assertTrue(trajectory.valid, trajectory.reason)
        self.assertTrue(trajectory.emergency_stop)

    def test_deprecated_environment_tuning_is_rejected(self):
        for name in ("NEVC_DECISION_LAUNCH_TTC_CAP", "NEVC_DECISION_STOP_MARGIN"):
            with self.assertRaises(ValueError):
                DecisionSettings.from_environment(environ={name: "3"})
        with self.assertRaises(ValueError):
            DecisionSettings(recovery_frames=0).validate()


if __name__ == "__main__":
    unittest.main()
