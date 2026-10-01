"""Offline planning invariants, using synthetic data (no SimOne connection)."""

import copy
import math
import random
import time
import unittest

from core.interfaces import DecisionMode, DecisionTarget, Perception, Target, Trajectory
from core.validation import validate_output
from members.planning.lane_planner import PlannerSettings, build_trajectory
from members.planning_stub import plan


def scene(speed=5.0, target=5.0, length=200.0):
    p = Perception()
    p.valid = p.ego.valid = p.lane.valid = p.targets_valid = True
    p.frame_id, p.timestamp, p.valid_until = 10, 1234, time.monotonic() + 60.0
    p.ego.speed = p.ego.vx = speed
    p.ego.gear = 1
    p.lane.lane_id = "lane-a"
    p.lane.center_line = [(0.0, 0.0), (length, 0.0)]
    d = DecisionTarget().bind(p)
    d.valid, d.target_speed, d.target_lane_id = True, target, p.lane.lane_id
    return p, d


def sensor_target(perception, x=30.0, y=0.0, vx=0.0, vy=0.0,
                  length=4.0, width=2.0):
    target = Target()
    target.valid = True
    target.id = 1
    target.x, target.y = x, y
    target.vx, target.vy = vx, vy
    target.length, target.width = length, width
    perception.targets = [target]
    perception.target_source = "sensor:test"
    perception.source_status["targets"] = {"usable": True}
    return target


class PlanningTests(unittest.TestCase):
    def assert_profile(self, result, decision, settings=None):
        settings = settings or PlannerSettings()
        self.assertTrue(result.valid, result.reason)
        validate_output(result, Trajectory, decision)
        self.assertEqual(0.0, result.points[0].relative_time)
        for a, b in zip(result.points, result.points[1:]):
            ds = math.hypot(b.x - a.x, b.y - a.y)
            dt = b.relative_time - a.relative_time
            self.assertGreater(dt, 0.0)
            self.assertAlmostEqual(ds, 0.5 * (a.speed + b.speed) * dt, places=7)
            if ds > 1e-6:
                acceleration = (b.speed ** 2 - a.speed ** 2) / (2.0 * ds)
                self.assertLessEqual(acceleration, settings.acceleration + 1e-7)
                self.assertGreaterEqual(acceleration, -settings.deceleration - 1e-7)

    def test_global_spacing_starts_at_projection_not_old_map_points(self):
        p, d = scene()
        p.ego.x = 7.3
        p.lane.center_line = [(x, 0.0) for x in (0, 5, 13.4, 20.2, 40, 200)]
        result = plan(p, d)
        self.assert_profile(result, d)
        self.assertAlmostEqual(7.3, result.points[0].x)
        self.assertAlmostEqual(67.3, result.points[-1].x)
        positions = [point.x for point in result.points]
        for i in range(61):
            self.assertTrue(any(abs(x - (7.3 + i)) < 1e-8 for x in positions))
        self.assertTrue(all(0 < b - a <= 1.0 + 1e-8 for a, b in zip(positions, positions[1:])))
        self.assertFalse(result.stop_required)
        self.assertEqual(p.frame_id, result.frame_id)
        self.assertEqual(p.timestamp, result.timestamp)
        self.assertEqual(d.valid_until, result.valid_until)

    def test_accelerates_from_actual_speed_and_can_launch_from_rest(self):
        for initial in (0.0, 2.0):
            p, d = scene(initial, 8.0)
            result = plan(p, d)
            self.assert_profile(result, d)
            self.assertEqual(initial, result.points[0].speed)
            self.assertGreater(result.points[1].speed, initial)
            self.assertAlmostEqual(8.0, result.points[-1].speed)

    def test_reduces_speed_without_instantaneous_jump(self):
        p, d = scene(10.0, 3.0)
        result = plan(p, d)
        self.assert_profile(result, d)
        self.assertEqual(10.0, result.points[0].speed)
        self.assertEqual(3.0, result.points[-1].speed)
        self.assertTrue(all(b.speed <= a.speed for a, b in zip(result.points, result.points[1:])))

    def test_stop_ends_at_requested_distance_with_zero_speed(self):
        p, d = scene(6.0, 0.0)
        p.ego.x = 12.2
        d.mode, d.stop_distance = DecisionMode.STOP, 17.35
        result = plan(p, d)
        self.assert_profile(result, d)
        self.assertEqual(6.0, result.points[0].speed)
        self.assertAlmostEqual(29.55, result.points[-1].x)
        self.assertEqual(0.0, result.points[-1].speed)
        self.assertTrue(result.stop_required)
        self.assertFalse(result.emergency_stop)

    def test_zero_target_while_moving_has_a_braking_profile(self):
        p, d = scene(4.0, 0.0)
        result = plan(p, d)
        self.assert_profile(result, d)
        self.assertEqual(4.0, result.points[0].speed)
        self.assertAlmostEqual(4.0, result.points[-1].x)
        self.assertEqual(0.0, result.points[-1].speed)
        self.assertFalse(result.emergency_stop)

    def test_map_end_is_a_stop_but_horizon_is_not(self):
        for length, stopped in ((15.0, True), (200.0, False)):
            p, d = scene(4.0, 4.0, length)
            result = plan(p, d)
            self.assert_profile(result, d)
            self.assertEqual(stopped, result.stop_required)
            self.assertEqual(0.0 if stopped else 4.0, result.points[-1].speed)

    def test_forward_reference_continues_at_current_lane_end(self):
        for x in (19.75, 20.0):
            p, d = scene(0.0, 3.0, 20.0)
            p.ego.x = x
            p.lane.forward_reference = [(0, 0), (20, 0), (180, 0)]
            p.lane.forward_reference_valid = True
            p.lane.forward_lane_ids = [p.lane.lane_id, "next"]
            p.lane.forward_reference_status = "map_end"
            result = plan(p, d)
            self.assert_profile(result, d)
            self.assertFalse(result.stop_required)
            self.assertGreater(result.points[-1].x, 70)
            self.assertEqual(3.0, result.points[-1].speed)
            self.assertGreater(result.points[1].speed, 0.0)

    def test_forward_reference_does_not_jump_to_nearby_future_lane(self):
        p, d = scene(0.0, 3.0, 20.0)
        p.ego.x, p.ego.y = 5.0, 0.1
        p.lane.forward_reference = [(0, 0), (20, 0), (100, 20), (5, 0.1), (-10, 0.1)]
        p.lane.forward_reference_valid = True
        p.lane.forward_lane_ids = [p.lane.lane_id, "future"]
        # The future exact match lies outside this frame's 60 m horizon.
        result = plan(p, d)
        self.assert_profile(result, d)
        self.assertEqual((5.0, 0.0), (result.points[0].x, result.points[0].y))

    def test_declared_forward_reference_requires_matching_current_prefix(self):
        for field in ("geometry", "lane_ids", "points"):
            p, d = scene(0.0, 3.0, 20.0)
            p.lane.forward_reference = [(0, 0), (20, 0), (180, 0)]
            p.lane.forward_reference_valid = True
            p.lane.forward_lane_ids = [p.lane.lane_id, "next"]
            if field == "geometry": p.lane.forward_reference[1] = (20, 0.1)
            if field == "lane_ids": p.lane.forward_lane_ids[0] = "wrong"
            if field == "points": p.lane.forward_reference = []
            self.assertFalse(plan(p, d).valid, field)

    def test_unavailable_continuation_still_stops_at_known_boundary(self):
        p, d = scene(1.0, 3.0, 15.0)
        p.lane.forward_reference = p.lane.center_line[:]
        p.lane.forward_reference_valid = True
        p.lane.forward_lane_ids = [p.lane.lane_id]
        p.lane.forward_reference_status = "ambiguous_successor"
        result = plan(p, d)
        self.assert_profile(result, d)
        self.assertTrue(result.stop_required)
        self.assertEqual(0.0, result.points[-1].speed)
        self.assertIn("ambiguous_successor", result.reason)

    def test_explicit_stop_is_kept_on_a_continuous_reference(self):
        p, d = scene(2.0, 0.0, 20.0)
        p.ego.x = 19.75
        d.mode, d.stop_distance = DecisionMode.STOP, 5.0
        p.lane.forward_reference = [(0, 0), (20, 0), (180, 0)]
        p.lane.forward_reference_valid = True
        p.lane.forward_lane_ids = [p.lane.lane_id, "next"]
        result = plan(p, d)
        self.assert_profile(result, d)
        self.assertTrue(result.stop_required)
        self.assertEqual(0.0, result.points[-1].speed)
        self.assertEqual(24.75, result.points[-1].x)

    def test_short_path_from_rest_has_a_nonzero_interior_speed(self):
        p, d = scene(0.0, 1.0, 0.4)
        result = plan(p, d)
        self.assert_profile(result, d)
        self.assertEqual(0.0, result.points[0].speed)
        self.assertGreater(result.points[1].speed, 0.0)
        self.assertEqual(0.0, result.points[-1].speed)

    def test_infeasible_stop_requests_emergency_in_existing_encoding(self):
        p, d = scene(10.0, 0.0)
        d.mode, d.stop_distance = DecisionMode.STOP, 1.0
        result = plan(p, d)
        self.assertTrue(result.valid, result.reason)
        validate_output(result, Trajectory, d)
        self.assertTrue(result.emergency_stop)
        self.assertEqual(0.0, result.target_speed)
        self.assertTrue(all(point.speed == 0.0 for point in result.points))
        self.assertTrue(all(point.x == p.ego.x for point in result.points))

    def test_emergency_does_not_require_map_or_target_observations(self):
        p, d = scene()
        p.lane.valid = p.targets_valid = False
        d.mode = DecisionMode.EMERGENCY_BRAKE
        result = plan(p, d)
        self.assertTrue(result.valid, result.reason)
        self.assertTrue(result.emergency_stop)
        validate_output(result, Trajectory, d)

    def test_stationary_hold_and_reference_end(self):
        for speed, target, x in ((0.0, 0.0, 0.0), (0.0, 4.0, 200.0), (4.0, 4.0, 200.0)):
            p, d = scene(speed, target)
            p.ego.x = x
            result = plan(p, d)
            self.assert_profile(result, d)
            self.assertEqual(speed > 0, result.emergency_stop)
            self.assertTrue(all(point.x == x and point.speed == 0 for point in result.points))

    def test_unknown_stop_distance_requests_brake_without_inventing_path(self):
        for speed in (0.0, 5.0):
            p, d = scene(speed=speed)
            d.mode = DecisionMode.STOP
            result = plan(p, d)
            self.assertTrue(result.valid, result.reason)
            self.assertEqual(speed > 0.0, result.emergency_stop)
            self.assertTrue(result.stop_required)
            self.assertTrue(all(point.x == p.ego.x for point in result.points))
            self.assertIn("unknown", result.reason)

    def test_duplicates_and_three_dimensional_map_points(self):
        p, d = scene()
        p.lane.center_line = [(0, 0, 0), (0, 0, 0), (20, 0, 1), (20, 0, 1), (200, 0, 2)]
        result = plan(p, d)
        self.assert_profile(result, d)
        self.assertEqual(60.0, result.points[-1].x)

    def test_curve_limits_speed_and_unwraps_heading(self):
        p, d = scene(1.0, 12.0)
        radius = 30.0
        angles = [1.3 + 0.02 * i for i in range(161)]
        p.lane.center_line = [(radius * math.cos(a), radius * math.sin(a)) for a in angles]
        p.ego.x, p.ego.y = p.lane.center_line[0]
        p.ego.heading = angles[0] + math.pi / 2
        p.ego.vx, p.ego.vy = math.cos(p.ego.heading), math.sin(p.ego.heading)
        result = plan(p, d)
        self.assert_profile(result, d)
        self.assertLess(max(point.speed for point in result.points), 8.0)
        self.assertTrue(all(abs(b.heading - a.heading) < 0.1
                            for a, b in zip(result.points, result.points[1:])))
        self.assertTrue(any(point.heading > math.pi for point in result.points))

    def test_sharp_corner_is_not_cut_by_resampling(self):
        p, d = scene(1.0, 2.0)
        p.lane.center_line = [(0, 0), (10.2, 0), (10.2, 30)]
        result = plan(p, d)
        self.assertFalse(result.valid)
        self.assertIn("corner", result.reason)

    def test_curve_speed_is_stable_across_ego_progress_and_sampling_phase(self):
        radius = 30.0
        road = [(radius * math.cos(i * 0.02), radius * math.sin(i * 0.02))
                for i in range(181)]
        expected_limit = math.sqrt(1.5 * radius)
        for angle in (1.0, 1.005, 1.01, 1.015, 1.019, 1.02):
            for spacing in (0.35, 1.0, 1.7):
                with self.subTest(angle=angle, spacing=spacing):
                    p, d = scene(5.5, 30.0 / 3.6)
                    p.lane.center_line = road
                    p.ego.x, p.ego.y = radius * math.cos(angle), radius * math.sin(angle)
                    p.ego.heading = angle + math.pi / 2
                    p.ego.vx = p.ego.speed * math.cos(p.ego.heading)
                    p.ego.vy = p.ego.speed * math.sin(p.ego.heading)
                    settings = PlannerSettings(spacing=spacing)
                    result = build_trajectory(p, d, settings)
                    self.assert_profile(result, d, settings)
                    self.assertFalse(result.emergency_stop)
                    self.assertAlmostEqual(expected_limit, result.points[-1].speed, delta=0.001)
                    self.assertLessEqual(max(point.speed for point in result.points), expected_limit)

    def test_pose_mismatch_reverse_and_unsupported_lane(self):
        for change in ("offset", "heading", "reverse", "reverse_velocity", "other_lane", "before"):
            p, d = scene()
            if change == "offset": p.ego.y = 0.6
            if change == "heading": p.ego.heading = math.pi
            if change == "reverse": p.ego.gear, p.ego.vx = -1, -5.0
            if change == "reverse_velocity": p.ego.vx = -5.0
            if change == "other_lane": d.target_lane_id = "lane-b"
            if change == "before": p.ego.x = -0.1
            result = plan(p, d)
            self.assertFalse(result.valid, change)
            self.assertEqual([], result.points)
            self.assertTrue(result.errors)

    def test_unknown_or_nonempty_targets_are_not_claimed_safe(self):
        for unavailable in (True, False):
            p, d = scene()
            if unavailable: p.targets_valid = False
            else: p.targets = [Target()]
            self.assertFalse(plan(p, d).valid)

    def test_sensor_obstacle_needs_measured_vehicle_dimensions(self):
        p, d = scene(0.0, 8.0)
        d.mode = DecisionMode.FOLLOW
        sensor_target(p)
        result = plan(p, d)
        self.assertFalse(result.valid)
        self.assertIn("measured vehicle", result.reason)

    def test_static_lead_allows_launch_then_stops_before_full_envelope(self):
        p, d = scene(0.0, 8.0)
        d.mode = DecisionMode.FOLLOW
        sensor_target(p)
        settings = PlannerSettings(front_offset_m=3.5, half_width_m=0.9)
        result = build_trajectory(p, d, settings)
        self.assert_profile(result, d, settings)
        self.assertTrue(result.stop_required)
        self.assertEqual(8.0, result.target_speed)
        self.assertGreater(result.points[1].speed, 0.0)
        self.assertEqual(0.0, result.points[-1].speed)
        envelope = 0.5 * math.hypot(4.0, 2.0) + 0.9
        self.assertLessEqual(result.points[-1].x + 3.5 + 4.0 + envelope,
                             30.0 + 1e-7)
        self.assertIn("obstacle stop", result.reason)

    def test_forward_lead_sequence_keeps_current_envelope_clear(self):
        settings = PlannerSettings(front_offset_m=3.5, half_width_m=0.9)
        for frame in range(5):
            p, d = scene(1.0 + frame * 0.5, 6.0)
            p.ego.x = frame * 0.5
            d.mode = DecisionMode.FOLLOW
            lead_x = 30.0 + frame * 0.3
            sensor_target(p, x=lead_x, vx=6.0)
            result = build_trajectory(p, d, settings)
            self.assert_profile(result, d, settings)
            envelope = 0.5 * math.hypot(4.0, 2.0) + 0.9
            self.assertLessEqual(result.points[-1].x + 3.5 + 4.0 + envelope,
                                 lead_x + 1e-7)
            self.assertGreater(result.points[1].speed, 0.0)

    def test_curved_reference_stops_before_target_envelope(self):
        p, d = scene(0.0, 5.0)
        d.mode = DecisionMode.FOLLOW
        radius = 30.0
        p.lane.center_line = [(radius * math.cos(i * 0.02),
                               radius * math.sin(i * 0.02)) for i in range(81)]
        p.ego.x, p.ego.y = p.lane.center_line[0]
        p.ego.heading = math.pi / 2
        target = sensor_target(p, x=radius * math.cos(0.7),
                               y=radius * math.sin(0.7))
        settings = PlannerSettings(front_offset_m=3.5, half_width_m=0.9)
        result = build_trajectory(p, d, settings)
        self.assert_profile(result, d, settings)
        self.assertTrue(result.stop_required)
        last = result.points[-1]
        front_x = last.x + settings.front_offset_m * math.cos(last.heading)
        front_y = last.y + settings.front_offset_m * math.sin(last.heading)
        envelope = 0.5 * math.hypot(target.length, target.width) + settings.half_width_m
        self.assertGreater(math.hypot(front_x - target.x, front_y - target.y), envelope)

    def test_known_reference_end_holds_even_with_target_list(self):
        p, d = scene(1.0, 3.0, 20.0)
        p.ego.x = 20.0
        sensor_target(p, x=30.0)
        result = build_trajectory(p, d, PlannerSettings(
            front_offset_m=3.5, half_width_m=0.9))
        self.assertTrue(result.valid, result.reason)
        self.assertTrue(result.emergency_stop)
        self.assertIn("end of known reference", result.reason)

    def test_verified_adjacent_target_outside_swept_path_does_not_stop(self):
        p, d = scene(0.0, 5.0)
        target = sensor_target(p, y=4.0)
        target.same_lane_valid = True
        target.lane_id = "lane-b"
        settings = PlannerSettings(front_offset_m=3.5, half_width_m=0.9)
        result = build_trajectory(p, d, settings)
        self.assert_profile(result, d, settings)
        self.assertFalse(result.stop_required)
        self.assertGreater(result.points[1].speed, 0.0)

    def test_crossing_oncoming_unknown_extent_and_untrusted_sources_fail(self):
        settings = PlannerSettings(front_offset_m=3.5, half_width_m=0.9)
        for change in ("crossing", "oncoming", "extent", "source"):
            p, d = scene(2.0, 5.0)
            target = sensor_target(p)
            if change == "crossing": target.vy = -1.0
            if change == "oncoming": target.vx = -2.0
            if change == "extent": target.length = 0.0
            if change == "source": p.target_source = "ground_truth"
            result = build_trajectory(p, d, settings)
            self.assertFalse(result.valid, change)
            self.assertEqual([], result.points)

    def test_fresh_optional_sensor_target_is_not_hidden_by_bad_presence_flag(self):
        p, d = scene(0.0, 5.0)
        p.scene_id = 6
        sensor_target(p)
        p.source_status["targets"]["sensor_presence"] = "not_configured"
        result = build_trajectory(p, d, PlannerSettings(
            front_offset_m=3.5, half_width_m=0.9))
        self.assert_profile(result, d)
        self.assertTrue(result.stop_required)
        self.assertIn("obstacle stop", result.reason)

    def test_vehicle_geometry_environment_requires_both_values(self):
        import os
        from unittest.mock import patch
        with patch.dict(os.environ, {"NEVC_VEHICLE_FRONT_OFFSET_M": "3.5",
                                  "NEVC_VEHICLE_HALF_WIDTH_M": "0.9"}):
            settings = PlannerSettings.from_environment()
            settings.validate()
            self.assertEqual(3.5, settings.front_offset_m)
            self.assertEqual(0.9, settings.half_width_m)
        with self.assertRaises(ValueError):
            PlannerSettings(front_offset_m=3.5).validate()
        p, d = scene()
        with patch.dict(os.environ, {"NEVC_VEHICLE_FRONT_OFFSET_M": "bad"}):
            failed = plan(p, d)
        self.assertFalse(failed.valid)
        self.assertIn("planning override", failed.reason)

    def test_optional_target_absence_or_unknown_configuration_allows_lane_reference(self):
        for scene_id, presence in ((4, "not_configured"), (6, "not_configured"),
                                   (6, "unknown"), (6, "configured")):
            p, d = scene(0.0, 2.0)
            p.scene_id, p.targets_valid = scene_id, False
            p.source_status["targets"] = {"sensor_presence": presence}
            self.assert_profile(plan(p, d), d)
            self.assertIn("unverified", plan(p, d).reason)
            # Ground Truth remains diagnostic when there is no target sensor.
            p.targets_valid, p.targets = True, [Target()]
            p.target_source = "ground_truth"
            self.assert_profile(plan(p, d), d)
        for scene_id, presence in ((1, "not_configured"), (0, "not_configured"),
                                   (1, "unknown"), (1, "configured")):
            p, d = scene(0.0, 2.0)
            p.scene_id, p.targets_valid = scene_id, False
            p.source_status["targets"] = {"sensor_presence": presence}
            self.assertFalse(plan(p, d).valid)

    def test_fresh_optional_sensor_target_still_requires_obstacle_aware_planning(self):
        p, d = scene(0.0, 2.0)
        p.scene_id, p.target_source = 6, "sensor:optional"
        p.source_status["targets"] = {"usable": True, "sensor_presence": "configured"}
        p.targets = [Target()]
        result = plan(p, d)
        self.assertFalse(result.valid)
        self.assertIn("obstacle-aware", result.reason)

    def test_follow_only_consumes_supplied_speed_no_following_policy(self):
        p, d = scene(6.0, 3.0)
        d.mode = DecisionMode.FOLLOW
        result = plan(p, d)
        self.assert_profile(result, d)
        self.assertEqual(3.0, result.points[-1].speed)

    def test_invalid_and_expired_inputs_fail_without_partial_points(self):
        for change in ("perception", "ego", "lane", "decision", "frame", "timestamp", "expiry",
                       "speed", "heading", "coordinate", "point", "negative_stop", "negative_speed",
                       "velocity", "gear"):
            p, d = scene()
            if change == "perception": p.valid = False
            if change == "ego": p.ego.valid = False
            if change == "lane": p.lane.valid = False
            if change == "decision": d.valid = False
            if change == "frame": d.frame_id += 1
            if change == "timestamp": d.timestamp += 1
            if change == "expiry": d.valid_until = time.monotonic() - 1
            if change == "speed": d.target_speed = float("nan")
            if change == "heading": p.ego.heading = float("inf")
            if change == "coordinate": p.ego.x = float("nan")
            if change == "point": p.lane.center_line = [(0, 0), (10, float("inf"))]
            if change == "negative_stop": d.stop_distance = -0.5
            if change == "negative_speed": p.ego.speed = -1.0
            if change == "velocity": p.ego.vx = float("nan")
            if change == "gear": p.ego.gear = "D"
            result = plan(p, d)
            self.assertFalse(result.valid, change)
            self.assertEqual([], result.points)
        self.assertFalse(plan(None, None).valid)

    def test_empty_single_and_malformed_reference(self):
        for points in ([], [(0, 0)], [(0, 0), (0, 0)], [(0, 0), None], [(0, 0), (True, 2)]):
            p, d = scene()
            p.lane.center_line = points
            self.assertFalse(plan(p, d).valid)

    def test_settings_validate_and_horizon_is_configurable(self):
        p, d = scene()
        self.assertFalse(build_trajectory(p, d, PlannerSettings(spacing=0)).valid)
        self.assertFalse(build_trajectory(p, d, PlannerSettings(horizon=float("inf"))).valid)
        settings = PlannerSettings(horizon=25.0, spacing=0.5)
        result = build_trajectory(p, d, settings)
        self.assert_profile(result, d, settings)
        self.assertEqual(25.0, result.points[-1].x)

    def test_inputs_are_not_modified_and_output_is_repeatable(self):
        p, d = scene()
        before_p, before_d = copy.deepcopy(p.__dict__), copy.deepcopy(d.__dict__)
        a, b = plan(p, d), plan(p, d)
        self.assertEqual(before_d, d.__dict__)
        self.assertEqual(before_p["lane"].__dict__, p.lane.__dict__)
        self.assertEqual(before_p["ego"].__dict__, p.ego.__dict__)
        self.assertEqual([point.__dict__ for point in a.points], [point.__dict__ for point in b.points])

    def test_varied_straight_profiles_obey_kinematics(self):
        rng = random.Random(927)
        for _ in range(40):
            initial, desired = rng.uniform(0, 12), rng.uniform(0.1, 12)
            p, d = scene(initial, desired, rng.uniform(80, 250))
            p.ego.x = rng.uniform(0, 10)
            result = plan(p, d)
            self.assert_profile(result, d)
            self.assertEqual(initial, result.points[0].speed)
            self.assertLessEqual(result.points[-1].x, p.ego.x + 60 + 1e-7)


if __name__ == "__main__":
    unittest.main()
