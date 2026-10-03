"""Forward planning gates, geometry and braking limits; synthetic inputs only."""

import copy
import math
import os
import unittest
from unittest.mock import patch

from core.interfaces import DecisionMode
from members import control_stub, decision_stub
from members.control.controller import ControlEngine
from members.decision.settings import DecisionSettings
from members.planning.lane_planner import PlannerSettings, build_trajectory
from members.planning_stub import plan
from tests import test_planning as fixtures
from tests import test_map_observations as map_fixtures
from tests.test_control import calibration
from tests.test_decision import add_target, perception


def settings(**overrides):
    return PlannerSettings(front_offset_m=3.9187, half_width_m=0.9, **overrides)


def limit(p, distance, speed, **overrides):
    p.traffic_signs_valid = True
    item = {"source": "hdmap", "applicable": True, "active": distance <= 0,
            "along_distance": distance, "speed_mps": speed, "sign_id": "test"}
    item.update(overrides)
    p.speed_limit_observations.append(item)
    return item


def verified_target(p, **overrides):
    target = fixtures.sensor_target(p, **overrides)
    target.heading = 0.0
    target.same_lane_valid, target.lane_id = True, p.lane.lane_id
    p.lane.lane_width_valid, p.lane.lane_width = True, 3.5
    return target


class PlanningConstraintTests(unittest.TestCase):
    def assert_profile(self, trajectory, decision, config):
        fixtures.PlanningTests.assert_profile(self, trajectory, decision, config)

    def test_micro_lateral_motion_expands_clearance_instead_of_disappearing(self):
        config = settings()
        for vy in (-0.01, 0.01):
            p, d = fixtures.scene(2.0, 5.0)
            target = verified_target(p, vx=5.0, vy=vy)
            result = build_trajectory(p, d, config)
            self.assert_profile(result, d, config)
            extent = target.length * 0.5
            self.assertLessEqual(result.points[-1].x + config.front_offset_m
                                 + config.obstacle_margin_m + extent, target.x + 1e-7)
            target.vy = 0.0
            stationary_lateral = build_trajectory(p, d, config)
            # Expansion is lateral, not an extra longitudinal business gap.
            self.assertAlmostEqual(result.stop_distance, stationary_lateral.stop_distance)

    def test_lateral_exception_needs_geometry_and_verified_current_lane(self):
        for change in ("unverified", "other_lane", "width", "heading", "edge",
                       "crossing", "oncoming"):
            p, d = fixtures.scene(2.0, 5.0)
            target = verified_target(p, vx=5.0, vy=0.01)
            if change == "unverified": target.same_lane_valid = False
            if change == "other_lane": target.lane_id = "other"
            if change == "width": p.lane.lane_width_valid = False
            if change == "heading": target.heading = float("nan")
            if change == "edge": target.y = 0.74
            if change == "crossing": target.vy = 1.0
            if change == "oncoming": target.vx = -2.0
            result = build_trajectory(p, d, settings())
            self.assertFalse(result.valid, change)
            self.assertEqual([], result.points)

    def test_lateral_guard_covers_current_braking_time(self):
        p, d = fixtures.scene(8.0, 8.0)
        verified_target(p, x=50.0, y=0.6, vx=8.0, vy=0.04)
        # Three seconds would fit (0.72 m); braking takes four, reaching 0.76 m.
        result = build_trajectory(p, d, settings())
        self.assertFalse(result.valid)

    def test_curved_lane_motion_uses_target_local_tangent(self):
        p, d = fixtures.scene(1.0, 5.0)
        radius, angle = 30.0, 0.7
        p.lane.center_line = [(radius*math.cos(i*0.02), radius*math.sin(i*0.02))
                              for i in range(81)]
        p.ego.x, p.ego.y, p.ego.heading = radius, 0.0, math.pi/2
        p.ego.vx, p.ego.vy = 0.0, 1.0
        target = verified_target(p, x=radius*math.cos(angle),
                                 y=radius*math.sin(angle),
                                 vx=-5*math.sin(angle), vy=5*math.cos(angle))
        target.heading = angle + math.pi/2
        self.assert_profile(build_trajectory(p, d, settings()), d, settings())

    def test_upcoming_limit_is_met_when_vehicle_front_reaches_fractional_marker(self):
        p, d = fixtures.scene(8.0, 10.0)
        config = settings()
        distance = 25.37
        limit(p, distance + config.front_offset_m, 5.0)
        result = build_trajectory(p, d, config)
        self.assert_profile(result, d, config)
        at_sign = [point for point in result.points if abs(point.x-distance) < 1e-8]
        self.assertEqual(1, len(at_sign))
        self.assertAlmostEqual(5.0, at_sign[0].speed)
        self.assertEqual(8.0, result.points[0].speed)
        self.assertTrue(all(point.speed <= 5.0+1e-8 for point in result.points
                            if point.x >= distance))
        self.assertIn("upcoming speed limits applied", result.reason)

    def test_actual_map_producer_and_fixed_chain_share_upcoming_limit_geometry(self):
        native_sign = map_fixtures.sign(value="18", x=30.27)
        p = map_fixtures.MapObservationTests().perception(
            map_fixtures.map_api(signs=[native_sign]), x=0.0)
        self.assertTrue(p.speed_limit_observations[0]["applicable"])
        self.assertFalse(p.speed_limit_observations[0]["active"])
        env = {"NEVC_VEHICLE_FRONT_OFFSET_M": "3.9187",
               "NEVC_VEHICLE_HALF_WIDTH_M": "0.9"}
        with patch.dict(os.environ, env), \
                patch.object(decision_stub, "_ENGINE", decision_stub.DecisionEngine(
                    DecisionSettings(front_offset_m=3.9187))), \
                patch.object(control_stub, "_engine", ControlEngine(calibration())):
            decision = decision_stub.decide(p)
            trajectory = plan(p, decision)
            control = control_stub.compute_control(p, trajectory)
        self.assertTrue(trajectory.valid, trajectory.reason)
        self.assertTrue(control.valid, control.errors)
        self.assertTrue(all(pt.speed <= 5.0+1e-8 for pt in trajectory.points
                            if pt.x+3.9187 >= 30.27-1e-8))
        self.assertGreater(control.throttle, 0.0)

    def test_limit_beyond_horizon_still_requires_braking_in_current_profile(self):
        p, d = fixtures.scene(12.0, 15.0, length=400.0)
        config = settings(horizon=10.0)
        limit(p, 40.0 + config.front_offset_m, 2.0)
        result = build_trajectory(p, d, config)
        self.assert_profile(result, d, config)
        self.assertLess(result.points[-1].speed, result.points[0].speed)
        # Enough distance remains to reach 2 m/s using the configured deceleration.
        self.assertLessEqual(result.points[-1].speed**2 - 2.0**2,
                             2*config.deceleration*(40.0-result.points[-1].x)+1e-8)

    def test_unreachable_upcoming_limit_requests_braking_without_fabricated_profile(self):
        p, d = fixtures.scene(10.0, 10.0)
        config = settings()
        limit(p, 5.0 + config.front_offset_m, 2.0)
        result = build_trajectory(p, d, config)
        self.assertTrue(result.valid, result.reason)
        self.assertTrue(result.emergency_stop)
        self.assertEqual(0.0, result.target_speed)
        self.assertTrue(all(point.speed == 0 for point in result.points))

    def test_all_upcoming_limits_apply_regardless_of_catalog_order(self):
        config = settings()
        p, d = fixtures.scene(5.0, 10.0)
        limit(p, 30.27+config.front_offset_m, 8.0)
        limit(p, 18.62+config.front_offset_m, 4.0)
        a = build_trajectory(p, d, config)
        p.speed_limit_observations.reverse()
        b = build_trajectory(p, d, config)
        self.assert_profile(a, d, config)
        self.assertEqual([(pt.x, pt.speed) for pt in a.points],
                         [(pt.x, pt.speed) for pt in b.points])
        self.assertTrue(all(pt.speed <= 4.0+1e-8 for pt in a.points if pt.x >= 18.62))

    def test_unresolved_foreign_and_passed_signs_do_not_create_an_upcoming_limit(self):
        p, d = fixtures.scene(5.0, 8.0)
        base = build_trajectory(p, d, settings())
        limit(p, 20.0, 1.0, applicable=False)
        limit(p, 20.0, 1.0, source="unknown")
        limit(p, -2.0, 1.0)
        result = build_trajectory(p, d, settings())
        self.assert_profile(result, d, settings())
        self.assertEqual([pt.speed for pt in base.points], [pt.speed for pt in result.points])

    def test_claimed_applicable_but_bad_sign_data_is_not_silently_ignored(self):
        for change in ("catalog", "unit_value", "distance", "active", "geometry"):
            p, d = fixtures.scene(5.0, 8.0)
            item = limit(p, 30.0, 2.0)
            config = settings()
            if change == "catalog": p.traffic_signs_valid = False
            if change == "unit_value": item["speed_mps"] = float("nan")
            if change == "distance": item["along_distance"] = None
            if change == "active": item["active"] = True
            if change == "geometry": config = PlannerSettings()
            result = build_trajectory(p, d, config)
            self.assertFalse(result.valid, change)

    def test_limits_and_obstacles_preserve_input_frame_expiry_and_each_other(self):
        p, d = fixtures.scene(5.0, 8.0)
        verified_target(p, x=45.0, vx=5.0, vy=0.01)
        config = settings()
        limit(p, 20.5+config.front_offset_m, 3.0)
        original = copy.deepcopy(p)
        result = build_trajectory(p, d, config)
        self.assert_profile(result, d, config)
        self.assertEqual(p.frame_id, result.frame_id)
        self.assertEqual(d.valid_until, result.valid_until)
        self.assertTrue(result.stop_required)
        self.assertTrue(all(pt.speed <= 3.0+1e-8 for pt in result.points if pt.x >= 20.5))
        self.assertEqual(original.targets[0].vy, p.targets[0].vy)
        self.assertEqual(original.speed_limit_observations, p.speed_limit_observations)

    def test_real_fixed_chain_consumes_micro_lateral_follow_at_zero_tolerance(self):
        p = perception(speed=2.0)
        p.scene_id, p.ego.frame_id, p.ego.age_ms = 14, p.frame_id, 0
        target = add_target(p, longitudinal=40.0, speed=5.0)
        target.vy = 0.01
        env = {"NEVC_VEHICLE_FRONT_OFFSET_M": "3.9187",
               "NEVC_VEHICLE_HALF_WIDTH_M": "0.9",
               "NEVC_PLANNING_MOTION_TOLERANCE_MPS": "0"}
        with patch.dict(os.environ, env), \
                patch.object(decision_stub, "_ENGINE", decision_stub.DecisionEngine(
                    DecisionSettings(front_offset_m=3.9187))), \
                patch.object(control_stub, "_engine", ControlEngine(calibration())):
            decision = decision_stub.decide(p)
            trajectory = plan(p, decision)
            control = control_stub.compute_control(p, trajectory)
        self.assertEqual(DecisionMode.FOLLOW, decision.mode)
        self.assertTrue(trajectory.valid, trajectory.reason)
        self.assertTrue(control.valid, control.errors)
        self.assertGreater(control.throttle, 0.0)
        self.assertEqual(0.0, control.brake)

    def curve_scene(self, speed):
        radius = 30.
        p, d = fixtures.scene(speed, 8.)
        p.lane.center_line = [(radius*math.sin(i*.02), radius*(1-math.cos(i*.02)))
                              for i in range(151)]
        p.ego.heading = .01
        p.ego.vx, p.ego.vy = speed*math.cos(.01), speed*math.sin(.01)
        return p, d

    def test_small_curve_overspeed_recovers_by_bounded_braking_without_stop_latch(self):
        config = settings()
        p, d = self.curve_scene(6.8)
        result = build_trajectory(p, d, config)
        self.assert_profile(result, d, config)
        self.assertFalse(result.emergency_stop)
        self.assertIn('bounded curve-speed recovery', result.reason)
        self.assertLess(result.points[1].speed, p.ego.speed)
        cap = math.sqrt(1.5*30*math.sin(.01)/.01)
        self.assertTrue(all(point.speed <= cap+1e-6 for point in result.points
                            if point.relative_time >= config.curve_recovery_time_s))

    def test_curve_recovery_cannot_relax_large_overspeed_stop_or_published_limit(self):
        for change in ('large', 'stop', 'sign'):
            p, d = self.curve_scene(8 if change == 'large' else 6.8)
            if change == 'stop': d.stop_distance = 5
            if change == 'sign': limit(p, 3.9187 + .01, 6.)
            result = build_trajectory(p, d, settings())
            self.assertTrue(result.valid, result.reason)
            self.assertTrue(result.emergency_stop, change)

    def test_unknown_body_heading_cannot_use_verified_lane_lateral_exception(self):
        p, d = fixtures.scene(2, 5)
        target = verified_target(p, vx=5, vy=.01)
        target.heading = None
        self.assertFalse(build_trajectory(p, d, settings()).valid)


if __name__ == "__main__":
    unittest.main()
