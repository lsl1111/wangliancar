"""Synthetic, independent checks for the planning-local P02 validator.

Coordinates below are authored mathematical examples, not SimOne observations.
Expected collisions use explicit occupied coordinates or analytic motion; none
uses the validator's footprint, interpolation or collision helper to prove it.
Passing these checks does not establish actuator calibration or scene scores.
"""

import copy
import math
import time
import unittest

from core.interfaces import TrajectoryPoint
from members.planning.candidate_validation import (
    MotionLimits, ObstaclePrediction, ValidationBudget, VehicleGeometry,
    validate_candidate,
)


def point(x, y=0.0, speed=1.0, heading=0.0, at=0.0):
    return TrajectoryPoint(x, y, speed, heading, at)


def rectangle(xmin=-20.0, ymin=-20.0, xmax=40.0, ymax=20.0):
    return [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]


def vehicle(front=1.0, rear=0.5, half_width=0.3, wheelbase=0.8):
    return VehicleGeometry(front, rear, half_width, wheelbase)


def limits(**overrides):
    values = dict(max_speed_mps=8.0, max_acceleration_mps2=3.0,
                  max_deceleration_mps2=4.0,
                  max_lateral_acceleration_mps2=4.0,
                  max_front_steer_rad=0.7, max_steer_rate_rad_s=4.0,
                  heading_tolerance_rad=0.1, distance_tolerance_m=0.01)
    values.update(overrides)
    return MotionLimits(**values)


def obstacle(x, y, length=0.2, width=0.2, heading=0.0, vx=0.0,
             vy=0.0, horizon=30.0, position_uncertainty=0.0,
             velocity_uncertainty=0.0, identity="synthetic-target"):
    return ObstaclePrediction(identity, x, y, length, width, heading,
                              vx, vy, horizon, position_uncertainty,
                              velocity_uncertainty)


def budget(**overrides):
    values = dict(max_checks=10000, max_depth=20, min_interval_s=0.00001,
                  deadline_monotonic_s=time.monotonic() + 5.0)
    values.update(overrides)
    return ValidationBudget(**values)


def arc(radius=10.0, speed=1.0, end_angle=0.4, count=9, direction=1):
    """Rear axle follows an analytic circle; headings are body yaw.

    Reverse with positive steering has negative body yaw. Its travel tangent
    differs from body yaw by pi, while its published body yaw does not.
    """
    output = []
    for index in range(count):
        angle = end_angle * index / (count - 1)
        output.append(point(direction * radius * math.sin(angle),
                            radius * (1.0 - math.cos(angle)), speed,
                            direction * angle, radius * angle / speed))
    return output


class CandidateValidationTests(unittest.TestCase):
    def validate(self, points=None, direction=1, geometry=None, motion=None,
                 corridor=None, obstacles=None, work_budget=None):
        if points is None:
            points = [point(0.0), point(2.0, at=2.0)]
        return validate_candidate(
            points, direction, geometry or vehicle(), motion or limits(),
            rectangle() if corridor is None else corridor,
            [] if obstacles is None else obstacles,
            work_budget or budget())

    def assert_rejected(self, report, reason):
        self.assertNotEqual(report["status"], "safe", report)
        self.assertEqual(report["reason_code"], reason, report)
        self.assertIsNone(report["clearance_lower_bound_m"], report)

    def test_straight_safe_path_has_positive_whole_body_clearance(self):
        report = self.validate(corridor=rectangle(-2.0, -2.0, 5.0, 2.0))
        self.assertEqual(report["status"], "safe", report)
        self.assertGreater(report["clearance_lower_bound_m"], 0.0)
        # The nearest true road edge is y=+/-2, with the body at +/-0.3.
        self.assertLessEqual(report["clearance_lower_bound_m"], 1.7 + 1e-9)

    def test_front_overhang_hits_while_rear_axle_is_clear(self):
        # At x=0, the body reaches 3.9m: target x=3.5 is already occupied.
        report = self.validate(
            [point(0.0), point(0.2, at=0.2)],
            geometry=vehicle(3.9, 0.88, 0.9, 2.9),
            obstacles=[obstacle(3.5, 0.0)])
        self.assert_rejected(report, "OBSTACLE_COLLISION")

    def test_reverse_rear_overhang_hits_without_rear_axle_contact(self):
        # Rear axle ends at -0.2; body rear reaches -1.08, target is -0.9.
        report = self.validate(
            [point(0.0), point(-0.2, at=0.2)], direction=-1,
            geometry=vehicle(3.9, 0.88, 0.9, 2.9),
            obstacles=[obstacle(-0.9, 0.0, length=0.1)])
        self.assert_rejected(report, "OBSTACLE_COLLISION")

    def test_turning_front_corner_leaves_road_despite_axle_inside(self):
        points = arc(end_angle=math.pi / 4.0, count=17)
        self.assertLess(max(p.y for p in points), 3.0)
        # Final front-left corner y = 10*(1-cos(pi/4))+(3.9+.9)/sqrt(2)
        # > 6.3m, though every rear axle y is below 3m.
        report = self.validate(
            points, geometry=vehicle(3.9, 0.88, 0.9, 2.9),
            corridor=rectangle(-3.0, -2.0, 20.0, 4.0))
        self.assert_rejected(report, "CORRIDOR_COLLISION")

    def test_narrow_corridor_rejects_car_width(self):
        report = self.validate(corridor=rectangle(-2.0, -0.25, 5.0, 0.25))
        self.assert_rejected(report, "CORRIDOR_COLLISION")

    def test_thin_obstacle_between_clear_endpoint_footprints(self):
        # End footprints occupy [-.05,.05] and [9.95,10.05]; x=5 is between.
        report = self.validate(
            [point(0.0), point(10.0, at=10.0)],
            geometry=vehicle(0.05, 0.05, 0.05, 0.04),
            obstacles=[obstacle(5.0, 0.0, length=0.01, width=0.2)])
        self.assert_rejected(report, "OBSTACLE_COLLISION")

    def test_long_transverse_target_collides_despite_distant_center(self):
        # A 6m target at (1,2), yaw pi/2 occupies y in [-1,5].
        report = self.validate(
            obstacles=[obstacle(1.0, 2.0, 6.0, 0.3, math.pi / 2.0)])
        self.assert_rejected(report, "OBSTACLE_COLLISION")

    def test_long_parallel_target_is_clear_with_known_heading(self):
        # Same center, yaw zero: occupied y is [1.85,2.15], ego is +/-0.3.
        report = self.validate(obstacles=[obstacle(1.0, 2.0, 6.0, 0.3)])
        self.assertEqual(report["status"], "safe", report)
        self.assertGreater(report["clearance_lower_bound_m"], 0.0)

    def test_unknown_target_heading_uses_conservative_bounding_occupancy(self):
        # Radius hypot(3,.15)>3: (1,0) lies in the unknown-yaw occupancy.
        report = self.validate(
            obstacles=[obstacle(1.0, 2.0, 6.0, 0.3, heading=None)])
        self.assert_rejected(report, "OBSTACLE_COLLISION")

    def test_vehicle_dimensions_change_collision_result(self):
        target = obstacle(3.0, 0.0)
        short = self.validate([point(0.0), point(0.1, at=0.1)],
                              geometry=vehicle(), obstacles=[target])
        long = self.validate([point(0.0), point(0.1, at=0.1)],
                             geometry=vehicle(3.2, 0.5, 0.3, 2.0),
                             obstacles=[target])
        self.assertEqual(short["status"], "safe", short)
        self.assert_rejected(long, "OBSTACLE_COLLISION")

    def test_reverse_keeps_body_yaw_instead_of_adding_pi(self):
        correct = self.validate([point(0.0), point(-2.0, at=2.0)],
                                direction=-1)
        wrong = self.validate([point(0.0, heading=math.pi),
                               point(-2.0, heading=math.pi, at=2.0)],
                              direction=-1)
        self.assertEqual(correct["status"], "safe", correct)
        self.assert_rejected(wrong, "MOTION_LIMIT")

    def test_reverse_curved_body_yaw_is_compatible_with_same_steering(self):
        report = self.validate(arc(direction=-1), direction=-1)
        self.assertEqual(report["status"], "safe", report)
        # Analytic rear axle tangent points backward while body yaw is -0.4.
        self.assertAlmostEqual(arc(direction=-1)[-1].heading, -0.4)

    def test_heading_wrap_at_pi_preserves_straight_motion(self):
        report = self.validate([point(0.0, heading=math.pi),
                                point(-1.0, heading=-math.pi, at=1.0)])
        self.assertEqual(report["status"], "safe", report)

    def test_rotation_and_translation_preserve_safe_and_collision_results(self):
        angle, dx, dy = 0.73, 120.0, -80.0

        def transformed(x, y):
            return (dx + math.cos(angle) * x - math.sin(angle) * y,
                    dy + math.sin(angle) * x + math.cos(angle) * y)

        points = [point(0.0), point(2.0, at=2.0)]
        rotated = [point(*(transformed(p.x, p.y)), speed=p.speed,
                         heading=p.heading + angle, at=p.relative_time)
                   for p in points]
        road = rectangle(-3.0, -3.0, 6.0, 3.0)
        for target_y, expected in ((0.0, "unsafe"), (2.0, "safe")):
            original = self.validate(points, corridor=road,
                                     obstacles=[obstacle(1.5, target_y)])
            tx, ty = transformed(1.5, target_y)
            moved = self.validate(
                rotated, corridor=[transformed(x, y) for x, y in road],
                obstacles=[obstacle(tx, ty, heading=angle)])
            self.assertEqual(original["status"], expected, original)
            self.assertEqual(moved["status"], expected, moved)
            self.assertEqual(moved["reason_code"], original["reason_code"])

    def test_dynamic_target_crosses_between_clear_observation_instants(self):
        # At t=1 both rear axle and target center are (2,0); t=0 and t=2
        # target y=+2/-2, so an endpoint-only check misses the collision.
        report = self.validate(
            [point(0.0, speed=2.0), point(4.0, speed=2.0, at=2.0)],
            geometry=vehicle(0.4, 0.4, 0.2, 0.3),
            obstacles=[obstacle(2.0, 2.0, 0.2, 0.2, vx=0.0, vy=-2.0)])
        self.assert_rejected(report, "OBSTACLE_COLLISION")

    def test_accelerating_path_uses_physical_time_for_dynamic_crossing(self):
        # a=2 -> x(t)=t^2; target center is (1,2-2*t). They meet at t=1.
        # Uniform spatial interpolation would wrongly place ego at x=2 then.
        report = self.validate(
            [point(0.0, speed=0.0), point(4.0, speed=4.0, at=2.0)],
            geometry=vehicle(0.05, 0.05, 0.05, 0.04),
            obstacles=[obstacle(1.0, 2.0, 0.1, 0.1, vy=-2.0)])
        self.assert_rejected(report, "OBSTACLE_COLLISION")

    def test_prediction_coverage_must_include_complete_path_time(self):
        report = self.validate(obstacles=[obstacle(30.0, 10.0, horizon=1.0)])
        self.assert_rejected(report, "PREDICTION_HORIZON")

    def test_unknown_velocity_is_not_assumed_stationary(self):
        report = self.validate(obstacles=[obstacle(30.0, 10.0, vx=None)])
        self.assert_rejected(report, "INVALID_INPUT")

    def test_position_uncertainty_can_block_apparently_clear_target(self):
        known = self.validate(obstacles=[obstacle(1.0, 1.0)])
        uncertain = self.validate(
            obstacles=[obstacle(1.0, 1.0, position_uncertainty=1.0)])
        self.assertEqual(known["status"], "safe", known)
        self.assert_rejected(uncertain, "OBSTACLE_COLLISION")

    def test_diagonal_uncertainty_uses_euclidean_corner_distance(self):
        # Ego square [-.1,.1]^2; target square [.9,1.1]^2. Corner distance
        # sqrt(.8^2+.8^2)=1.131... exceeds .9m uncertainty, despite each
        # individual projection gap being only .8m. There is no collision.
        report = self.validate(
            [point(0.0, speed=0.0), point(0.0, speed=0.0, at=1.0)],
            geometry=vehicle(0.1, 0.1, 0.1, 0.08),
            obstacles=[obstacle(1.0, 1.0, position_uncertainty=0.9)])
        self.assertEqual(report["status"], "safe", report)
        self.assertGreater(report["clearance_lower_bound_m"], 0.0)
        self.assertLessEqual(report["clearance_lower_bound_m"],
                             math.sqrt(1.28) - 0.9 + 1e-9)

    def test_velocity_uncertainty_grows_during_prediction(self):
        # A stationary center at y=3 is clear; 2m/s unknown velocity error
        # permits an additional 4m displacement by the path end at t=2.
        report = self.validate(
            obstacles=[obstacle(1.0, 3.0, velocity_uncertainty=2.0)])
        self.assert_rejected(report, "OBSTACLE_COLLISION")

    def test_zero_speed_with_zero_distance_is_a_valid_hold(self):
        report = self.validate([point(0.0, speed=0.0),
                                point(0.0, speed=0.0, at=1.0)])
        self.assertEqual(report["status"], "safe", report)

    def test_nonzero_distance_cannot_have_two_zero_speeds(self):
        report = self.validate([point(0.0, speed=0.0),
                                point(1.0, speed=0.0, at=1.0)])
        self.assert_rejected(report, "INVALID_INPUT")
        self.assertIn("distance/speed/time inconsistent", report["details"]["message"])

    def test_speed_and_time_must_explain_travel_distance(self):
        # Mean speed 1m/s over 1s cannot cover 2m.
        report = self.validate([point(0.0), point(2.0, at=1.0)])
        self.assert_rejected(report, "INVALID_INPUT")
        self.assertIn("distance/speed/time inconsistent", report["details"]["message"])

    def test_longitudinal_acceleration_limit_without_mutating_measurement(self):
        points = [point(0.0, speed=1.0), point(2.0, speed=3.0, at=1.0)]
        original = copy.deepcopy([vars(p) for p in points])
        report = self.validate(points, motion=limits(max_acceleration_mps2=1.0))
        self.assert_rejected(report, "MOTION_LIMIT")
        self.assertEqual([vars(p) for p in points], original)

    def test_longitudinal_deceleration_limit(self):
        report = self.validate(
            [point(0.0, speed=3.0), point(2.0, speed=1.0, at=1.0)],
            motion=limits(max_deceleration_mps2=1.0))
        self.assert_rejected(report, "MOTION_LIMIT")

    def test_lateral_acceleration_limit_on_known_radius(self):
        # v^2/R=16/2=8m/s^2 exceeds the explicit 4m/s^2 limit.
        report = self.validate(arc(radius=2.0, speed=4.0, end_angle=0.2),
                               motion=limits())
        self.assert_rejected(report, "MOTION_LIMIT")

    def test_front_steering_limit_on_known_radius(self):
        # atan(wheelbase/R)=atan(.8/1) ~= .675rad > .4rad.
        report = self.validate(arc(radius=1.0, speed=0.5, end_angle=0.4),
                               motion=limits(max_front_steer_rad=0.4))
        self.assert_rejected(report, "MOTION_LIMIT")

    def test_steering_change_rate_limit(self):
        distance = math.sqrt(1.01)
        report = self.validate(
            [point(0.0, speed=distance),
             point(1.0, 0.1, speed=distance, heading=0.2, at=1.0),
             point(2.0, 0.0, speed=distance, heading=0.0, at=2.0)],
            motion=limits(max_steer_rate_rad_s=0.1))
        self.assert_rejected(report, "MOTION_LIMIT")

    def test_speed_limit_is_not_repaired_by_clamping_input(self):
        points = [point(0.0, speed=3.0), point(3.0, speed=3.0, at=1.0)]
        report = self.validate(points, motion=limits(max_speed_mps=2.0))
        self.assert_rejected(report, "MOTION_LIMIT")
        self.assertEqual([p.speed for p in points], [3.0, 3.0])

    def test_distance_tolerance_cannot_hide_actual_interpolated_overspeed(self):
        # Accepted residual .09m does not permit a physical 1.09m/s path
        # under a 1.05m/s cap: _pose traverses the full 1.09m in one second.
        report = self.validate(
            [point(0.0), point(1.09, at=1.0)],
            motion=limits(distance_tolerance_m=0.1, max_speed_mps=1.05))
        self.assert_rejected(report, "MOTION_LIMIT")
        self.assertEqual(report["constraint"], "speed", report)

    def test_distance_tolerance_cannot_hide_actual_interpolated_acceleration(self):
        # Stated mean speed 1.5m/s, actual distance 1.6m: spatial scaling
        # is 16/15 and actual acceleration is 16/15 > 1.02m/s^2.
        report = self.validate(
            [point(0.0, speed=1.0), point(1.6, speed=2.0, at=1.0)],
            motion=limits(distance_tolerance_m=0.2,
                          max_acceleration_mps2=1.02))
        self.assert_rejected(report, "MOTION_LIMIT")
        self.assertEqual(report["constraint"], "acceleration", report)

    def test_distance_tolerance_cannot_hide_actual_interpolated_deceleration(self):
        report = self.validate(
            [point(0.0, speed=2.0), point(1.6, speed=1.0, at=1.0)],
            motion=limits(distance_tolerance_m=0.2,
                          max_deceleration_mps2=1.02))
        self.assert_rejected(report, "MOTION_LIMIT")
        self.assertEqual(report["constraint"], "deceleration", report)

    def test_distance_tolerance_checks_squared_actual_speed_for_lateral_limit(self):
        # Chord tangent .1rad matches mean body yaw .1rad. For the explicit
        # segment bicycle model curvature=.2/1.09. Actual speed 1.09 gives
        # lateral acceleration .218m/s^2; nominal speed 1 gives only .1835.
        report = self.validate(
            [point(0.0), point(1.09 * math.cos(0.1),
                               1.09 * math.sin(0.1), heading=0.2, at=1.0)],
            motion=limits(distance_tolerance_m=0.1,
                          max_lateral_acceleration_mps2=0.2))
        self.assert_rejected(report, "MOTION_LIMIT")
        self.assertEqual(report["constraint"], "lateral_acceleration", report)

    def test_constant_reported_heading_does_not_hide_geometric_turning(self):
        # Equal 1m legs turn .6rad: their circumcircle curvature is
        # 2*sin(.3)=.591.../m and atan(.8*.591)>.4rad. Allowing a loose
        # sample-yaw tolerance must not erase that map/path geometry fact.
        report = self.validate(
            [point(-1.0), point(0.0, at=1.0),
             point(math.cos(0.6), math.sin(0.6), at=2.0)],
            motion=limits(heading_tolerance_rad=1.0,
                          max_front_steer_rad=0.1))
        self.assert_rejected(report, "MOTION_LIMIT")
        self.assertIn("steer", report["constraint"], report)

    def test_stationary_samples_cannot_hide_geometric_steering_limit(self):
        # Curvature = 2*.09/(1+.09^2), requiring atan(.8*curvature)
        # = .14188rad. Zero-speed dwell samples cannot make .05rad feasible.
        expected = math.atan(0.8 * 0.18 / 1.0081)
        leg_time = 2.0 * math.hypot(1.0, 0.09)
        for direction in (1, -1):
            for sign in (1, -1):
                for count in (1, 3):
                    with self.subTest(direction=direction, sign=sign, count=count):
                        points = [point(-direction, sign * 0.09),
                                  point(0.0, speed=0.0, at=leg_time)]
                        points.extend(point(0.0, speed=0.0, at=leg_time + n)
                                      for n in range(1, count + 1))
                        points.append(point(direction, sign * 0.09,
                                            at=2.0 * leg_time + count))
                        report = self.validate(points, direction=direction,
                            motion=limits(max_front_steer_rad=0.05))
                        self.assert_rejected(report, "MOTION_LIMIT")
                        self.assertEqual(report["constraint"], "steering", report)
                        self.assertAlmostEqual(report["details"]["observed"], expected)
                        self.assertEqual(report["details"]["point"], 1)

    def test_stationary_sample_cannot_hide_adjacent_moving_lateral_limit(self):
        # Moving legs reach 2m/s, so v^2*k = 4*.18/1.0081 > .5m/s^2,
        # even though all copies of the turning vertex have zero speed.
        leg_time = math.hypot(1.0, 0.09)
        for direction in (1, -1):
            with self.subTest(direction=direction):
                report = self.validate(
                    [point(-direction, 0.09, speed=2.0),
                     point(0.0, speed=0.0, at=leg_time),
                     point(0.0, speed=0.0, at=leg_time + 1.0),
                     point(direction, 0.09, speed=2.0, at=2.0 * leg_time + 1.0)],
                    direction=direction,
                    motion=limits(max_lateral_acceleration_mps2=0.5))
                self.assert_rejected(report, "MOTION_LIMIT")
                self.assertEqual(report["constraint"], "lateral_acceleration", report)
                self.assertAlmostEqual(report["details"]["observed"], 4.0 * 0.18 / 1.0081)

    def test_feasible_turn_with_hold_preserves_original_time_and_samples(self):
        leg_time = 2.0 * math.hypot(1.0, 0.09)
        for direction in (1, -1):
            with self.subTest(direction=direction):
                points = [point(-direction, 0.09),
                          point(0.0, speed=0.0, at=leg_time),
                          point(0.0, speed=0.0, at=leg_time + 1.0),
                          point(direction, 0.09, at=2.0 * leg_time + 1.0)]
                original = copy.deepcopy([vars(p) for p in points])
                report = self.validate(points, direction=direction,
                                       motion=limits(max_front_steer_rad=0.2))
                self.assertEqual(report["status"], "safe", report)
                self.assertEqual(report["details"]["segment_count"], 3)
                self.assertAlmostEqual(report["details"]["validated_horizon_s"],
                                       2.0 * leg_time + 1.0)
                self.assertEqual([vars(p) for p in points], original)

    def test_straight_path_with_multiple_holds_is_safe_in_both_directions(self):
        for direction in (1, -1):
            with self.subTest(direction=direction):
                report = self.validate(
                    [point(-2.0 * direction),
                     point(-direction, speed=0.0, at=2.0),
                     point(-direction, speed=0.0, at=3.0),
                     point(-direction, speed=0.0, at=4.0),
                     point(0.0, at=6.0)], direction=direction,
                    motion=limits(max_front_steer_rad=0.05))
                self.assertEqual(report["status"], "safe", report)
                self.assertEqual(report["details"]["segment_count"], 4)
                self.assertEqual(report["details"]["validated_horizon_s"], 6.0)

    def test_leading_and_trailing_holds_do_not_create_geometric_turns(self):
        for direction in (1, -1):
            with self.subTest(direction=direction):
                report = self.validate(
                    [point(-direction, speed=0.0),
                     point(-direction, speed=0.0, at=1.0),
                     point(0.0, at=3.0),
                     point(direction, speed=0.0, at=5.0),
                     point(direction, speed=0.0, at=6.0),
                     point(direction, speed=0.0, at=7.0)], direction=direction)
                self.assertEqual(report["status"], "safe", report)
                self.assertEqual(report["details"]["validated_horizon_s"], 7.0)

    def test_dynamic_collision_during_hold_retains_time_occupancy(self):
        # The target crosses (0,0) at t=4. A four-second stop occupies it
        # then; the same two moving legs without the stop finish elsewhere.
        for direction in (1, -1):
            with self.subTest(direction=direction):
                geometry = vehicle(0.2, 0.2, 0.1, 0.15)
                target = obstacle(0.0, 4.0, 0.1, 0.1, vy=-1.0)
                moving = [point(-direction), point(0.0, speed=0.0, at=2.0),
                          point(direction, at=4.0)]
                clear = self.validate(moving, direction=direction, geometry=geometry,
                                      obstacles=[target])
                self.assertEqual(clear["status"], "safe", clear)
                stopped = moving[:2] + [point(0.0, speed=0.0, at=6.0),
                                        point(direction, at=8.0)]
                report = self.validate(stopped, direction=direction, geometry=geometry,
                                       obstacles=[target])
                self.assert_rejected(report, "OBSTACLE_COLLISION")
                self.assertGreater(report["details"]["relative_time_s"], 2.0)
                self.assertLess(report["details"]["relative_time_s"], 6.0)

    def test_hold_samples_still_require_zero_speed_and_no_body_rotation(self):
        for speed, heading in ((0.01, 0.0), (0.0, 0.01)):
            with self.subTest(speed=speed, heading=heading):
                report = self.validate(
                    [point(-1.0), point(0.0, speed=0.0, at=2.0),
                     point(0.0, speed=speed, heading=heading, at=3.0),
                     point(1.0, at=5.0)])
                self.assert_rejected(report, "INVALID_INPUT")
                self.assertIn("stationary segment", report["details"]["message"])

    def test_geometric_steering_rate_across_hold_uses_original_elapsed_time(self):
        travel = math.hypot(1.0, 0.09)
        leg_time = 2.0 * travel
        steer_change = 2.0 * math.atan(0.8 * 0.18 / 1.0081)
        for direction in (1, -1):
            for hold, expected in ((0.1, "unsafe"), (1.0, "safe")):
                with self.subTest(direction=direction, hold=hold):
                    report = self.validate(
                        [point(-direction, 0.09),
                         point(0.0, speed=0.0, at=leg_time),
                         point(0.0, speed=0.0, at=leg_time + hold),
                         point(direction, 0.09, at=2.0 * leg_time + hold),
                         point(2.0 * direction, at=2.0 * leg_time + hold + travel)],
                        direction=direction, motion=limits(max_steer_rate_rad_s=0.1))
                    self.assertEqual(report["status"], expected, report)
                    if expected == "unsafe":
                        self.assert_rejected(report, "MOTION_LIMIT")
                        self.assertEqual(report["constraint"], "steering_rate", report)
                        self.assertAlmostEqual(report["details"]["observed"],
                                               steer_change / (leg_time + hold))

    def test_non_increasing_time_is_invalid(self):
        report = self.validate([point(0.0), point(1.0, at=0.0)])
        self.assert_rejected(report, "INVALID_INPUT")

    def test_nonzero_start_time_does_not_silently_shift_prediction_clock(self):
        report = self.validate([point(0.0, at=2.0), point(1.0, at=3.0)])
        self.assert_rejected(report, "INVALID_INPUT")

    def test_obstacle_order_preserves_collision_semantics(self):
        hit = obstacle(1.5, 0.0, identity="hit")
        clear = obstacle(5.0, 5.0, identity="clear")
        for targets in ([hit, clear], [clear, hit]):
            report = self.validate(obstacles=targets)
            self.assert_rejected(report, "OBSTACLE_COLLISION")

    def test_mixed_direction_path_is_not_a_single_authorized_segment(self):
        report = self.validate([point(0.0), point(1.0, at=1.0),
                                point(0.0, at=2.0)])
        self.assert_rejected(report, "MOTION_LIMIT")

    def test_missing_or_concave_corridor_is_not_unbounded_permission(self):
        for road in ([], [(0.0, 0.0), (5.0, 0.0), (2.0, 1.0),
                          (5.0, 5.0), (0.0, 5.0)]):
            report = self.validate(corridor=road)
            self.assert_rejected(report, "INVALID_INPUT")

    def test_nonfinite_geometry_and_unknown_direction_are_rejected(self):
        for values in (dict(geometry=vehicle(front=float("nan"))),
                       dict(direction=0), dict(direction=True)):
            report = self.validate(**values)
            self.assert_rejected(report, "INVALID_INPUT")

    def test_check_budget_exhaustion_never_claims_safe(self):
        report = self.validate(work_budget=budget(max_checks=1))
        self.assert_rejected(report, "BUDGET_EXHAUSTED")
        self.assertEqual(report["status"], "inconclusive", report)

    def test_elapsed_deadline_never_claims_safe(self):
        report = self.validate(work_budget=budget(
            deadline_monotonic_s=time.monotonic() - 1.0))
        self.assert_rejected(report, "BUDGET_EXHAUSTED")
        self.assertEqual(report["status"], "inconclusive", report)


if __name__ == "__main__":
    unittest.main()
