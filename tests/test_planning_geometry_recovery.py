"""Supported recovery through real planning/control; synthetic maps and plant."""
import math
import unittest

from core.interfaces import DecisionMode, DecisionTarget, ControlOut, Trajectory
from core.validation import validate_output
from members.control.controller import ControlEngine
from members.decision.engine import DecisionEngine
from members.decision.settings import DecisionSettings
from members.planning.lane_planner import PlannerSettings, build_trajectory
from tests.control_benchmark import vehicle
from tests.test_control_maneuvers import BidirectionalPlant
from tests.test_decision import perception
from tests import test_planning as fixtures
from tests.test_planning_constraints import limit


def capability(**overrides):
    values = dict(front_offset_m=3.9187, rear_offset_m=.88, half_width_m=.9,
                  wheelbase_m=2.7, front_steer_max_rad=.5)
    values.update(overrides)
    return PlannerSettings(**values)


def circle(speed=5, step=.02, radius=30, theta=.36):
    p, d = fixtures.scene(speed, 8)
    p.lane.center_line = [(radius*math.sin(i*step), radius*(1-math.cos(i*step)))
                          for i in range(int(3.6/step)+1)]
    p.lane.lane_width_valid, p.lane.lane_width = True, 3.5
    p.ego.x, p.ego.y = radius*math.sin(theta), radius*(1-math.cos(theta))
    p.ego.heading = theta
    p.ego.vx, p.ego.vy = speed*math.cos(theta), speed*math.sin(theta)
    return p, d


class PlanningGeometryRecoveryTests(unittest.TestCase):
    def assert_profile(self, t, d, settings):
        fixtures.PlanningTests.assert_profile(self, t, d, settings)
        self.assertFalse(t.emergency_stop, t.reason)

    def test_offset_just_above_old_gate_generates_gradual_feasible_return(self):
        for offset in (-.6, -.51, .51, .6):
            p, d = fixtures.scene(3, 5)
            p.ego.x, p.ego.y = 10, offset
            p.lane.lane_width_valid, p.lane.lane_width = True, 3.5
            config = capability()
            t = build_trajectory(p, d, config)
            self.assert_profile(t, d, config)
            self.assertAlmostEqual(p.ego.x, t.points[0].x)
            self.assertAlmostEqual(offset, t.points[0].y)
            self.assertAlmostEqual(p.ego.heading, t.points[0].heading)
            self.assertAlmostEqual(0, t.points[-1].y)
            self.assertTrue(all(abs(point.y) <= abs(offset)+1e-6 for point in t.points))
            for a, b in zip(t.points, t.points[1:]):
                self.assertLess(abs(b.y-a.y), .1)

    def test_recovery_requires_body_steering_and_verified_corridor(self):
        for change in ('rear', 'steering', 'width', 'outside', 'opposed'):
            p, d = fixtures.scene(3, 5)
            p.ego.x, p.ego.y = 10, .6
            p.lane.lane_width_valid, p.lane.lane_width = True, 3.5
            config = capability()
            if change == 'rear': config.rear_offset_m = None
            if change == 'steering': config.front_steer_max_rad = None
            if change == 'width': p.lane.lane_width_valid = False
            if change == 'outside': p.ego.y = 1.0
            if change == 'opposed': p.ego.heading = 1.0
            t = build_trajectory(p, d, config)
            self.assertFalse(t.valid, (change, t.reason))

    def test_sparse_and_dense_samples_of_same_circle_are_supported(self):
        for step in (.02, .34, .36):
            p, d = circle(step=step, theta=step*2)
            config = capability()
            t = build_trajectory(p, d, config)
            self.assert_profile(t, d, config)
            if step > config.max_corner_angle:
                self.assertIn('sparse curve reconstructed', t.reason)
                self.assertTrue(all(abs(math.hypot(point.x, point.y-30)-30)<.02
                                    for point in t.points))

    def test_sharp_kink_and_unsupported_successor_remain_rejected(self):
        p, d = fixtures.scene(3, 5)
        p.ego.x = 10
        p.lane.lane_width_valid, p.lane.lane_width = True, 3.5
        p.lane.center_line = [(0, 0), (30, 0), (30, 40), (30, 100)]
        self.assertFalse(build_trajectory(p, d, capability()).valid)
        p, d = circle(step=.36, theta=.72)
        self.assertFalse(build_trajectory(p, d, PlannerSettings()).valid)
        p.lane.center_line = p.lane.center_line[:4]
        p.lane.forward_reference_valid = True
        p.lane.forward_reference = circle(step=.36)[0].lane.center_line
        p.lane.forward_lane_ids = [p.lane.lane_id, 'next']
        self.assertFalse(build_trajectory(p, d, capability()).valid)

    def test_comfort_overspeed_uses_geometry_and_braking_distance(self):
        for speed in (7.1, 7.2):
            p, d = circle(speed)
            config = capability()
            t = build_trajectory(p, d, config)
            self.assert_profile(t, d, config)
            self.assertEqual(speed, t.points[0].speed)
            self.assertIn('minimum-risk curve-speed recovery', t.reason)
            self.assertIn('physical lateral capability unavailable', t.reason)
            self.assertLess(t.points[1].speed, speed)
            cap = math.sqrt(config.lateral_acceleration*30*math.sin(.01)/.01)
            distance = (speed**2-cap**2)/(2*config.deceleration)
            progress = 0
            for a, b in zip(t.points, t.points[1:]):
                progress += math.hypot(b.x-a.x, b.y-a.y)
                if progress >= distance+1e-5:
                    self.assertLessEqual(b.speed, cap+1e-5)

    def test_recovery_preserves_actual_lateral_stop_and_published_limit(self):
        for change in ('physical', 'stop', 'sign', 'tighter'):
            p, d = circle(7.2)
            config = capability()
            if change == 'physical': config.max_lateral_acceleration_mps2 = 1.6
            if change == 'stop': d.stop_distance = 2
            if change == 'sign': limit(p, config.front_offset_m+.5, 6)
            if change == 'tighter':
                # A much tighter future bend cannot borrow the current cap.
                p.lane.center_line[21] = (p.lane.center_line[21][0],
                                         p.lane.center_line[21][1]+.05)
            t = build_trajectory(p, d, config)
            self.assertTrue(t.valid, (change, t.reason))
            self.assertTrue(t.emergency_stop, (change, t.reason))

    def test_real_controller_executes_offset_recovery_with_actuator_lag(self):
        for lag in (.1, .4):
            plant, now = BidirectionalPlant(x=10, y=.6, speed=3, lag=lag), [0]
            calibration = vehicle()
            controller = ControlEngine(calibration, clock=lambda: now[0])
            config = capability()
            decider = DecisionEngine(DecisionSettings(front_offset_m=config.front_offset_m,
                                                    half_width_m=config.half_width_m))
            for frame in range(1, 121):
                p = perception(speed=plant.speed, frame_id=frame, ttl=30)
                p.case_id, p.task_id, p.scene_id = 'offset-return', 'offset-return', 25
                p.ego.frame_id, p.ego.age_ms = frame, 0
                p.ego.x, p.ego.y, p.ego.heading = plant.x, plant.y, plant.heading
                p.ego.vx, p.ego.vy = plant.speed*math.cos(plant.heading), plant.speed*math.sin(plant.heading)
                p.ego.yaw_rate = plant.yaw_rate
                d = decider.run(p)
                t = build_trajectory(p, d, config)
                c = controller.compute(p, t)
                validate_output(d, DecisionTarget, p)
                self.assert_profile(t, d, config)
                validate_output(c, ControlOut, t)
                self.assertNotEqual(DecisionMode.EMERGENCY_BRAKE, d.mode, d.reason)
                self.assertLess(abs(plant.y), .61)
                self.assertLess(abs(c.steering), 1)
                plant.step(c, .05, calibration)
                now[0] += .05
            self.assertLess(abs(plant.y), .08)
            self.assertGreater(plant.x, 25)

    def test_curve_recovery_is_consumed_as_braking_and_remains_trackable(self):
        for lag in (.1, .4):
            p, _ = circle(7.2)
            plant = BidirectionalPlant(x=p.ego.x, y=p.ego.y, heading=p.ego.heading,
                                       speed=p.ego.speed, lag=lag)
            calibration, now = vehicle(), [0]
            plant.steering = math.atan(calibration.wheelbase_m/30)/calibration.front_steer_max_rad
            plant.yaw_rate = plant.speed/30
            controller = ControlEngine(calibration, clock=lambda: now[0])
            config = capability()
            decider = DecisionEngine(DecisionSettings(front_offset_m=config.front_offset_m,
                                                    half_width_m=config.half_width_m))
            peak_error = 0
            for frame in range(1, 101):
                p = perception(speed=plant.speed, frame_id=frame, ttl=30)
                p.case_id, p.task_id, p.scene_id = 'curve-recovery', 'curve-recovery', 25
                p.ego.frame_id, p.ego.age_ms = frame, 0
                p.ego.x, p.ego.y, p.ego.heading = plant.x, plant.y, plant.heading
                p.ego.vx, p.ego.vy = plant.speed*math.cos(plant.heading), plant.speed*math.sin(plant.heading)
                p.ego.yaw_rate = plant.yaw_rate
                p.lane.center_line = circle()[0].lane.center_line
                d = decider.run(p)
                t = build_trajectory(p, d, config)
                c = controller.compute(p, t)
                self.assert_profile(t, d, config)
                validate_output(c, ControlOut, t)
                self.assertNotEqual(DecisionMode.EMERGENCY_BRAKE, d.mode, d.reason)
                self.assertFalse(c.throttle > 0 and c.brake > 0)
                if frame == 1:
                    self.assertEqual(0, c.throttle)
                    self.assertGreater(c.brake, 0)
                peak_error = max(peak_error, abs(math.hypot(plant.x, plant.y-30)-30))
                plant.step(c, .05, calibration)
                now[0] += .05
            self.assertLess(peak_error, .3)
            self.assertLess(plant.speed, 6.9)

    def test_hard_feasible_approach_does_not_accelerate_after_comfort_margin_is_used(self):
        p, d = fixtures.scene(12, 15, length=400)
        config = capability(horizon=10)
        limit(p, 40+config.front_offset_m, 2)
        t = build_trajectory(p, d, config)
        self.assert_profile(t, d, config)
        self.assertIn('required braking begins', t.reason)
        self.assertLess(t.points[1].speed, p.ego.speed)
        first, second = t.points[:2]
        deceleration = (first.speed**2-second.speed**2)/(2*(second.x-first.x))
        self.assertAlmostEqual((12**2-2**2)/(2*40), deceleration)

    def test_full_chain_brakes_before_future_curve_and_enters_without_emergency(self):
        # The historical captured failures were future-curvature constraints;
        # changing only an initial overspeed window cannot prevent those.
        road = [(float(x), 0.) for x in range(-20, 41)]
        road.extend((40+12*math.sin(i*.02), 12*(1-math.cos(i*.02)))
                    for i in range(1, 201))
        for lag in (.1, .4):
            plant, now = BidirectionalPlant(speed=8, lag=lag), [0]
            calibration, config = vehicle(), capability()
            controller = ControlEngine(calibration, clock=lambda: now[0])
            decider = DecisionEngine(DecisionSettings(front_offset_m=config.front_offset_m,
                                                    half_width_m=config.half_width_m))
            braking_before_entry = entered = False
            for frame in range(1, 161):
                p = perception(speed=plant.speed, frame_id=frame, ttl=30)
                p.case_id, p.task_id, p.scene_id = 'future-curve', 'future-curve', 25
                p.ego.frame_id, p.ego.age_ms = frame, 0
                p.ego.x, p.ego.y, p.ego.heading = plant.x, plant.y, plant.heading
                p.ego.vx, p.ego.vy = plant.speed*math.cos(plant.heading), plant.speed*math.sin(plant.heading)
                p.ego.yaw_rate = plant.yaw_rate
                p.lane.center_line = road
                d = decider.run(p)
                t = build_trajectory(p, d, config)
                c = controller.compute(p, t)
                self.assert_profile(t, d, config)
                validate_output(c, ControlOut, t)
                self.assertNotEqual(DecisionMode.EMERGENCY_BRAKE, d.mode, d.reason)
                if plant.x < 35 and c.brake > 0:
                    braking_before_entry = True
                if plant.x >= 40:
                    self.assertLess(plant.speed, 4.45)
                    entered = True
                plant.step(c, .05, calibration)
                now[0] += .05
            self.assertTrue(braking_before_entry)
            self.assertTrue(entered)


if __name__ == '__main__':
    unittest.main()
