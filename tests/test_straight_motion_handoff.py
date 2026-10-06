"""Pose continuity and physical collision relevance through the real chain.

Synthetic data and a lagged bicycle only; no SimOne connection or scenario IDs.
"""
import math
import unittest
from types import SimpleNamespace

from core.interfaces import DecisionMode
from core.geometry import world_to_ego, calculate_ttc
from core.safety_supervisor import SafetySupervisor
from members.control.controller import ControlEngine
from members.decision.engine import DecisionEngine
from members.decision.settings import DecisionSettings
from members.planning.lane_planner import PlannerSettings, build_trajectory
from tests.control_benchmark import vehicle
from tests.test_control_maneuvers import BidirectionalPlant
from tests.test_decision import perception, add_target


class StraightMotionHandoffTests(unittest.TestCase):
    def setUp(self):
        self.ds = DecisionSettings(front_offset_m=3.9187, half_width_m=.9,
                                   cruise_speed=4.)
        self.ps = PlannerSettings(front_offset_m=3.9187, half_width_m=.9,
                                  rear_offset_m=.88, wheelbase_m=2.9187,
                                  front_steer_max_rad=.5)
        self.calibration = vehicle()
        self.calibration.wheelbase_m = self.ps.wheelbase_m

    def sample(self, x=2., y=.12, heading=0., speed=0., frame=1):
        p = perception(speed=speed, frame_id=frame, ttl=30)
        p.ego.x, p.ego.y, p.ego.heading = x, y, heading
        p.ego.frame_id, p.ego.age_ms = frame, 0
        p.ego.vx, p.ego.vy = speed*math.cos(heading), speed*math.sin(heading)
        p.lane.center_line = [(float(i), 0.) for i in range(-5, 105)]
        # Explicit boundaries are measured inputs, not fabricated by the planner.
        p.lane.left_boundary = [(-5., 1.75), (104., 1.75)]
        p.lane.right_boundary = [(-5., -1.75), (104., -1.75)]
        return p

    def safety(self, p):
        d = DecisionEngine(self.ds).run(p)
        t = build_trajectory(p, d, self.ps)
        return d, t, SafetySupervisor(SimpleNamespace(), planning_settings=self.ps,
                                     decision_settings=self.ds).evaluate(p, d, t)

    def roadside(self, y=1.33, width=.325, target_type=15):
        p = self.sample(y=0., speed=5.)
        target = add_target(p, longitudinal=8., speed=0., length=.325)
        target.x, target.y, target.width = p.ego.x+8., y, width
        target.type, target.heading = target_type, math.pi
        target.lane_id, target.same_lane_valid, target.same_lane = '', False, False
        target.lateral_band_match, target.ttc = True, 1.6
        # After leaving a signal's scope, no lamp remains in applicable candidates.
        p.traffic.association_valid, p.traffic.candidates = True, []
        p.source_status['traffic'] = {'association_valid': True, 'usable': True}
        return p

    def test_small_offsets_start_at_measured_pose_and_heading(self):
        for offset in (-.3, -.12, .12, .3):
            for speed in (0., 2., 8.):
                p = self.sample(y=offset, speed=speed)
                d = DecisionEngine(self.ds).run(p)
                t = build_trajectory(p, d, self.ps)
                self.assertTrue(t.valid, t.reason)
                self.assertFalse(t.emergency_stop, t.reason)
                first = t.points[0]
                self.assertAlmostEqual(p.ego.x, first.x, places=6)
                self.assertAlmostEqual(p.ego.y, first.y, places=6)
                self.assertAlmostEqual(p.ego.heading, first.heading, places=6)
                c = ControlEngine(self.calibration).compute(p, t)
                self.assertTrue(c.valid, c.errors)
                if speed == 0:
                    self.assertLess(abs(c.diagnostics['front_angle_rad']), math.radians(5))
                else:
                    # Initial moving inputs must first engage the requested
                    # direction; that legitimate brake-only stage has no carrot.
                    self.assertIn(c.diagnostics['state'], ('TRACK', 'GEAR_STOP'))

    def test_rotated_road_has_the_same_pose_continuity(self):
        p = self.sample()
        angle = math.radians(67)
        def rotate(x, y):
            return x*math.cos(angle)-y*math.sin(angle), x*math.sin(angle)+y*math.cos(angle)
        p.ego.x, p.ego.y = rotate(p.ego.x, p.ego.y)
        p.ego.heading = angle
        for field in ('center_line', 'left_boundary', 'right_boundary'):
            setattr(p.lane, field, [rotate(*point) for point in getattr(p.lane, field)])
        d = DecisionEngine(self.ds).run(p)
        t = build_trajectory(p, d, self.ps)
        self.assertTrue(t.valid, t.reason)
        self.assertAlmostEqual(p.ego.x, t.points[0].x)
        self.assertAlmostEqual(p.ego.y, t.points[0].y)
        self.assertAlmostEqual(angle, t.points[0].heading)

    def test_small_offset_launch_converges_without_large_countersteering(self):
        for sign in (-1, 1):
            now = [0.]
            plant = BidirectionalPlant(x=2., y=sign*.12, lag=.3)
            calibration = self.calibration
            control = ControlEngine(calibration, clock=lambda: now[0])
            decision = DecisionEngine(self.ds)
            peak = 0.
            for frame in range(1, 401):
                p = self.sample(plant.x, plant.y, plant.heading, plant.speed, frame)
                p.ego.yaw_rate = plant.yaw_rate
                d = decision.run(p)
                t = build_trajectory(p, d, self.ps)
                self.assertTrue(t.valid, (frame, t.reason))
                c = control.compute(p, t)
                self.assertTrue(c.valid, (frame, c.errors))
                peak = max(peak, abs(c.diagnostics.get('front_angle_rad', 0.)))
                self.assertLessEqual(abs(plant.y), .125)
                plant.step(c, .05, calibration)
                now[0] += .05
            self.assertGreater(plant.x, 30.)
            self.assertLess(abs(plant.y), .025)
            self.assertLess(peak, math.radians(5))

    def test_clear_roadside_footprint_does_not_emergency_stop_for_small_ttc(self):
        for target_type in (15, 6, 0):
            for side in (-1, 1):
                p = self.roadside(y=side*1.33, target_type=target_type)
                d, t, safety = self.safety(p)
                self.assertEqual(DecisionMode.KEEP_LANE, d.mode, d.reason)
                self.assertTrue(t.valid, t.reason)
                self.assertFalse(t.stop_required)
                self.assertEqual('normal', safety.mode, safety.reason)

    def test_actual_footprint_intrusion_remains_an_independent_emergency(self):
        for y, width, vy in ((.8, .325, 0.), (1.33, 1.8, 0.), (1.33, .325, -1.)):
            p = self.roadside(y=y, width=width)
            p.targets[0].vy = vy
            self.assertTrue(SafetySupervisor._imminent_collision(p, self.ps, self.ds))

    def test_missing_geometry_or_dimensions_keeps_conservative_ttc_fallback(self):
        for field in ('reference', 'width', 'dimensions'):
            p = self.roadside()
            if field == 'reference':
                p.lane.center_line = []
            elif field == 'width':
                p.lane.lane_width_valid = False
            else:
                p.targets[0].width = None
            self.assertTrue(SafetySupervisor._imminent_collision(p, self.ps, self.ds))

    def test_measured_ego_offset_and_heading_are_included_in_collision_corridor(self):
        for y, heading in ((.6, 0.), (0., .15)):
            p = self.roadside()
            p.ego.y, p.ego.heading = y, heading
            self.assertTrue(SafetySupervisor._imminent_collision(p, self.ps, self.ds))

    def test_unassociated_object_on_bend_keeps_conservative_ttc_fallback(self):
        p = self.roadside()
        p.lane.center_line = [(30.*math.sin(i*.02), 30.*(1.-math.cos(i*.02)))
                              for i in range(151)]
        self.assertTrue(SafetySupervisor._imminent_collision(p, self.ps, self.ds))

    def test_successor_without_current_lane_body_coverage_does_not_force_new_blend(self):
        p = self.sample(x=12., y=.00012)
        p.lane.center_line = [(float(i), 0.) for i in range(-5, 16)]
        p.lane.left_boundary = [(-5., 1.75), (15., 1.75)]
        p.lane.right_boundary = [(-5., -1.75), (15., -1.75)]
        p.lane.forward_reference_valid = True
        p.lane.forward_reference = p.lane.center_line + [(float(i), 0.) for i in range(16, 105)]
        p.lane.forward_lane_ids = [p.lane.lane_id, 'successor']
        p.lane.forward_reference_status = 'map_end'
        d, t, safety = self.safety(p)
        self.assertTrue(t.valid, t.reason)
        self.assertFalse(t.emergency_stop)
        self.assertEqual('normal', safety.mode, safety.reason)
        self.assertNotIn('lateral recovery', t.reason)

    def test_full_chain_passes_roadside_fixture_without_stop_restart_cycles(self):
        plant = BidirectionalPlant(x=2., y=.12)
        now = [0.]
        control = ControlEngine(self.calibration, clock=lambda: now[0])
        decision = DecisionEngine(self.ds)
        supervisor = SafetySupervisor(SimpleNamespace(), planning_settings=self.ps,
                                      decision_settings=self.ds)
        for frame in range(1, 401):
            p = self.sample(plant.x, plant.y, plant.heading, plant.speed, frame)
            p.ego.yaw_rate = plant.yaw_rate
            target = add_target(p, longitudinal=25.-plant.x, speed=0., length=.325)
            target.x, target.y, target.width, target.type = 25., 1.33, .325, 15
            target.heading, target.lane_id = math.pi, ''
            target.same_lane, target.same_lane_valid = False, False
            target.longitudinal_distance, target.lateral_distance = world_to_ego(
                plant.x, plant.y, plant.heading, target.x, target.y)
            target.lateral_band_match = abs(target.lateral_distance) < 2.
            target.ttc, target.relative_speed = calculate_ttc(target.longitudinal_distance,
                plant.speed, target.vx, target.vy, plant.heading)
            d = decision.run(p)
            t = build_trajectory(p, d, self.ps)
            c = control.compute(p, t)
            self.assertTrue(t.valid and not t.emergency_stop, t.reason)
            self.assertTrue(c.valid, c.errors)
            safety = supervisor.evaluate(p, d, t, c)
            self.assertEqual('normal', safety.mode, (frame, safety.reason))
            if 20. <= plant.x <= 30.:
                self.assertGreater(plant.speed, 3.)
            plant.step(c, .05, self.calibration)
            now[0] += .05
        self.assertGreater(plant.x, 30.)

    def test_optional_recenter_does_not_require_predecessor_body_coverage(self):
        for boundaries in (True, False):
            for offset in (-.12, -1.3e-6, 1.3e-6, .12):
                for speed in (0., 2.7):
                    p = self.sample(x=-4.92, y=offset, speed=speed)
                    p.lane.predecessor_lane_ids = ['previous']
                    p.lane.lane_width_valid, p.lane.lane_width = True, 3.5
                    if not boundaries:
                        p.lane.left_boundary, p.lane.right_boundary = [], []
                    d, t, safety = self.safety(p)
                    self.assertTrue(t.valid and not t.emergency_stop, t.reason)
                    self.assertEqual('normal', safety.mode, safety.reason)
                    self.assertNotIn('lateral recovery', t.reason)

    def test_required_large_offset_recovery_still_requires_complete_corridor(self):
        p = self.sample(x=-4.92, y=.6, speed=2.7)
        p.lane.predecessor_lane_ids = ['previous']
        d, t, safety = self.safety(p)
        self.assertFalse(t.valid, t.reason)
        self.assertEqual('trajectory_invalid_or_expired', safety.reason)

    def test_speed_dependent_blend_cannot_borrow_successor_boundaries(self):
        p = self.sample(x=2., y=.3, speed=8.)
        p.lane.center_line = [(float(i), 0.) for i in range(-5, 15)]
        p.lane.left_boundary = [(-5., 1.75), (14., 1.75)]
        p.lane.right_boundary = [(-5., -1.75), (14., -1.75)]
        p.lane.forward_reference_valid = True
        p.lane.forward_reference = p.lane.center_line + [(float(i), 0.) for i in range(15, 105)]
        p.lane.forward_lane_ids = [p.lane.lane_id, 'next']
        p.lane.forward_reference_status = 'map_end'
        d, t, safety = self.safety(p)
        self.assertTrue(t.valid and not t.emergency_stop, t.reason)
        self.assertEqual('normal', safety.mode, safety.reason)
        self.assertNotIn('lateral recovery', t.reason)

    def test_short_planning_horizon_does_not_force_incomplete_comfort_blend(self):
        for horizon in (8., 10.):
            self.ps.horizon = horizon
            p = self.sample(y=.12)
            d, t, safety = self.safety(p)
            self.assertTrue(t.valid and not t.emergency_stop, t.reason)
            self.assertEqual('normal', safety.mode, safety.reason)
            self.assertNotIn('lateral recovery', t.reason)

    def test_red_green_restart_crosses_lane_seams_without_fault_braking(self):
        plant = BidirectionalPlant(x=2., y=1.3e-6)
        now = [0.]
        control = ControlEngine(self.calibration, clock=lambda: now[0])
        decision = DecisionEngine(self.ds)
        supervisor = SafetySupervisor(SimpleNamespace(), planning_settings=self.ps,
                                      decision_settings=self.ds)
        red_stopped = False
        crossed = set()
        for frame in range(1, 851):
            p = self.sample(plant.x, plant.y, plant.heading, plant.speed, frame)
            # Floating-point GPS samples need not converge below the numerical
            # geometry epsilon, even when the actual car tracks the centre.
            p.ego.y += 2e-6 if frame % 2 else -2e-6
            p.ego.yaw_rate = plant.yaw_rate
            start, end, name = ((-5, 15, 'before') if plant.x < 15 else
                                (15, 35, 'middle') if plant.x < 35 else (35, 204, 'after'))
            crossed.add(name)
            p.lane.lane_id = name
            p.lane.center_line = [(float(i), 0.) for i in range(start, end+1)]
            p.lane.left_boundary = [(float(start), 1.75), (float(end), 1.75)]
            p.lane.right_boundary = [(float(start), -1.75), (float(end), -1.75)]
            p.lane.forward_reference = p.lane.center_line + [(float(i), 0.) for i in range(end+1, 205)]
            p.lane.forward_reference_valid = True
            p.lane.forward_lane_ids = [name] + (['later'] if end < 204 else [])
            p.lane.forward_reference_status = 'map_end'
            p.traffic.observed, p.traffic.valid, p.traffic.association_valid = True, True, True
            p.traffic.signal_state = 'RED' if frame <= 250 else 'GREEN'
            p.traffic.stop_line_distance = 18.-plant.x
            p.source_status['traffic'] = {'association_valid': True, 'usable': True}
            d = decision.run(p)
            t = build_trajectory(p, d, self.ps)
            c = control.compute(p, t)
            self.assertTrue(t.valid and not t.emergency_stop, (frame, t.reason))
            self.assertTrue(c.valid, (frame, c.errors))
            safety = supervisor.evaluate(p, d, t, c)
            self.assertEqual('normal', safety.mode, (frame, safety.reason))
            if frame <= 250:
                self.assertLessEqual(plant.x+self.ps.front_offset_m, 18.)
                red_stopped = red_stopped or (frame > 150 and plant.speed < .1)
            plant.step(c, .05, self.calibration)
            now[0] += .05
        self.assertTrue(red_stopped)
        self.assertEqual({'before', 'middle', 'after'}, crossed)
        self.assertGreater(plant.x, 45.)
        self.assertGreater(plant.speed, 3., (plant.x, d.reason, t.reason, c.diagnostics))

    def test_recentered_pose_still_obeys_original_signal_stop_boundary(self):
        p = self.sample(y=.12, speed=2.)
        p.traffic.observed, p.traffic.valid, p.traffic.association_valid = True, True, True
        p.traffic.signal_state, p.traffic.stop_line_distance = 'RED', 30.
        p.source_status['traffic'] = {'association_valid': True, 'usable': True}
        d, t, safety = self.safety(p)
        self.assertTrue(t.valid and not t.emergency_stop, t.reason)
        self.assertIn('lateral recovery', t.reason)
        self.assertTrue(t.stop_required)
        self.assertLessEqual(t.stop_distance, 30.-self.ps.front_offset_m-self.ps.traffic_stop_margin)
        self.assertEqual('normal', safety.mode, safety.reason)

    def test_signal_stop_is_preserved_while_ignoring_clear_roadside_target(self):
        p = self.roadside()
        p.traffic.observed, p.traffic.valid = True, True
        p.traffic.signal_state, p.traffic.stop_line_distance = 'RED', 30.
        d, t, safety = self.safety(p)
        self.assertIn('STOP_TRAFFIC', d.reason)
        self.assertTrue(t.stop_required)
        self.assertLessEqual(t.stop_distance, 30.-self.ps.front_offset_m-self.ps.traffic_stop_margin)
        self.assertEqual('normal', safety.mode, safety.reason)


if __name__ == '__main__':
    unittest.main()
