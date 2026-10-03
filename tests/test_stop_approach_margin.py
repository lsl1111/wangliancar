"""Approach braking must start before the hard infeasibility boundary."""
import math
import unittest

from core.interfaces import DecisionMode, DecisionTarget, Trajectory, ControlOut
from core.validation import validate_output
from members.control.controller import ControlEngine
from members.decision.engine import DecisionEngine
from members.decision.settings import DecisionSettings
from members.planning.lane_planner import PlannerSettings, build_trajectory
from tests.control_benchmark import vehicle
from tests.test_control_maneuvers import BidirectionalPlant
from tests.test_decision import perception, add_red_light, add_target


class StopApproachMarginTests(unittest.TestCase):
    def test_red_or_static_stop_requests_braking_while_still_feasible(self):
        for source in ('red', 'static'):
            p = perception(speed=6, frame_id=1)
            p.ego.frame_id, p.ego.age_ms = 1, 0
            if source == 'red':
                add_red_light(p, 20)
            else:
                add_target(p, longitudinal=22, speed=0)
            d = DecisionEngine(DecisionSettings(front_offset_m=3.9187)).run(p)
            self.assertEqual(DecisionMode.KEEP_LANE, d.mode, d.reason)
            self.assertGreater(d.target_speed, 0)
            self.assertLess(d.target_speed, p.ego.speed)
            t = build_trajectory(p, d, PlannerSettings(front_offset_m=3.9187, half_width_m=.9))
            self.assertTrue(t.valid, t.reason)
            self.assertFalse(t.emergency_stop, t.reason)
            self.assertLess(t.points[1].speed, t.points[0].speed)

    def test_near_red_still_requires_emergency_braking(self):
        p = perception(speed=6)
        add_red_light(p, 10)
        d = DecisionEngine(DecisionSettings(front_offset_m=3.9187)).run(p)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, d.mode)
        self.assertIn('STOP_INFEASIBLE', d.reason)

    def test_ratio_is_bounded_and_scales_existing_deceleration_override(self):
        for value in (0, -0.5, 1.1, float('nan')):
            with self.assertRaises(ValueError):
                DecisionSettings(approach_deceleration_ratio=value).validate()
        s = DecisionSettings.from_environment({'NEVC_DECISION_FOLLOW_DECELERATION': '.5',
            'NEVC_DECISION_APPROACH_DECELERATION_RATIO': '.4'})
        self.assertEqual(.2, s.follow_deceleration*s.approach_deceleration_ratio)

    def test_real_signal_chain_stops_and_resumes_without_routine_emergency(self):
        # Inputs and plant are synthetic; all algorithm outputs are real.
        for speed in (0, 6):
            for lag in (.1, .3, .5):
                with self.subTest(initial_speed=speed, actuator_lag=lag):
                    self.signal_chain(speed, lag)

    def signal_chain(self, initial_speed, lag):
        plant, now = BidirectionalPlant(speed=initial_speed, lag=lag), [0.0]
        calibration = vehicle()
        controller = ControlEngine(calibration, clock=lambda: now[0])
        controller.integral = 3.4383475565878467
        decision = DecisionEngine(DecisionSettings(front_offset_m=3.9187))
        planning = PlannerSettings(front_offset_m=3.9187, half_width_m=.9)
        held, restarted = False, False
        for frame in range(1, 461):
            p = perception(speed=plant.speed, frame_id=frame, ttl=30)
            p.ego.frame_id, p.ego.age_ms = frame, 0
            p.ego.x, p.ego.y, p.ego.heading = plant.x, plant.y, plant.heading
            p.ego.vx, p.ego.vy = plant.speed*math.cos(plant.heading), plant.speed*math.sin(plant.heading)
            p.scene_id, p.task_id, p.case_id = 10, 'synthetic-signal', 'synthetic-signal'
            p.traffic.observed = p.traffic.valid = p.traffic.association_valid = True
            p.traffic.required, p.traffic.signal_presence = True, 'present'
            p.traffic.signal_state = 'RED' if now[0] < 20 else 'GREEN'
            p.traffic.stop_line_distance = max(0, 40-plant.x)
            p.source_status['traffic'] = dict(usable=True, quality='ok', association_valid=True)
            d = decision.run(p)
            t = build_trajectory(p, d, planning)
            c = controller.compute(p, t)
            validate_output(d, DecisionTarget, p)
            validate_output(t, Trajectory, d)
            validate_output(c, ControlOut, t)
            self.assertNotEqual(DecisionMode.EMERGENCY_BRAKE, d.mode, (frame, d.reason))
            self.assertFalse(t.emergency_stop, (frame, t.reason))
            self.assertFalse(c.throttle > 0 and c.brake > 0)
            if 18 <= now[0] < 20:
                self.assertLess(plant.speed, .05)
                self.assertLess(plant.x + 3.9187, 40)
                held = True
            if now[0] >= 20 and c.throttle > 0:
                restarted = True
            if now[0] >= 20.3:
                self.assertTrue(restarted)
            plant.step(c, .05, calibration)
            now[0] += .05
        self.assertTrue(held)
        self.assertTrue(restarted)
        self.assertGreater(plant.speed, .5)
