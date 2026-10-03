"""Route-relative motion and signal/target arbitration, not scene-ID rules."""
import math
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from core.interfaces import DecisionMode, Target
from members.decision.engine import DecisionEngine
from members.decision.settings import DecisionSettings
from members.planning.lane_planner import PlannerSettings, build_trajectory
from members.control.controller import ControlEngine
from tests.test_control import calibration
from tests.test_decision import perception, add_target, add_red_light
from tests.test_route_continuation import Map, NativeId, manager, ego
from tests import test_planning_route_follow as curve_fixtures


class GeneralRouteMotionTests(unittest.TestCase):
    def setUp(self):
        self.ds = DecisionSettings(front_offset_m=3.9187)
        self.ps = PlannerSettings(front_offset_m=3.9187, half_width_m=.9)

    def test_wide_curve_lead_motion_is_not_constant_straight_lateral_drift(self):
        fixture = curve_fixtures.PlanningRouteFollowTests()
        p, unused = fixture.observation()
        target = p.targets[0]
        target.width, target.heading = 2.23, target.heading + .06
        target.vx, target.vy = 5*math.cos(target.heading), 5*math.sin(target.heading)
        with patch('members.planning.obstacle_guard.lateral_residual', return_value=None):
            before = build_trajectory(p, DecisionEngine(self.ds).run(p), self.ps)
        self.assertFalse(before.valid)
        d = DecisionEngine(self.ds).run(p)
        self.assertEqual(DecisionMode.FOLLOW, d.mode, d.reason)
        after = build_trajectory(p, d, self.ps)
        self.assertTrue(after.valid, after.reason)
        self.assertFalse(after.emergency_stop)
        self.assertGreater(after.points[1].speed, 0)

    def test_unknown_object_beyond_route_end_does_not_override_red_or_green(self):
        engine = DecisionEngine(self.ds)
        p = perception(speed=0, frame_id=1)
        p.lane.center_line = [(0,0),(100,0)]
        target = add_target(p, longitudinal=140, speed=0, same_lane_valid=False, lane_id='')
        target.id = 51
        add_red_light(p,30)
        red = engine.run(p)
        self.assertIn('STOP_TRAFFIC', red.reason)
        self.assertGreater(red.stop_distance,0)
        p.frame_id += 1
        p.traffic.signal_state, p.traffic.valid = 'GREEN', True
        green = engine.run(p)
        self.assertEqual(DecisionMode.KEEP_LANE, green.mode, green.reason)
        self.assertGreater(green.target_speed,0)
        t = build_trajectory(p,green,self.ps)
        self.assertTrue(t.valid,t.reason)
        self.assertFalse(t.emergency_stop)

    def test_swept_crossing_intersection_is_checked_between_observed_endpoints(self):
        p = perception(speed=3)
        target = add_target(p,longitudinal=10,speed=0,same_lane_valid=False,lane_id='')
        target.y, target.vy, target.heading = 10,-6,-math.pi/2
        d = DecisionEngine(self.ds).run(p)
        self.assertIn(d.mode,(DecisionMode.STOP,DecisionMode.EMERGENCY_BRAKE))
        self.assertIn('TARGET_',d.reason)

    def test_other_lane_oncoming_car_does_not_prevent_green_launch(self):
        p = perception(speed=0)
        target = add_target(p,longitudinal=30,speed=-5,lane_id='other')
        target.y, target.heading = 8,math.pi
        p.traffic.observed,p.traffic.valid,p.traffic.signal_state = True,True,'GREEN'
        d = DecisionEngine(self.ds).run(p)
        self.assertGreater(d.target_speed,0)
        t = build_trajectory(p,d,self.ps)
        self.assertTrue(t.valid,t.reason)
        target.y,target.lane_id,target.same_lane = 0,p.lane.lane_id,True
        self.assertIn(DecisionEngine(self.ds).run(p).mode,(DecisionMode.STOP,DecisionMode.EMERGENCY_BRAKE))

    def test_real_static_hazard_disappearance_still_needs_clear_confirmation(self):
        engine = DecisionEngine(self.ds)
        p = perception(speed=0,frame_id=1)
        add_target(p,longitudinal=5,speed=0)
        self.assertEqual(DecisionMode.STOP,engine.run(p).mode)
        self.assertEqual(DecisionMode.STOP,engine.run(perception(speed=0,frame_id=2)).mode)
        # The normal HOLD state independently confirms release after hazards clear.
        outputs=[engine.run(perception(speed=0,frame_id=i)) for i in range(3,6)]
        self.assertEqual(DecisionMode.KEEP_LANE,outputs[-1].mode)

    def test_nearest_lane_endpoint_failure_uses_verified_successor_geometry(self):
        api = Map({'a':[(0,0),(20,0)],'b':[(20.05,0),(21,.03),(22,.1)]},{'a':['b']})
        api.pySimString = NativeId
        api.getLaneWidth = lambda *args: SimpleNamespace(exists=True,width=3.97)
        route = manager(api)
        route.update(ego(19),force=True)
        target = Target()
        target.x,target.y,target.height = 20.02,-.03,1.4
        self.assertEqual(('b',True),route.locate_target(target))
        self.assertEqual(3.97,target.lane_width_m)
        target.z = 8
        self.assertEqual(('',False),route.locate_target(target))
        self.assertIsNone(target.lane_width_m)

    def test_standstill_rollback_is_consistent_through_real_members(self):
        clock = [time.monotonic()]
        controller = ControlEngine(calibration(),clock=lambda:clock[0])
        for frame in range(1,5):
            clock[0] += .05
            p = perception(speed=.03,frame_id=frame)
            p.ego.frame_id,p.ego.age_ms = frame,0
            p.ego.vx = -.03
            d = DecisionEngine(self.ds).run(p)
            self.assertGreater(d.target_speed,0,d.reason)
            t = build_trajectory(p,d,self.ps)
            self.assertTrue(t.valid,t.reason)
            c = controller.compute(p,t)
            self.assertTrue(c.valid,c.errors)
        self.assertGreater(c.throttle,0)
        p.ego.speed,p.ego.vx = .06,-.06
        d = DecisionEngine(self.ds).run(p)
        self.assertIn('EGO_MOTION',d.reason)
        self.assertIn(d.mode,(DecisionMode.STOP,DecisionMode.EMERGENCY_BRAKE))

    def test_unsupported_motion_far_outside_planning_range_is_not_an_immediate_brake(self):
        p = perception(speed=8)
        target = add_target(p,longitudinal=80,speed=8)
        target.y,target.vy = .6,.04
        p.lane.lane_width = 3.5
        d = DecisionEngine(self.ds).run(p)
        t = build_trajectory(p,d,self.ps)
        self.assertTrue(t.valid,t.reason)
        self.assertFalse(t.emergency_stop)
