"""Route continuity regressions with public consumers and fake HDMap only."""
import math
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from core.geometry import project_polyline, projection_within_polyline
from core.interfaces import DecisionMode, Target
from members.decision.engine import DecisionEngine
from members.decision.settings import DecisionSettings
from members.planning.lane_planner import PlannerSettings, build_trajectory
from perception.perception_builder import PerceptionBuilder
from tests.test_decision import perception, add_target
from tests.test_route_continuation import Map, NativeId, manager, ego
from tests.test_task_route_selection import branch_map


def mapped(segments, successors=None, widths=None):
    api = Map(segments, successors)
    api.pySimString = NativeId
    api.getLaneWidth = lambda native, position: SimpleNamespace(
        exists=True, width=(widths or {}).get(native.GetString(), 3.5))
    return api


def moving(x, y=0.0, heading=0.0, speed=3.0):
    value = ego(x, y, heading)
    value.speed = speed
    value.vx, value.vy = speed*math.cos(heading), speed*math.sin(heading)
    return value


def public_frame(lane, state, frame=1):
    p = perception(speed=state.speed, frame_id=frame)
    p.lane, p.ego = lane, state
    p.ego.frame_id, p.ego.age_ms = frame, 0
    p.ego.timestamp = p.timestamp
    return p


class RouteHandoffRegressionTests(unittest.TestCase):
    def setUp(self):
        self.ds = DecisionSettings(front_offset_m=3.9187)
        self.ps = PlannerSettings(front_offset_m=3.9187, half_width_m=.9)

    def test_continuously_stale_native_lane_does_not_undo_promotion(self):
        api = mapped({'a':[(0,0),(20,0)], 'b':[(20,0),(180,0)]}, {'a':['b']})
        route, decision = manager(api), DecisionEngine(self.ds)
        base = time.monotonic()
        with patch('perception.route_manager.time.monotonic', return_value=base):
            route.update(moving(19.99), True)
        positions = []
        for frame, elapsed in enumerate((.01,.02,.22,.23,.24,.46,.47,.48), 1):
            x = 20.02 + 3*(elapsed-.01)
            with patch('perception.route_manager.time.monotonic', return_value=base+elapsed):
                state = moving(x)
                lane = route.update(state)
                self.assertEqual('a', api.current)
                self.assertEqual('b', lane.lane_id)
                p = public_frame(lane, state, frame)
                d = decision.run(p)
                self.assertEqual(DecisionMode.KEEP_LANE, d.mode, d.reason)
                t = build_trajectory(p, d, self.ps)
                self.assertTrue(t.valid, t.reason)
                self.assertFalse(t.emergency_stop or t.stop_required)
            positions.append((elapsed,x))
        for a,b in zip(positions,positions[1:]):
            self.assertAlmostEqual(3*(b[0]-a[0]), b[1]-a[1])

    def test_multiple_short_verified_segments_are_reachable_in_one_refresh(self):
        api = mapped({'a':[(0,0),(20,0)], 'b':[(20,0),(20.5,0)],
                      'c':[(20.5,0),(180,0)]}, {'a':['b'],'b':['c']})
        route = manager(api)
        base = time.monotonic()
        with patch('perception.route_manager.time.monotonic', return_value=base):
            route.update(moving(19.9), True)
        with patch('perception.route_manager.time.monotonic', return_value=base+.7/3):
            state = moving(20.6)
            lane = route.update(state)
            self.assertEqual('c', lane.lane_id)
            p = public_frame(lane,state)
            add_target(p, longitudinal=30, speed=3, lane_id='c')
            d = DecisionEngine(self.ds).run(p)
            self.assertEqual(DecisionMode.FOLLOW,d.mode,d.reason)
            t = build_trajectory(p,d,self.ps)
            self.assertTrue(t.valid,t.reason)
            self.assertFalse(t.emergency_stop)

    def test_reachable_first_leg_cannot_jump_to_nearby_return_leg(self):
        curve = [(20+10*math.sin(i*math.pi/60),10*(1-math.cos(i*math.pi/60)))
                 for i in range(121)]
        api = mapped({'a':[(0,0),(20,0)],'b':curve,'c':[(20,0),(180,0)]},
                     {'a':['b'],'b':['c']})
        route = manager(api)
        base = time.monotonic()
        with patch('perception.route_manager.time.monotonic', return_value=base):
            route.update(moving(19.9),True)
        with patch('perception.route_manager.time.monotonic', return_value=base+.2/3):
            lane = route.update(moving(20.1),True)
        self.assertEqual('b',lane.lane_id)
        self.assertGreater(sum(math.hypot(b[0]-a[0],b[1]-a[1])
                               for a,b in zip(lane.center_line,lane.center_line[1:])),60)

    def test_unselected_branch_still_prevents_false_promotion_after_heading_alignment(self):
        route = manager(branch_map())
        route.set_route_hint([(0,0),(40,8)],True)
        route.update(moving(19),True)
        lane = route.update(moving(22,-.51,-.1),True)
        self.assertEqual('a',lane.lane_id)
        d = DecisionEngine(self.ds).run(public_frame(lane,moving(22,-.51,-.1)))
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE,d.mode)

    def test_changed_geometry_width_height_or_spans_do_not_authorize_recovery(self):
        for invalid in ('width','height','geometry','spans'):
            with self.subTest(invalid=invalid):
                api = mapped({'a':[(0,0),(20,0)],'b':[(20,0),(180,0)]},{'a':['b']})
                route = manager(api)
                route.update(moving(19.9),True)
                state = moving(20.02)
                if invalid == 'width':
                    api.getLaneWidth = lambda *args: SimpleNamespace(exists=False,width=0)
                elif invalid == 'height':
                    state.z = 1
                elif invalid == 'geometry':
                    api.segments['b'] = [(21,0),(180,0)]
                else:
                    route.last_lane.forward_lane_spans[1]['start_index'] = 0
                self.assertEqual('a',route.update(state,True).lane_id)

    def test_recovery_heading_uses_forward_consumer_support_not_join_tolerance(self):
        for heading, expected in ((.36,'b'),(.80,'a')):
            with self.subTest(heading=heading):
                api = mapped({'a':[(0,0),(20,0)],'b':[(20,0),(180,0)]},{'a':['b']})
                route = manager(api)
                route.update(moving(19.9),True)
                state = moving(20.02,heading=heading)
                lane = route.update(state,True)
                self.assertEqual(expected,lane.lane_id)
                if expected == 'b':
                    p = public_frame(lane,state)
                    d = DecisionEngine(self.ds).run(p)
                    self.assertGreater(d.target_speed,0,d.reason)
                    t = build_trajectory(p,d,self.ps)
                    self.assertTrue(t.valid,t.reason)

    def test_task_change_clears_stale_native_recovery_evidence(self):
        api = mapped({'a':[(0,0),(20,0)],'b':[(20,0),(180,0)]},{'a':['b']})
        route = manager(api)
        route.set_route_hint([(0,0),(100,0)],True,('one','task'))
        route.update(moving(19.9),True)
        self.assertEqual('b',route.update(moving(20.02),True).lane_id)
        route.set_route_hint([(0,0),(100,0)],True,('two','task'))
        self.assertEqual('a',route.update(moving(20.04),True).lane_id)

    def test_current_lane_internal_vertex_retains_target_membership_and_follow(self):
        api = mapped({'a':[(0,0),(20,0),(40,4),(150,26)]})
        route = manager(api)
        lane = route.update(moving(0,speed=0),True)
        p = public_frame(lane,moving(0,speed=0))
        target = add_target(p,longitudinal=20.02,speed=3)
        target.y,target.height,target.heading = -.3,1.4,.1
        target.vx,target.vy = 3*math.cos(.1),3*math.sin(.1)
        target.lane_id,target.same_lane_valid = route.locate_target(target)
        target.same_lane = target.same_lane_valid and target.lane_id == lane.lane_id
        self.assertEqual(('a',True),(target.lane_id,target.same_lane_valid))
        self.assertEqual(3.5,target.lane_width_m)
        d = DecisionEngine(self.ds).run(p)
        self.assertEqual(DecisionMode.FOLLOW,d.mode,d.reason)
        t = build_trajectory(p,d,self.ps)
        self.assertTrue(t.valid,t.reason)
        self.assertFalse(t.emergency_stop)

    def test_actual_route_start_end_width_and_height_still_reject_target(self):
        api = mapped({'a':[(0,0),(20,0),(40,4)]})
        route = manager(api)
        route.update(moving(0),True)
        for x,y,z in ((-.01,0,0),(40.1,4.02,0),(10,4,0),(10,0,8)):
            target = Target()
            target.x,target.y,target.z,target.height = x,y,z,1.4
            self.assertEqual(('',False),route.locate_target(target))
            self.assertIsNone(target.lane_width_m)

    def test_shared_projection_accepts_internal_vertex_but_not_true_end_extrapolation(self):
        points = [(0,0),(20,0),(40,4)]
        internal = project_polyline(points,20.02,-.3)
        self.assertGreater(internal['raw_ratio'],1)
        self.assertTrue(projection_within_polyline(internal,len(points)))
        self.assertFalse(projection_within_polyline(project_polyline(points,-.01,0),len(points)))
        self.assertFalse(projection_within_polyline(project_polyline(points,40.1,4.02),len(points)))

    def test_future_fork_skips_intermediate_and_shared_join_task_points(self):
        for middle in ((30,0),(40,0)):
            api = mapped({'a':[(0,0),(20,0)],'b':[(20,0),(40,0)],
                          'c':[(40,0),(41,.1),(60,8),(200,66.21052631578948)],
                          'd':[(40,0),(41,-.1),(60,-8)]}, {'a':['b'],'b':['c','d']})
            route = manager(api)
            route.set_route_hint([(0,0),middle,(55,6)],True)
            lane = route.update(moving(10),True)
            self.assertEqual(['a','b','c'],lane.forward_lane_ids)
            p = public_frame(lane,moving(10))
            d = DecisionEngine(self.ds).run(p)
            t = build_trajectory(p,d,self.ps)
            self.assertTrue(t.valid,t.reason)
            self.assertFalse(t.stop_required or t.emergency_stop)

    def test_future_prefix_waypoint_uses_that_segments_measured_width(self):
        api = mapped({'a':[(0,0),(20,0)],'b':[(20,0),(40,0)],
                      'c':[(40,0),(41,.1),(60,8)],'d':[(40,0),(41,-.1),(60,-8)]},
                     {'a':['b'],'b':['c','d']},widths={'b':8.0})
        route = manager(api)
        route.set_route_hint([(0,0),(30,2),(55,6)],True)
        self.assertEqual(['a','b','c'],route.update(moving(10),True).forward_lane_ids)

    def mapped_signal_frame(self, x, y, sign_heading=math.pi, sign_valid=True):
        api = mapped({'1_0_-1':[(0,0),(20,0),(40,4)]})
        api.current = '1_0_-1'
        adapter = SimpleNamespace(map_loaded=True,hdmap=api,
            read_traffic=lambda lane_id: [dict(opendrive_id=51,status=1,
                x=25.0,y=1.0,stop_line_x=x,stop_line_y=y,read_ok=True)],
            last_traffic_query=dict(association_valid=True,available=True,read_ok=True))
        sign = dict(id=9,type='1010203800001413',value='20',unit='km/h',
                    x=x,y=y,valid=sign_valid,is_dynamic=False,
                    heading=sign_heading,heading_valid=True,
                    validities=[dict(road_id=1,section_index=0,from_lane_id=-1,to_lane_id=-1)])
        raw = dict(gps=dict(frame=1,timestamp=50,x=0.0,y=0.0,z=0.0,heading=0.0,
                            vx=0.0,vy=0.0,age_ms=0),
                   targets=[],targets_valid=True,target_source='sensor:test',
                   targets_frame=1,targets_timestamp=50,targets_age_ms=0,
                   traffic_signs=[sign],traffic_signs_valid=True)
        return PerceptionBuilder(adapter,manager(api),scene_override=6).build_from_raw(raw)

    def test_builder_publishes_internal_vertex_stopline_and_sign_approach(self):
        p = self.mapped_signal_frame(20.02,-.3)
        self.assertTrue(p.valid)
        self.assertTrue(p.traffic.valid)
        self.assertEqual('RED',p.traffic.signal_state)
        self.assertAlmostEqual(20,p.traffic.stop_line_distance)
        sign = p.speed_limit_observations[0]
        self.assertTrue(sign['applicable'])
        self.assertFalse(sign['active'])
        self.assertAlmostEqual(20,sign['along_distance'])

    def test_builder_semantics_keep_true_endpoint_and_direction_source_gates(self):
        for x,y in ((-.01,0),(40.1,4.02)):
            p = self.mapped_signal_frame(x,y)
            self.assertFalse(p.traffic.valid)
            self.assertEqual([],p.traffic.candidates)
            self.assertFalse(p.speed_limit_observations[0]['applicable'])
            self.assertEqual('sign_outside_lane_coverage',p.speed_limit_observations[0]['reason'])
        for heading,valid in ((0,True),(math.pi,False)):
            p = self.mapped_signal_frame(20.02,-.3,heading,valid)
            self.assertTrue(p.traffic.valid)
            self.assertFalse(p.speed_limit_observations[0]['applicable'])


if __name__ == '__main__':
    unittest.main()
