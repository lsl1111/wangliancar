"""Body collision and bounded target position across decision/plan/safety.

Synthetic geometry reproduces the recorded failure without case coordinates,
target IDs, a SimOne connection or a changed projection tolerance.
"""
import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from core.interfaces import DecisionMode, DecisionTarget
from core.route_obstacles import RouteContext, mapped_route_motion
from core.safety_supervisor import SafetySupervisor
from members.decision.engine import DecisionEngine
from members.decision.settings import DecisionSettings
from members.planning.lane_planner import PlannerSettings, build_trajectory
from members.planning.obstacle_guard import _route_association
from tests.test_decision import perception, add_target, add_red_light
import tests.test_pipeline_integration as integration


class WideLaneTargetHandoffTests(unittest.TestCase):
    def setUp(self):
        self.ds = DecisionSettings(front_offset_m=3.9187, half_width_m=.9)
        self.ps = PlannerSettings(front_offset_m=3.9187, half_width_m=.9)

    def frame(self, lateral=3.25, distance=31, speed=0, width=12):
        p = perception(speed=speed)
        p.scene_id = 7
        p.ego.frame_id, p.ego.age_ms = p.frame_id, 0
        p.lane.lane_width = width
        target = add_target(p, longitudinal=distance, length=4.8)
        target.y, target.width, target.heading = lateral, 2.23, math.pi/2
        target.lane_width_m = width
        return p

    def chain(self, p, engine=None):
        d = (engine or DecisionEngine(self.ds)).run(p)
        t = build_trajectory(p, d, self.ps)
        safety = SafetySupervisor(SimpleNamespace(), planning_settings=self.ps,
                                  decision_settings=self.ds).evaluate(p, d, t)
        self.assertTrue(d.valid, d.reason)
        self.assertTrue(t.valid, t.reason)
        return d, t, safety

    def test_wide_lane_overlap_approaches_a_body_safe_boundary(self):
        for scene in (1, 7, 15):
            p = self.frame()
            p.scene_id = scene
            d, t, safety = self.chain(p)
            self.assertNotIn('TARGET_UNKNOWN', d.reason)
            self.assertGreater(d.target_speed, 0)
            self.assertGreater(d.stop_distance, 10)
            self.assertGreater(max(q.speed for q in t.points), 0)
            self.assertTrue(t.stop_required)
            self.assertLessEqual(t.stop_distance, d.stop_distance+1e-6)
            clearance = self.ds.min_gap if scene == 15 else self.ds.obstacle_stop_margin
            self.assertLessEqual(t.points[-1].x+self.ps.front_offset_m+clearance,
                                 p.targets[0].x-p.targets[0].width*.5+1e-6)
            self.assertEqual(0, t.points[-1].speed)
            self.assertEqual('normal', safety.mode, safety.reason)
        self.assertEqual(2.5, self.ds.projection_tolerance_m)
        self.assertEqual(2.5, self.ps.projection_tolerance_m)

    def test_position_is_shared_without_widening_trajectory_projection(self):
        p = self.frame()
        target = p.targets[0]
        route = RouteContext(p, self.ds)
        self.assertIsNone(route.project(target.x, target.y, self.ds, target.lane_id))
        projection = route.project_target(target, self.ds)
        self.assertAlmostEqual(31, projection['s']-route.ego_s)
        association = _route_association(p, target, self.ps)
        motion = mapped_route_motion(p, target, route, self.ds, self.ps.front_offset_m)
        self.assertIsNotNone(association)
        self.assertIsNotNone(motion)
        self.assertAlmostEqual(31, association[0]['s'])
        self.assertAlmostEqual(31, motion['distance'])
        self.assertIsNone(route.project(target.x, target.y, self.ds))

    def test_parked_body_clear_of_straight_route_never_becomes_a_lead(self):
        for lateral in (3.35, -3.35, 5, -5):
            p = self.frame(lateral=lateral, distance=9, speed=8)
            # Old map lane / ego-axis TTC alone describes a hazard, while
            # the oriented body and zero-motion sweep cannot touch our road.
            p.targets[0].ttc = .2
            d, t, safety = self.chain(p)
            self.assertEqual(DecisionMode.KEEP_LANE, d.mode, d.reason)
            self.assertIn('CRUISE', d.reason)
            self.assertFalse(t.stop_required)
            self.assertFalse(SafetySupervisor._imminent_collision(p, self.ps, self.ds))
            self.assertEqual('normal', safety.mode, safety.reason)

    def test_near_body_overlap_retains_emergency_and_independent_guard(self):
        p = self.frame(distance=9, speed=8)
        d, t, safety = self.chain(p)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, d.mode, d.reason)
        self.assertTrue(t.emergency_stop)
        self.assertTrue(SafetySupervisor._imminent_collision(p, self.ps, self.ds))
        self.assertEqual('emergency_stop', safety.mode)

    def test_sideways_motion_that_enters_the_route_still_protects(self):
        p = self.frame(lateral=3.35)
        p.targets[0].vy = -1
        d, t, safety = self.chain(p)
        self.assertEqual(DecisionMode.STOP, d.mode, d.reason)
        self.assertEqual(0, max(q.speed for q in t.points))
        manual = DecisionTarget().bind(p)
        manual.valid, manual.target_speed = True, 5
        self.assertFalse(build_trajectory(p, manual, self.ps).valid)

    def test_missing_or_contradictory_wide_lane_evidence_keeps_protection(self):
        for field in ('membership', 'width', 'width_valid', 'target_width', 'heading', 'extent', 'wrong_lane'):
            p = self.frame()
            if field == 'membership': p.targets[0].same_lane_valid = False
            if field == 'width': p.targets[0].lane_width_m = 4
            if field == 'width_valid': p.lane.lane_width_valid = False
            if field == 'target_width': p.targets[0].lane_width_m = None
            if field == 'heading': p.targets[0].heading = None
            if field == 'extent': p.targets[0].width = 0
            if field == 'wrong_lane': p.targets[0].lane_id = 'unselected'
            d, t, safety = self.chain(p)
            self.assertEqual(DecisionMode.STOP, d.mode, (field, d.reason))
            self.assertEqual(0, max(q.speed for q in t.points))

    def test_successor_requires_its_own_width_and_verified_span(self):
        for change in ('valid', 'no_width', 'narrow', 'no_span', 'wrong_span'):
            p = self.frame(distance=40)
            p.lane.lane_width = 3.5
            p.lane.center_line = [(0, 0), (20, 0)]
            p.lane.forward_reference = [(0, 0), (20, 0), (100, 0)]
            p.lane.forward_reference_valid = True
            p.lane.forward_lane_ids = ['lane-1', 'next']
            p.lane.forward_lane_spans = [
                dict(lane_id='lane-1', start_index=0, end_index=1),
                dict(lane_id='next', start_index=1, end_index=2)]
            target = p.targets[0]
            target.lane_id, target.lane_width_m = 'next', 12
            if change == 'no_width': target.lane_width_m = None
            if change == 'narrow': target.lane_width_m = 4
            if change == 'no_span': p.lane.forward_lane_spans = []
            if change == 'wrong_span': p.lane.forward_lane_spans[1]['start_index'] = 0
            d, t, safety = self.chain(p)
            if change == 'valid':
                self.assertGreater(d.target_speed, 0)
                self.assertGreater(t.stop_distance, 30)
                self.assertEqual('normal', safety.mode, safety.reason)
            else:
                self.assertEqual(DecisionMode.STOP, d.mode, (change, d.reason))
                self.assertIn('TARGET_UNKNOWN', d.reason)

    def test_wide_lane_never_extends_a_known_endpoint(self):
        p = self.frame(distance=21)
        p.lane.center_line = [(0, 0), (20, 0)]
        route = RouteContext(p, self.ds)
        self.assertIsNone(route.project_target(p.targets[0], self.ds))
        self.assertEqual('outside_route_start_or_end', route.projection_reason)
        d, t, safety = self.chain(p)
        self.assertEqual(DecisionMode.STOP, d.mode, d.reason)
        self.assertIn('TARGET_UNKNOWN', d.reason)

    def test_red_light_boundary_still_dominates_a_known_side_target(self):
        p = self.frame(distance=50)
        add_red_light(p, 20)
        d, t, safety = self.chain(p)
        bound = 20-self.ps.front_offset_m-self.ps.traffic_stop_margin
        self.assertLessEqual(d.stop_distance, bound+1e-6)
        self.assertLessEqual(t.stop_distance, bound+1e-6)
        self.assertEqual('normal', safety.mode, safety.reason)

    def test_collision_classification_is_invariant_to_world_pose_and_side(self):
        for heading in (0, .6, -1.7, math.pi):
            for lateral in (3.25, -3.25, 3.35, -3.35):
                p = self.frame(lateral=lateral)
                c, s = math.cos(heading), math.sin(heading)
                def point(x, y):
                    return (120+c*x-s*y, -70+s*x+c*y)
                p.lane.center_line = [point(*q) for q in p.lane.center_line]
                p.ego.x, p.ego.y = point(p.ego.x, p.ego.y)
                p.ego.heading = heading
                target = p.targets[0]
                target.x, target.y = point(target.x, target.y)
                target.heading += heading
                d, t, safety = self.chain(p)
                if abs(lateral) == 3.25:
                    self.assertAlmostEqual(31-1.115-self.ps.front_offset_m-.5,
                                           d.stop_distance)
                    self.assertTrue(t.stop_required)
                else:
                    self.assertFalse(t.stop_required)
                    self.assertIn('CRUISE', d.reason)
                self.assertEqual('normal', safety.mode, safety.reason)

    def test_full_loop_sends_propulsion_with_real_members_and_retained_obstacle(self):
        for lateral in (3.25, 3.35):
            class ParkedSDK(integration.SimulatedSDK):
                def __init__(self, *args, **kwargs):
                    super(ParkedSDK, self).__init__(*args, **kwargs)
                    self.heading = 0

                def install(self, adapter):
                    super(ParkedSDK, self).install(adapter)
                    adapter.hdmap.getLaneWidth = lambda *args: SimpleNamespace(exists=True, width=12)
                    def sample(identity):
                        def line(y):
                            return integration.Vector([SimpleNamespace(x=float(x), y=y, z=0)
                                                       for x in range(161)])
                        return SimpleNamespace(exists=True, laneInfo=SimpleNamespace(
                            centerLine=line(0), leftBoundary=line(6), rightBoundary=line(-6)))
                    adapter.hdmap.getLaneSample = sample

                def targets(self, vehicle, sensor, native):
                    native.frame, native.timestamp, native.objectSize = self.reads, self.reads*50, 1
                    native.objects = [SimpleNamespace(id=99, type=6, posX=41, posY=lateral, posZ=0,
                        velX=0, velY=0, velZ=0, oriZ=math.pi/2,
                        length=4.8, width=2.23, height=1.5, probability=1)]
                    return True

            with patch('tests.test_pipeline_integration.SimulatedSDK', ParkedSDK), \
                    patch.dict('os.environ', {'NEVC_VEHICLE_FRONT_OFFSET_M': '3.9187',
                        'NEVC_VEHICLE_HALF_WIDTH_M': '.9'}):
                sdk = integration.PipelineIntegrationTests().run_pipeline(
                    scene=7, sensor=True, frames=60, initial_offset=0)
            self.assertEqual(60, len(sdk.sent))
            self.assertGreater(sdk.sent[0]['throttle'], 0)
            self.assertGreater(sdk.x, 11)
            self.assertGreater(sdk.speed, 1)
            for frame in sdk.pipelines:
                self.assertTrue(frame['trajectory']['valid'])
                self.assertEqual('sent', frame['send']['reason'])
                self.assertEqual('normal', frame['safety']['mode'])
                self.assertEqual(1, frame['target_input']['object_count'])
                self.assertEqual(lateral == 3.25, frame['trajectory']['stop_required'])


if __name__ == '__main__':
    unittest.main()
