"""Behavior/trajectory agreement over the real production member boundary."""
import copy
import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from core.interfaces import DecisionMode, DecisionTarget
from core.safety_supervisor import SafetySupervisor
from core.validation import validate_output
from core.interfaces import Trajectory
from members.decision.engine import DecisionEngine
from members.decision.settings import DecisionSettings
from members.planning.lane_planner import PlannerSettings, build_trajectory
from runtime import _validate_motion_contract
from tests.test_decision import perception, add_target, add_red_light
import tests.test_pipeline_integration as integration


class DecisionPlanningContractTests(unittest.TestCase):
    def setUp(self):
        self.ds = DecisionSettings(front_offset_m=3.5, half_width_m=.9)
        self.ps = PlannerSettings(front_offset_m=3.5, half_width_m=.9)

    def frame(self, speed=0, width=3.5, y=0, vx=5, vy=0, distance=30):
        p = perception(speed=speed)
        p.ego.frame_id, p.ego.age_ms = p.frame_id, 0
        p.lane.lane_width = width
        target = add_target(p, longitudinal=distance, speed=vx)
        target.y, target.vy = y, vy
        return p

    def chain(self, p, ds=None, ps=None):
        ds, ps = ds or self.ds, ps or self.ps
        d = DecisionEngine(ds).run(p)
        t = build_trajectory(p, d, ps)
        self.assertTrue(d.valid, d.reason)
        self.assertTrue(t.valid, t.reason)
        safety = SafetySupervisor(SimpleNamespace(), planning_settings=ps,
                                  decision_settings=ds).evaluate(p, d, t)
        return d, t, safety

    def test_supported_lateral_drift_on_wide_road_is_followed_without_threshold_stop(self):
        p = self.frame(width=8, vy=.4)
        d, t, safety = self.chain(p)
        self.assertEqual(DecisionMode.FOLLOW, d.mode, d.reason)
        self.assertGreater(t.points[1].speed, 0)
        self.assertEqual('normal', safety.mode)

    def test_drift_that_exceeds_remaining_corridor_is_protected_by_decision(self):
        for speed in (0, 1):
            p = self.frame(speed=speed, y=.5, vy=.2)
            d, t, safety = self.chain(p)
            self.assertEqual(DecisionMode.STOP if speed == 0 else DecisionMode.EMERGENCY_BRAKE,
                             d.mode, d.reason)
            self.assertEqual(speed > 0, t.emergency_stop)
            self.assertTrue(all(point.speed == 0 for point in t.points))

    def test_slow_reverse_object_is_not_mistaken_for_safe_static_approach(self):
        p = self.frame(vx=-.2)
        d, t, safety = self.chain(p)
        self.assertEqual(DecisionMode.STOP, d.mode, d.reason)
        self.assertTrue(all(point.speed == 0 for point in t.points))

    def test_far_unsupported_motion_keeps_boundary_while_near_motion_protects(self):
        for distance in (30, 100):
            p = self.frame(vx=-.2, distance=distance)
            d, t, safety = self.chain(p)
            if distance == 100:
                self.assertIn('TARGET_MOTION_AHEAD', d.reason)
                self.assertGreater(d.target_speed, 0)
                self.assertGreater(d.stop_distance, self.ps.horizon)
                self.assertGreater(t.points[1].speed, 0)
            else:
                self.assertEqual(DecisionMode.STOP, d.mode)

    def test_same_explicit_motion_tolerance_is_used_by_both_members(self):
        environment = {'NEVC_VEHICLE_FRONT_OFFSET_M': '3.5',
            'NEVC_VEHICLE_HALF_WIDTH_M': '.9', 'NEVC_PLANNING_MOTION_TOLERANCE_MPS': '.25'}
        ds, ps = DecisionSettings.from_environment(environment), PlannerSettings.from_environment(environment)
        self.assertEqual(.25, ds.motion_tolerance_mps)
        _validate_motion_contract(ps, ds)
        p = self.frame(vx=-.2)
        d, t, safety = self.chain(p, ds, ps)
        self.assertGreater(d.target_speed, 0)
        self.assertGreater(t.points[1].speed, 0)

    def test_shared_braking_override_reaches_decision_feasibility_not_only_motion_window(self):
        environment = {'NEVC_PLANNING_DECELERATION_MPS2': '1.7'}
        ds, ps = DecisionSettings.from_environment(environment), PlannerSettings.from_environment(environment)
        self.assertEqual(1.7, ds.follow_deceleration)
        _validate_motion_contract(ps, ds)
        for change in ('follow_deceleration', 'motion_tolerance_mps',
                       'projection_tolerance_m', 'route_ambiguity_m'):
            incompatible = copy.copy(ds)
            setattr(incompatible, change, getattr(incompatible, change)+.1)
            with self.assertRaises(ValueError, msg=change):
                _validate_motion_contract(ps, incompatible)

    def test_current_lane_curve_does_not_require_optional_successor_span_metadata(self):
        p = self.frame(distance=30)
        radius = 30
        p.lane.center_line = [(radius*math.sin(i*.1), radius*(1-math.cos(i*.1)), 0)
                              for i in range(16)]
        p.lane.forward_reference_valid = False
        target = p.targets[0]
        angle = .95
        target.x, target.y = radius*math.sin(angle), radius*(1-math.cos(angle))
        target.heading = angle
        target.vx, target.vy = 5*math.cos(angle), 5*math.sin(angle)
        d, t, safety = self.chain(p)
        self.assertEqual(DecisionMode.FOLLOW, d.mode, d.reason)
        self.assertFalse(t.emergency_stop)
        self.assertEqual('normal', safety.mode)

    def test_known_red_bound_limits_motion_relevance_without_hiding_an_intrusion(self):
        for distance, vx, vy in ((40, 0, 1), (40, -12, 0), (8, 0, 1)):
            p = self.frame(vx=vx, vy=vy, distance=distance)
            add_red_light(p, 15)
            d, t, safety = self.chain(p)
            if distance == 40 and vx == 0:
                self.assertIn('STOP_TRAFFIC', d.reason)
                self.assertGreater(d.target_speed, 0)
                self.assertGreater(t.points[1].speed, 0)
                self.assertLess(t.points[-1].x+3.5, 15)
                self.assertEqual('normal', safety.mode)
            else:
                self.assertEqual(DecisionMode.STOP, d.mode)
                self.assertTrue(all(point.speed == 0 for point in t.points))

    def test_planner_uses_its_own_red_bound_for_obstacle_motion_when_decision_omits_it(self):
        p = self.frame(vx=0, vy=1, distance=40)
        add_red_light(p, 15)
        d = DecisionTarget().bind(p)
        d.valid, d.target_speed = True, 5
        t = build_trajectory(p, d, self.ps)
        self.assertTrue(t.valid, t.reason)
        self.assertTrue(t.stop_required)
        self.assertGreater(t.points[1].speed, 0)
        self.assertLess(t.points[-1].x+3.5, 15)

    def test_motion_matrix_keeps_behavior_and_trajectory_contracts_consistent(self):
        for width in (3.5, 8):
            for speed in (0, 3, 8):
                for vx in (0, 5, -.2):
                    for vy in (0, .1, .4, .9):
                        for y in (0, .5):
                            for distance in (20, 40, 100):
                                with self.subTest(width=width, speed=speed, vx=vx,
                                                  vy=vy, y=y, distance=distance):
                                    p = self.frame(speed, width, y, vx, vy, distance)
                                    d, t, safety = self.chain(p)
                                    validate_output(d, DecisionTarget, p)
                                    validate_output(t, Trajectory, d)
                                    if d.stop_distance >= 0:
                                        self.assertTrue(t.stop_required)
                                        self.assertLessEqual(t.stop_distance, d.stop_distance+1e-6)
                                    if not t.emergency_stop:
                                        self.assertAlmostEqual(p.ego.speed, t.points[0].speed)
                                    self.assertLessEqual(max(point.speed for point in t.points),
                                                        max(p.ego.speed, d.target_speed)+1e-6)

    def test_full_sdk_chain_follows_supported_drift_and_holds_unsupported_drift(self):
        for width, lateral, offset, follows in ((8, .4, 0, True), (3.5, .2, .5, False)):
            class DriftSDK(integration.SimulatedSDK):
                def install(self, adapter):
                    super(DriftSDK, self).install(adapter)
                    adapter.hdmap.getLaneWidth = lambda *args: SimpleNamespace(exists=True, width=width)
                    def sample(identity):
                        def line(y):
                            return integration.Vector([SimpleNamespace(x=float(x), y=y, z=0)
                                                       for x in range(161)])
                        return SimpleNamespace(exists=True, laneInfo=SimpleNamespace(
                            centerLine=line(0), leftBoundary=line(width*.5), rightBoundary=line(-width*.5)))
                    adapter.hdmap.getLaneSample = sample
                def targets(self, vehicle, sensor, native):
                    native.frame, native.timestamp = self.reads, self.reads*50
                    native.objectSize = 1
                    native.objects = [SimpleNamespace(id=31, type=6,
                        posX=40+5*self.reads*.05, posY=offset+lateral*self.reads*.05, posZ=0,
                        velX=5, velY=lateral, velZ=0, oriZ=0,
                        length=4, width=1.8, height=1.5, probability=1)]
                    return True
            with patch('tests.test_pipeline_integration.SimulatedSDK', DriftSDK), \
                    patch.dict('os.environ', {'NEVC_DECISION_CRUISE_SPEED': '6',
                        'NEVC_VEHICLE_FRONT_OFFSET_M': '3.5', 'NEVC_VEHICLE_HALF_WIDTH_M': '.9'}):
                sdk = integration.PipelineIntegrationTests().run_pipeline(
                    scene=15, sensor=True, frames=70, initial_offset=0)
            self.assertTrue(all(p['trajectory']['valid'] for p in sdk.pipelines))
            if follows:
                self.assertTrue(all(p['decision']['mode'] == DecisionMode.FOLLOW for p in sdk.pipelines))
                self.assertGreater(sdk.x, 12)
                self.assertGreater(sdk.speed, 1)
            else:
                self.assertTrue(all(p['decision']['mode'] == DecisionMode.STOP for p in sdk.pipelines))
                self.assertTrue(all(command['throttle'] == 0 for command in sdk.sent))
                self.assertAlmostEqual(10, sdk.x)


if __name__ == '__main__':
    unittest.main()
