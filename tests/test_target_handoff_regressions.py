"""Independent route collision facts and target handoff through real modules.

The map/SDK providers are fake; producer, decision, planner and safety are the
production implementations. Passing this suite is not platform validation.
"""
import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from core.interfaces import DecisionMode, DecisionTarget
from core.route_obstacles import motion_guard_distance, motion_guard_time
from core.safety_supervisor import SafetySupervisor
from members.decision.engine import DecisionEngine
from members.decision.settings import DecisionSettings
from members.planning.lane_planner import PlannerSettings, build_trajectory
from perception.perception_builder import PerceptionBuilder
from tests.test_decision import observation, perception, add_target
from tests.test_route_continuation import Map, NativeId, manager


class TargetHandoffRegressions(unittest.TestCase):
    def setUp(self):
        self.ds = DecisionSettings(front_offset_m=3.9187, half_width_m=.9)
        self.ps = PlannerSettings(front_offset_m=3.9187, half_width_m=.9)

    def safety(self, planning=None):
        return SafetySupervisor(SimpleNamespace(), planning_settings=planning or self.ps,
                                decision_settings=self.ds)

    def chain(self, p, engine=None):
        d = (engine or DecisionEngine(self.ds)).run(p)
        t = build_trajectory(p, d, self.ps)
        assessment = self.safety().evaluate(p, d, t)
        return d, t, assessment

    def produced(self, points, target, speed=0, width=3.97, neighbor=False):
        api = Map({'a': points, 'b': [(400, 3.5), (0, 3.5)]})
        api.getLaneWidth = lambda *args: SimpleNamespace(exists=True, width=width)
        if neighbor:
            api.getNearMostLane = lambda position: SimpleNamespace(
                exists=True, laneId=NativeId('b' if position[1] >= 2 else 'a'))
        route = manager(api)
        adapter = SimpleNamespace(map_loaded=True, hdmap=api, read_traffic=lambda *args: [],
                                  config=SimpleNamespace(pipeline_timeout_ms=5000))
        raw = observation(7, points[0][0], points[0][1], 0, 0)
        heading = math.atan2(points[1][1]-points[0][1], points[1][0]-points[0][0])
        raw['gps'].update(heading=heading, vx=speed*math.cos(heading),
                          vy=speed*math.sin(heading))
        raw['target_source'] = 'sensor:test'
        raw['targets'] = [dict(id=1, type=6, z=0, length=4, width=1.8,
                               height=1.4, probability=1, **target)]
        return PerceptionBuilder(adapter, route, scene_override=15).build_from_raw(raw)

    def bend(self, speed=0):
        points = [(12*math.sin(i*math.pi/36), 12*(1-math.cos(i*math.pi/36)))
                  for i in range(37)]
        angle = math.radians(150)
        return self.produced(points, dict(x=12*math.sin(angle),
            y=12*(1-math.cos(angle)), heading=angle,
            vx=5*math.cos(angle), vy=5*math.sin(angle)), speed=speed)

    def test_official_bend_producer_no_longer_turns_ego_axis_ttc_into_emergency(self):
        for speed in (0, 2):
            p = self.bend(speed)
            target = p.targets[0]
            self.assertTrue(target.same_lane_valid and target.same_lane)
            self.assertTrue(p.lane.forward_reference_valid)
            self.assertLess(target.ttc, 2)  # Retained diagnostic ego-axis value.
            d, t, safety = self.chain(p)
            self.assertEqual(DecisionMode.FOLLOW, d.mode, d.reason)
            self.assertTrue(t.valid, t.reason)
            self.assertFalse(t.emergency_stop)
            self.assertEqual('normal', safety.mode, safety.reason)

    def test_independent_road_ttc_still_protects_stopped_lead_and_oncoming(self):
        for ego_speed, lead_speed, heading in ((3, 0, 0), (0, -5, math.pi)):
            p = self.produced([(0, 0), (100, 0)],
                dict(x=8, y=0, vx=lead_speed, vy=0, heading=heading), speed=ego_speed)
            d, t, assessment = self.chain(p)
            self.assertEqual('imminent_target_collision', assessment.reason)
            self.assertEqual('emergency_stop', assessment.mode)
            self.assertEqual(DecisionMode.EMERGENCY_BRAKE, d.mode)

    def test_missing_road_evidence_keeps_existing_ttc_fallback(self):
        for change in ('width', 'heading', 'geometry', 'dimensions'):
            p = self.bend()
            if change == 'width':
                p.lane.lane_width_valid = False
                p.targets[0].lane_width_m = None
            if change == 'heading': p.targets[0].heading = None
            if change == 'geometry': p.lane.center_line = []
            config = self.ps if change != 'dimensions' else PlannerSettings()
            self.assertTrue(SafetySupervisor._imminent_collision(p, config, self.ds), change)

    def test_common_vehicle_environment_supports_legacy_safety_constructor(self):
        p = self.bend()
        with patch.dict('os.environ', {'NEVC_VEHICLE_FRONT_OFFSET_M': '3.9187',
                                      'NEVC_VEHICLE_HALF_WIDTH_M': '.9'}):
            safety = SafetySupervisor(SimpleNamespace())
        self.assertFalse(safety._imminent_collision(p, safety.collision_settings,
                                                   safety.route_settings))

    def test_validated_snapshot_wins_over_conflicting_environment(self):
        with patch.dict('os.environ', {'NEVC_VEHICLE_FRONT_OFFSET_M': '20',
                                      'NEVC_VEHICLE_HALF_WIDTH_M': '10'}):
            safety = self.safety()
        self.assertEqual(self.ps.front_offset_m, safety.collision_settings.front_offset_m)
        self.assertEqual(self.ps.half_width_m, safety.collision_settings.half_width_m)

    def test_long_adjacent_oncoming_vehicle_is_clear_with_actual_footprint(self):
        for length in (5.2, 10, 12):
            p = self.produced([(0, 0), (400, 0)],
                dict(x=30, y=3.5, vx=-5, vy=0, heading=math.pi), width=3.5, neighbor=True)
            p.targets[0].length, p.targets[0].width = length, 2.23
            self.assertTrue(p.targets[0].same_lane_valid)
            self.assertFalse(p.targets[0].same_lane)
            d, t, safety = self.chain(p)
            self.assertEqual(DecisionMode.KEEP_LANE, d.mode, d.reason)
            self.assertTrue(t.valid, t.reason)
            self.assertFalse(t.stop_required or t.emergency_stop)
            self.assertEqual('normal', safety.mode)

    def test_neighbor_future_intrusion_and_real_crossing_remain_protected(self):
        for x, y, vx, vy, heading in ((30, 3.5, -5, -1, math.pi),
                                     (10, 10, 0, -6, -math.pi/2)):
            p = self.produced([(0, 0), (400, 0)],
                dict(x=x, y=y, vx=vx, vy=vy, heading=heading), speed=3,
                width=3.5, neighbor=True)
            d, t, safety = self.chain(p)
            self.assertEqual(DecisionMode.EMERGENCY_BRAKE, d.mode, d.reason)
            self.assertTrue(t.emergency_stop)
            manual = DecisionTarget().bind(p)
            manual.valid, manual.target_speed = True, 5
            self.assertFalse(build_trajectory(p, manual, self.ps).valid)

    def test_far_unsupported_route_motion_has_a_bound_without_immediate_emergency(self):
        for distance in (80, 150):
            p = self.produced([(0, 0), (400, 0)],
                dict(x=distance, y=0, vx=8, vy=1, heading=0), speed=8, width=3.5)
            d, t, safety = self.chain(p)
            self.assertIn('TARGET_MOTION_AHEAD', d.reason)
            self.assertNotEqual(DecisionMode.EMERGENCY_BRAKE, d.mode)
            self.assertGreater(d.target_speed, 0)
            self.assertGreater(d.stop_distance, self.ps.horizon)
            self.assertTrue(t.valid, t.reason)
            self.assertFalse(t.emergency_stop)
            self.assertEqual('normal', safety.mode, safety.reason)
            self.assertAlmostEqual(distance-2-self.ps.front_offset_m-.5, t.stop_distance)

    def test_motion_range_is_shared_and_explicitly_configurable(self):
        p = self.produced([(0, 0), (400, 0)],
            dict(x=40, y=0, vx=5, vy=1, heading=0), speed=3, width=3.5)
        ds = self.ds.replace(motion_horizon_m=20)
        short = DecisionEngine(ds).run(p)
        self.assertIn('TARGET_MOTION_AHEAD', short.reason)
        long = DecisionEngine(self.ds).run(p)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, long.mode)
        ps = PlannerSettings(front_offset_m=3.9187, half_width_m=.9, horizon=20)
        self.assertTrue(build_trajectory(p, short, ps).valid)
        self.assertEqual(20, motion_guard_distance(3, 20, 2, 3.9187))
        self.assertAlmostEqual(67.9187, motion_guard_distance(16, 20, 2, 3.9187))
        self.assertEqual(8, motion_guard_time(16, 3, 2))

    def test_target_entering_braking_reach_is_not_treated_as_far_and_clear(self):
        p = self.produced([(0, 0), (400, 0)],
            dict(x=80, y=0, vx=-8, vy=0, heading=math.pi), speed=8, width=3.5)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, self.chain(p)[0].mode)

    def test_other_safe_lead_cannot_release_a_disappeared_emergency_hazard(self):
        for reverse in (False, True):
            engine = DecisionEngine(self.ds)
            p = perception(speed=3, frame_id=1)
            add_target(p, longitudinal=10, speed=0)
            add_target(p, longitudinal=80, speed=5).id = 2
            if reverse: p.targets.reverse()
            self.assertEqual(DecisionMode.EMERGENCY_BRAKE, engine.run(p).mode)
            p = perception(speed=3, frame_id=2)
            add_target(p, longitudinal=80, speed=5).id = 2
            pending = engine.run(p)
            self.assertEqual(DecisionMode.EMERGENCY_BRAKE, pending.mode)
            self.assertIn('TARGET_CLEAR', pending.reason)
            self.assertEqual(DecisionMode.EMERGENCY_BRAKE, engine.run(p).mode)
            p.frame_id = 3
            released = engine.run(p)
            self.assertEqual(DecisionMode.FOLLOW, released.mode, released.reason)

    def test_same_hazard_resuming_motion_releases_after_its_own_confirmation(self):
        engine = DecisionEngine(self.ds)
        p = perception(speed=0, frame_id=1)
        add_target(p, longitudinal=30, speed=0)
        engine.run(p)
        for frame in (2, 3):
            p.frame_id = frame
            p.targets[0].vx = 5
            d = engine.run(p)
            if frame == 2:
                self.assertIn('STOP_TARGET', d.reason)
                self.assertIn('STOP_TARGET', engine.run(p).reason)
            else:
                self.assertEqual(DecisionMode.FOLLOW, d.mode, d.reason)

    def test_safe_follow_disappearance_itself_never_creates_a_hazard_latch(self):
        engine = DecisionEngine(self.ds)
        p = perception(speed=3, frame_id=1)
        add_target(p, longitudinal=80, speed=5)
        self.assertEqual(DecisionMode.FOLLOW, engine.run(p).mode)
        p.targets, p.frame_id = [], 2
        continued = engine.run(p)
        self.assertIn(continued.mode, (DecisionMode.KEEP_LANE, DecisionMode.FOLLOW))
        self.assertGreater(continued.target_speed, 0.0)
        self.assertFalse(engine._had_target_conflict)
        self.assertFalse(engine._blind_stop)

    def test_geometry_and_target_source_failures_still_protect(self):
        for change in ('source', 'width', 'front'):
            p = perception(speed=3)
            v = add_target(p, longitudinal=80, speed=8)
            v.vy = 1
            ds = self.ds
            if change == 'source': p.source_status['targets']['usable'] = False
            if change == 'width': v.width = 0
            if change == 'front': ds = ds.replace(front_offset_m=None)
            d = DecisionEngine(ds).run(p)
            self.assertEqual(DecisionMode.EMERGENCY_BRAKE, d.mode, (change, d.reason))

    def test_shared_motion_environment_is_parsed_for_decision(self):
        settings = DecisionSettings.from_environment({
            'NEVC_VEHICLE_HALF_WIDTH_M': '.9', 'NEVC_PLANNING_HORIZON_M': '35',
            'NEVC_PLANNING_DECELERATION_MPS2': '1.8',
            'NEVC_PLANNING_LATERAL_GUARD_TIME_S': '4', 'NEVC_PLANNING_LATERAL_MARGIN_M': '.1'})
        self.assertEqual((.9, 35, 1.8, 4, .1),
            (settings.half_width_m, settings.motion_horizon_m, settings.motion_deceleration,
             settings.motion_guard_time_s, settings.motion_lateral_margin_m))

    def test_mapped_object_beyond_coverage_keeps_a_body_safe_boundary_stop(self):
        p = perception(speed=3)
        p.lane.center_line = [(0, 0), (30, 0)]
        target = add_target(p, longitudinal=33, speed=0, in_lane=False, lane_id='b')
        target.lane_width_m = 3.5
        d, t, assessment = self.chain(p)
        self.assertIn('STOP_ROUTE_COVERAGE', d.reason)
        self.assertGreater(d.target_speed, 0)
        self.assertTrue(t.valid, t.reason)
        self.assertFalse(t.emergency_stop)
        self.assertLessEqual(t.points[-1].x+self.ps.front_offset_m, 30)
        self.assertEqual('normal', assessment.mode, assessment.reason)

    def test_beyond_coverage_stop_cannot_release_unknown_overlap_or_infeasible_stop(self):
        for change in ('unmapped', 'current_lane', 'inside_overlap', 'infeasible'):
            p = perception(speed=12 if change == 'infeasible' else 3)
            p.lane.center_line = [(0, 0), (30, 0)]
            target = add_target(p, longitudinal=33, speed=0, in_lane=False, lane_id='b')
            target.lane_width_m = 3.5
            if change == 'unmapped': target.same_lane_valid = False
            if change == 'current_lane': target.lane_id = p.lane.lane_id
            if change == 'inside_overlap': target.x = 30.8
            d, t, assessment = self.chain(p)
            self.assertEqual(DecisionMode.EMERGENCY_BRAKE, d.mode, (change, d.reason))
            self.assertTrue(t.emergency_stop)
            self.assertEqual('emergency_stop', assessment.mode)


if __name__ == '__main__':
    unittest.main()
