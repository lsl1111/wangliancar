"""Signals are scoped independent route constraints, across all driving cases."""
import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from core.interfaces import DecisionMode, DecisionTarget, Trajectory, TrajectoryPoint
from core.safety_supervisor import SafetySupervisor
from core.traffic_quality import signal_stop_requirement
from members.decision.engine import DecisionEngine
from members.decision.settings import DecisionSettings
from members.planning.lane_planner import PlannerSettings, build_trajectory
from perception.perception_builder import PerceptionBuilder
from simone_platform.simone_adapter import SimOneAdapter
from tests.test_decision import perception, add_target
from tests.test_pipeline_integration import Vector
import tests.test_pipeline_integration as integration


def scope(road, low=-2, high=-1):
    return SimpleNamespace(roadId=road, sectionIndex=0, fromLaneId=low,
                           toLaneId=high, stopLineIds=Vector(), crosswalkIds=Vector())


class TrafficConstraintChainTests(unittest.TestCase):
    def setUp(self):
        self.ds = DecisionSettings(front_offset_m=4)
        self.ps = PlannerSettings(front_offset_m=4, half_width_m=.9)

    def frame(self, scene=6):
        p = perception(speed=0)
        p.ego.age_ms, p.ego.frame_id = 0, p.frame_id
        p.scene_id, p.ego.x, p.ego.y, p.ego.heading = scene, 10, 0, 0
        lane = p.lane
        lane.lane_id = '10_0_-2'
        lane.center_line = [(0, 0, 0), (25, 0, 0)]
        lane.forward_reference = lane.center_line + [(35, 0, 0), (200, 0, 0)]
        lane.forward_reference_valid = True
        lane.forward_reference_status = 'map_end'
        lane.forward_lane_ids = ['10_0_-2', '20_0_-2', '30_0_-2']
        lane.forward_lane_spans = [dict(lane_id=identity, start_index=i, end_index=i+1)
                                   for i, identity in enumerate(lane.forward_lane_ids)]
        lane.successor_lane_ids = ['20_0_-2', '21_0_-2']
        return p

    def record(self, x, state=1, identity=101):
        return dict(opendrive_id=identity, status=state, read_ok=state != 0,
                    count_down=10, x=x+5, y=0, stop_line_x=x, stop_line_y=0,
                    stop_line_boundary=[[x, -2, 0], [x, 2, 0]])

    def build(self, p, records):
        adapter = SimpleNamespace(last_traffic_query=dict(association_valid=True),
            read_traffic=lambda identity: records.get(identity, []))
        p.traffic = PerceptionBuilder(adapter, None)._build_traffic(p.ego, p.lane, p.errors)
        p.source_status['traffic'] = dict(association_valid=p.traffic.association_valid,
            usable=p.traffic.valid, quality=p.traffic.reason)
        return p

    def test_sdk_road_association_cannot_override_explicit_lane_validity(self):
        p = self.frame()
        native_lights = Vector([
            SimpleNamespace(id=101, pt=SimpleNamespace(x=31, y=0),
                validities=Vector([scope(10, -2, -1), scope(20, -2, -1)])),
            SimpleNamespace(id=102, pt=SimpleNamespace(x=31, y=3.5),
                validities=Vector([scope(10, -1, -1), scope(21, -1, -1)])),
            SimpleNamespace(id=103, pt=SimpleNamespace(x=34, y=5),
                validities=Vector([scope(30, 1, 2)]))])
        adapter = SimOneAdapter(SimpleNamespace(vehicle_id='0'), SimpleNamespace())
        adapter.map_loaded = True
        stop = SimpleNamespace(pt=SimpleNamespace(x=26, y=0),
            boundaryKnots=Vector([SimpleNamespace(x=26, y=-2, z=0),
                                  SimpleNamespace(x=26, y=2, z=0)]))
        # Like the observed binding, this stand-in returns a line for the
        # physical road even when the light's lane range excludes the caller.
        adapter.hdmap = SimpleNamespace(pySimString=lambda value: value,
            getTrafficLightList=lambda: native_lights, getStoplineList=lambda *args: Vector([stop]))
        adapter.structs = SimpleNamespace(SimOne_Data_TrafficLight=SimpleNamespace)
        def read(vehicle, identity, native):
            native.status, native.countDown = (2 if identity == 101 else 1), 10
            return True
        adapter.sensor_api = SimpleNamespace(SoGetTrafficLights=read)
        records = adapter.read_traffic(p.lane.lane_id)
        self.assertEqual([101], [record['opendrive_id'] for record in records])

    def test_farther_red_is_anticipated_even_when_nearest_signal_is_green(self):
        for scene in (6, 10, 15):
            p = self.build(self.frame(scene), {'10_0_-2': [self.record(26, 2)],
                                              '30_0_-2': [self.record(70, 1, 103)]})
            d = DecisionEngine(self.ds).run(p)
            self.assertIn('STOP_TRAFFIC', d.reason)
            self.assertAlmostEqual(70-10-4-.3, d.stop_distance)
            t = build_trajectory(p, d, self.ps)
            self.assertTrue(t.valid, t.reason)
            self.assertTrue(t.stop_required)

    def test_far_unresolved_line_retains_a_scoped_boundary_and_does_not_hold_here(self):
        record = self.record(70, 0, 103)
        record['stop_line_boundary'] = [[70, 4, 0], [70, 7, 0]]
        p = self.build(self.frame(), {'10_0_-2': [self.record(26, 2)], '30_0_-2': [record]})
        d = DecisionEngine(self.ds).run(p)
        self.assertNotEqual(DecisionMode.STOP, d.mode, d.reason)
        self.assertGreater(d.stop_distance, 0)
        self.assertLessEqual(d.stop_distance, 35-10-4)
        self.assertGreater(d.target_speed, 0)
        self.assertTrue(build_trajectory(p, d, self.ps).valid)

    def test_nearer_red_survives_a_far_unresolved_record(self):
        record = self.record(70, 0, 103)
        record['stop_line_boundary'] = [[70, 4, 0], [70, 7, 0]]
        p = self.build(self.frame(), {'10_0_-2': [self.record(26)], '30_0_-2': [record]})
        d = DecisionEngine(self.ds).run(p)
        self.assertIn('STOP_TRAFFIC', d.reason)
        self.assertAlmostEqual(26-10-4-.3, d.stop_distance)

    def test_planner_independently_enforces_red_when_decision_omits_it(self):
        p = self.build(self.frame(), {'10_0_-2': [self.record(26)]})
        d = DecisionTarget().bind(p)
        d.valid, d.target_speed, d.target_lane_id = True, 5, p.lane.lane_id
        t = build_trajectory(p, d, self.ps)
        self.assertTrue(t.valid, t.reason)
        self.assertTrue(t.stop_required)
        self.assertLessEqual(t.stop_distance, 26-10-4)
        self.assertLess(t.points[-1].x+4, 26)

    def test_planner_does_not_invent_missing_body_extent_to_release_red(self):
        p = self.build(self.frame(), {'10_0_-2': [self.record(26)]})
        d = DecisionEngine(self.ds).run(p)
        t = build_trajectory(p, d, PlannerSettings())
        self.assertFalse(t.valid)
        self.assertIn('vehicle front required', t.reason)

    def test_monitor_rejects_trajectory_crossing_a_known_red(self):
        p = self.build(self.frame(), {'10_0_-2': [self.record(26)]})
        d = DecisionTarget().bind(p)
        d.valid, d.target_speed, d.target_lane_id = True, 5, p.lane.lane_id
        t = Trajectory().bind(d)
        t.valid, t.target_speed, t.target_lane_id = True, 5, p.lane.lane_id
        t.points = [TrajectoryPoint(10, 0, 0, 0, 0), TrajectoryPoint(35, 0, 5, 0, 10)]
        safety = SafetySupervisor(SimpleNamespace(), planning_settings=self.ps,
                                  decision_settings=self.ds).evaluate(p, d, t)
        self.assertTrue(safety.active)
        self.assertEqual(0, safety.control.throttle)
        self.assertGreater(safety.control.brake, 0)

    def test_follow_and_signal_constraints_combine_without_erasing_nearer_target(self):
        p = self.build(self.frame(), {'10_0_-2': [self.record(26, 2)],
                                     '30_0_-2': [self.record(70, 1, 103)]})
        add_target(p, longitudinal=8, speed=0)
        d = DecisionEngine(self.ds).run(p)
        self.assertGreaterEqual(d.stop_distance, 0)
        self.assertLess(d.stop_distance, 70-10-4)

    def test_selected_connector_filters_a_different_turn_without_choosing_by_color(self):
        for selected in ('20_0_-2', '21_0_-2'):
            p = self.frame()
            p.lane.forward_lane_ids[1] = selected
            p.lane.forward_lane_spans[1]['lane_id'] = selected
            straight, turn = self.record(26, 2), self.record(26, 1, 102)
            def detached(road):
                return dict(road_id=road, section_index=0, from_lane_id=-2, to_lane_id=-1)
            straight['signal_validities'] = [detached(10), detached(20)]
            turn['signal_validities'] = [detached(10), detached(21)]
            p = self.build(p, {'10_0_-2': [straight, turn]})
            d = DecisionEngine(self.ds).run(p)
            self.assertEqual(selected == '21_0_-2', 'STOP_TRAFFIC' in d.reason)
            self.assertEqual(1, len(p.traffic.candidates))

    def test_scope_and_boundary_logic_is_invariant_under_world_rotation_and_translation(self):
        for angle in (0, math.pi/2, -1.3, math.pi):
            p = self.frame()
            def transform(x, y):
                return (120+x*math.cos(angle)-y*math.sin(angle),
                        -35+x*math.sin(angle)+y*math.cos(angle), 0)
            p.ego.x, p.ego.y, unused = transform(10, 0)
            p.ego.heading = angle
            p.lane.center_line = [transform(*point[:2]) for point in p.lane.center_line]
            p.lane.forward_reference = [transform(*point[:2]) for point in p.lane.forward_reference]
            records = []
            for status, identity in ((1, 101), (2, 102)):
                record = self.record(26, status, identity)
                record['stop_line_boundary'] = [transform(*point[:2]) for point in record['stop_line_boundary']]
                record['x'], record['y'], unused = transform(31, 0)
                # GREEN faces the other way and must not conflict with RED.
                record['signal_heading'] = angle+math.pi if status == 1 else angle
                records.append(record)
            p = self.build(p, {'10_0_-2': records})
            self.assertEqual('RED', p.traffic.signal_state)
            self.assertEqual(1, len(p.traffic.candidates))
            d = DecisionEngine(self.ds).run(p)
            self.assertAlmostEqual(11.7, d.stop_distance)
            self.assertTrue(build_trajectory(p, d, self.ps).valid)

    def test_partial_future_query_is_bounded_and_current_query_failure_is_not_released(self):
        for failed in ('10_0_-2', '30_0_-2'):
            p = self.frame()
            adapter = SimpleNamespace(last_traffic_query=dict(association_valid=True))
            def read(identity):
                if identity == failed:
                    raise RuntimeError('offline map query unavailable')
                return [self.record(26, 2)] if identity == '10_0_-2' else []
            # With a failed current query, retain observed future signal evidence.
            if failed == '10_0_-2':
                def read(identity):
                    if identity == failed: raise RuntimeError('offline map query unavailable')
                    return [self.record(70, 2)] if identity == '30_0_-2' else []
            adapter.read_traffic = read
            p.traffic = PerceptionBuilder(adapter, None)._build_traffic(p.ego, p.lane, [])
            p.source_status['traffic'] = dict(association_valid=p.traffic.association_valid,
                usable=p.traffic.valid, quality=p.traffic.reason)
            d = DecisionEngine(self.ds).run(p)
            if failed == '30_0_-2':
                self.assertGreater(d.stop_distance, 0)
                self.assertGreater(d.target_speed, 0)
            else:
                self.assertEqual(DecisionMode.STOP, d.mode)

    def test_stale_green_or_malformed_group_cannot_release_stop(self):
        for change in ('stale', 'receipt', 'empty', 'negative_green', 'reason', 'association'):
            p = self.build(self.frame(), {'10_0_-2': [self.record(26, 2)]})
            if change == 'stale': p.source_status['traffic']['quality'] = 'stale'
            if change == 'receipt': p.source_status['traffic']['usable'] = False
            if change == 'association': p.source_status['traffic']['association_valid'] = False
            if change == 'empty': p.traffic.signal_groups = []
            if change == 'negative_green': p.traffic.signal_groups[0]['stop_line_distance'] = -1
            if change == 'reason': p.traffic.signal_groups[0].pop('reason')
            self.assertIsNotNone(signal_stop_requirement(p), change)
            self.assertGreaterEqual(DecisionEngine(self.ds).run(p).stop_distance, 0, change)

    def test_monitor_checks_actual_points_in_addition_to_declared_stop_distance(self):
        p = self.build(self.frame(), {'10_0_-2': [self.record(26)]})
        d = DecisionEngine(self.ds).run(p)
        t = build_trajectory(p, d, self.ps)
        # A producer bug cannot hide a crossing by declaring a safe distance.
        t.points[-1].x = 35
        assessment = SafetySupervisor(SimpleNamespace(), planning_settings=self.ps,
                                      decision_settings=self.ds).evaluate(p, d, t)
        self.assertEqual('traffic_trajectory_crosses_boundary', assessment.reason)

    def test_unknown_scope_keeps_a_known_line_stop_without_claiming_green(self):
        record = self.record(26, 2)
        record['signal_scope_valid'] = False
        p = self.build(self.frame(), {'10_0_-2': [record]})
        self.assertFalse(p.traffic.valid)
        d = DecisionEngine(self.ds).run(p)
        self.assertIn('STOP_TRAFFIC', d.reason)
        self.assertGreater(d.stop_distance, 0)

    def test_full_sdk_chain_ignores_foreign_lamps_anticipates_far_red_and_restarts(self):
        class ScopedSDK(integration.SimulatedSDK):
            def install(self, adapter):
                super(ScopedSDK, self).install(adapter)
                self.lamp_reads = []
                lines = {'1_0_-1': [(0, 0), (39.89, 0)],
                         '2_0_-1': [(39.89, 0), (200, 0)]}
                def sample(identity):
                    center = Vector([SimpleNamespace(x=x, y=y, z=0) for x, y in lines[identity]])
                    return SimpleNamespace(exists=True, laneInfo=SimpleNamespace(
                        centerLine=center, leftBoundary=Vector(), rightBoundary=Vector()))
                def links(identity):
                    return SimpleNamespace(exists=True, laneLink=SimpleNamespace(
                        leftNeighborLaneId='', rightNeighborLaneId='',
                        successorLaneIds=Vector(['2_0_-1'] if identity == '1_0_-1' else []),
                        predecessorLaneIds=Vector(['1_0_-1'] if identity == '2_0_-1' else [])))
                adapter.hdmap.getLaneSample, adapter.hdmap.getLaneLink = sample, links
                adapter.hdmap.getNearMostLane = lambda *args: SimpleNamespace(
                    exists=True, laneId='1_0_-1' if self.x < 39.89 else '2_0_-1')
                lamps = [(52, 1, -1, 40), (60, 2, -1, 65),
                         (99, 1, 1, 40), (100, 2, 1, 65)]
                adapter.hdmap.getTrafficLightList = lambda: Vector([
                    SimpleNamespace(id=identity, pt=SimpleNamespace(x=x+5, y=0),
                        validities=Vector([scope(road, lane, lane)]))
                    for identity, road, lane, x in lamps])
                # Model the binding returning the road's associated line even
                # for an opposite-lane lamp. Explicit scopes must reject it.
                def stop(signal, identity):
                    road = int(identity.split('_')[0])
                    lamp = next(item for item in lamps if item[0] == signal.id)
                    x = lamp[3]
                    return Vector([SimpleNamespace(pt=SimpleNamespace(x=x, y=0),
                        boundaryKnots=Vector([SimpleNamespace(x=x, y=y, z=0)
                                              for y in (-2, 5)]))]) if lamp[1] == road else Vector()
                adapter.hdmap.getStoplineList = stop
                def state(vehicle, identity, native):
                    self.lamp_reads.append(identity)
                    native.status = 2 if identity == 52 or (identity == 60 and self.reads >= 440) else 1
                    native.countDown = 10
                    return True
                adapter.sensor_api.SoGetTrafficLights = state
        with patch('tests.test_pipeline_integration.SimulatedSDK', ScopedSDK), \
                patch.dict('os.environ', {'NEVC_DECISION_CRUISE_SPEED': '6.0',
                    'NEVC_VEHICLE_FRONT_OFFSET_M': '3.9187', 'NEVC_VEHICLE_HALF_WIDTH_M': '.9'}):
            sdk = integration.PipelineIntegrationTests().run_pipeline(
                scene=10, red_light=True, frames=560, initial_offset=0)
        self.assertEqual({52, 60}, set(sdk.lamp_reads))
        held = sdk.positions[420:439]
        self.assertTrue(all(speed < .05 for x, y, speed in held), held[-1])
        self.assertGreater(min(x for x, y, speed in held), 40)
        self.assertLess(max(x+3.9187 for x, y, speed in sdk.positions[:439]), 65)
        self.assertTrue(any('STOP_TRAFFIC:id=60' in p['decision']['reason']
                            for p in sdk.pipelines[:439]))
        self.assertTrue(all(p['trajectory']['valid'] for p in sdk.pipelines))
        self.assertGreater(sdk.x, 65)
        self.assertGreater(sdk.speed, 1)


if __name__ == '__main__':
    unittest.main()
