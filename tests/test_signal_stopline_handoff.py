"""Real stop-line geometry at a verified lane join must reach the stop chain."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from core.interfaces import DecisionMode, LaneContext
from members.decision.engine import DecisionEngine
from members.decision.settings import DecisionSettings
from members.planning.lane_planner import PlannerSettings, build_trajectory
from perception.perception_builder import PerceptionBuilder
from simone_platform.simone_adapter import SimOneAdapter
from tests.test_decision import perception
from tests import test_pipeline_integration as integration

Vector = integration.Vector


class SignalStoplineHandoffTests(unittest.TestCase):
    def lane(self):
        lane = LaneContext()
        lane.valid, lane.lane_id = True, '290_0_-2'
        # Coordinates from the failed scene-10 run; the stop line is 0.1 m
        # beyond the current native lane, inside its verified successor.
        lane.center_line = [(-192.82575753348667, 2.2812667392832706, 0),
                            (-19.541306500954317, 1.8436797417262065, 0)]
        lane.forward_reference = lane.center_line + [(16.5, 1.75, 0), (200, 1.3, 0)]
        lane.forward_reference_valid = True
        lane.forward_lane_ids = ['290_0_-2', '409_0_-2', '342_0_-2']
        lane.forward_lane_spans = [dict(lane_id=identity, start_index=i, end_index=i+1)
                                   for i, identity in enumerate(lane.forward_lane_ids)]
        lane.forward_reference_status = 'map_end'
        lane.speed_limit = 8.333333333333334
        return lane

    def record(self, status=1, identity=52):
        knots = [[-19.428049286317805, 7.093410477995584, 0],
                 [-19.44572599763336, .09343279703993801, 0]]
        return dict(opendrive_id=identity, status=status, read_ok=status != 0,
                    x=18.1640625, y=-1.453125, stop_line_x=-19.43688764197558,
                    stop_line_y=3.593421637517761, stop_line_boundary=knots)

    def build(self, records=None, lane=None, x=-103.25558, speed=0):
        p = perception(speed=speed)
        p.scene_id, p.ego.x, p.ego.y = 10, x, 1.93699
        p.lane = lane or self.lane()
        adapter = SimpleNamespace(read_traffic=lambda identity: records or [self.record()],
                                  last_traffic_query=dict(association_valid=True))
        errors = []
        p.traffic = PerceptionBuilder(adapter, None)._build_traffic(p.ego, p.lane, errors)
        p.source_status['traffic'] = dict(usable=p.traffic.valid,
            association_valid=p.traffic.association_valid, quality=p.traffic.reason)
        return p, errors

    def test_captured_stop_line_reaches_decision_and_planning(self):
        p, errors = self.build()
        self.assertTrue(p.traffic.valid, (p.traffic.reason, errors))
        self.assertEqual('RED', p.traffic.signal_state)
        self.assertEqual('present', p.traffic.signal_presence)
        self.assertAlmostEqual(83.81, p.traffic.stop_line_distance, delta=.03)
        d = DecisionEngine(DecisionSettings(front_offset_m=3.9187)).run(p)
        self.assertIn('STOP_TRAFFIC', d.reason)
        t = build_trajectory(p, d, PlannerSettings(front_offset_m=3.9187, half_width_m=.9))
        self.assertTrue(t.valid, t.reason)
        self.assertTrue(t.stop_required)
        self.assertLess(t.points[-1].x + 3.9187, -19.45)

    def test_last_cruising_frame_requires_braking_instead_of_absence(self):
        p, unused = self.build(x=-23.5495662689209, speed=6.730787280695725)
        self.assertTrue(p.traffic.required)
        self.assertLess(p.traffic.stop_line_distance, 4.2)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE,
                         DecisionEngine(DecisionSettings(front_offset_m=3.9187)).run(p).mode)

    def test_transverse_boundary_intersection_wins_over_off_lane_midpoint(self):
        record = self.record()
        record['stop_line_boundary'] = [[-19.44, -1, 0], [-19.44, 19, 0]]
        record['stop_line_y'] = 9
        p, unused = self.build([record])
        self.assertTrue(p.traffic.valid, p.traffic.reason)
        self.assertAlmostEqual(83.816, p.traffic.stop_line_distance, delta=.03)

    def test_unverified_or_inconsistent_successor_never_proves_absence(self):
        for change in ('flag', 'prefix', 'spans'):
            lane = self.lane()
            if change == 'flag': lane.forward_reference_valid = False
            if change == 'prefix': lane.forward_reference[0] = (-190, 0, 0)
            if change == 'spans': lane.forward_lane_spans[1]['start_index'] = 0
            p, unused = self.build(lane=lane)
            self.assertTrue(p.traffic.required, change)
            self.assertNotEqual('absent', p.traffic.signal_presence, change)
            self.assertFalse(p.traffic.valid, change)
            self.assertEqual(DecisionMode.STOP, DecisionEngine(
                DecisionSettings(front_offset_m=3.9187)).run(p).mode, change)

    def test_failed_geometry_before_farther_green_cannot_be_discarded(self):
        broken, green = self.record(), self.record(2, 53)
        broken['stop_line_boundary'] = [[-19.44, 8, 0], [-19.44, 10, 0]]
        green.update(stop_line_x=25, stop_line_y=1.7,
                     stop_line_boundary=[[25, -1, 0], [25, 5, 0]])
        p, unused = self.build([broken, green])
        self.assertTrue(p.traffic.required)
        self.assertFalse(p.traffic.valid)
        self.assertEqual('UNKNOWN', p.traffic.signal_state)

    def test_conflicting_signals_and_failed_read_keep_known_line_stop(self):
        for records in ([self.record(1, 51), self.record(2, 52)], [self.record(0)]):
            p, unused = self.build(records)
            self.assertFalse(p.traffic.valid)
            self.assertTrue(p.traffic.required)
            self.assertGreater(p.traffic.stop_line_distance, 80)
            self.assertIn('STOP_TRAFFIC', DecisionEngine(
                DecisionSettings(front_offset_m=3.9187)).run(p).reason)

    def test_unanimous_associated_lamps_release_only_on_fresh_green(self):
        for status in (1, 2):
            p, unused = self.build([self.record(status, 51), self.record(status, 52)])
            self.assertTrue(p.traffic.valid, p.traffic.reason)
            self.assertEqual(-1, p.traffic.signal_id)
            d = DecisionEngine(DecisionSettings(front_offset_m=3.9187)).run(p)
            self.assertEqual(status == 1, 'STOP_TRAFFIC' in d.reason)
        p, unused = self.build([self.record(2, 51), self.record(0, 52)])
        self.assertFalse(p.traffic.valid)
        self.assertIn('STOP_TRAFFIC', DecisionEngine(
            DecisionSettings(front_offset_m=3.9187)).run(p).reason)

    def test_bad_boundary_height_or_multiple_crossings_stays_unknown(self):
        for boundary in ([[-19.44, -1, 8], [-19.44, 5, 8]],
                         [[-19.44, -1, 0], [float('nan'), 5, 0]]):
            record = self.record()
            record['stop_line_boundary'] = boundary
            p, unused = self.build([record])
            self.assertTrue(p.traffic.required)
            self.assertNotEqual('absent', p.traffic.signal_presence)
            self.assertFalse(p.traffic.valid)
        lane = self.lane()
        lane.forward_reference = lane.center_line + [(20, 1.7, 0), (-30, 3.5, 0)]
        p, unused = self.build(lane=lane)
        self.assertTrue(p.traffic.required)
        self.assertFalse(p.traffic.valid)

    def test_missing_association_api_is_unknown_without_inventing_a_signal(self):
        p = perception()
        adapter = SimpleNamespace(read_traffic=lambda identity: [],
                                  last_traffic_query=dict(association_valid=False))
        p.traffic = PerceptionBuilder(adapter, None)._build_traffic(p.ego, p.lane, [])
        self.assertEqual('unknown', p.traffic.signal_presence)
        self.assertFalse(p.traffic.required)
        p.scene_id = 6
        self.assertEqual(DecisionMode.KEEP_LANE,
                         DecisionEngine(DecisionSettings(front_offset_m=3.9187)).run(p).mode)
        p.scene_id = 10
        self.assertEqual(DecisionMode.STOP,
                         DecisionEngine(DecisionSettings(front_offset_m=3.9187)).run(p).mode)

    def test_signal_associated_only_with_selected_successor_is_queried(self):
        lane, queried = self.lane(), []
        def read(identity):
            queried.append(identity)
            return [self.record()] if identity == '409_0_-2' else []
        adapter = SimpleNamespace(read_traffic=read,
                                  last_traffic_query=dict(association_valid=True))
        p = perception()
        p.ego.x, p.ego.y = -103, 1.9
        traffic = PerceptionBuilder(adapter, None)._build_traffic(p.ego, lane, [])
        self.assertEqual(lane.forward_lane_ids, queried)
        self.assertEqual('RED', traffic.signal_state)

    def test_passed_supported_line_can_be_absent_but_off_route_line_cannot(self):
        record = self.record()
        record.update(stop_line_x=-110, stop_line_y=2,
                      stop_line_boundary=[[-110, -1, 0], [-110, 5, 0]])
        p, unused = self.build([record])
        self.assertEqual('absent', p.traffic.signal_presence)
        record = self.record()
        record['stop_line_boundary'] = [[201, -1, 0], [201, 5, 0]]
        record.update(stop_line_x=201, stop_line_y=2)
        p, unused = self.build([record])
        self.assertTrue(p.traffic.required)
        self.assertNotEqual('absent', p.traffic.signal_presence)

    def test_adapter_preserves_complete_boundary_instead_of_only_average(self):
        record = self.record()
        stop = SimpleNamespace(id=463, pt=SimpleNamespace(x=-19.43, y=7.09),
            boundaryKnots=Vector([SimpleNamespace(x=x, y=y, z=z)
                                  for x, y, z in record['stop_line_boundary']]))
        adapter = SimOneAdapter(SimpleNamespace(vehicle_id='0'), SimpleNamespace())
        adapter.map_loaded = True
        adapter.hdmap = SimpleNamespace(pySimString=lambda value: value,
            getTrafficLightList=lambda: Vector([SimpleNamespace(id=52,
                pt=SimpleNamespace(x=record['x'], y=record['y']))]),
            getStoplineList=lambda *args: Vector([stop]))
        adapter.structs = SimpleNamespace(SimOne_Data_TrafficLight=SimpleNamespace)
        def red(vehicle, identity, native):
            native.status, native.countDown = 1, 10
            return True
        adapter.sensor_api = SimpleNamespace(SoGetTrafficLights=red)
        observed = adapter.read_traffic('290_0_-2')[0]
        self.assertEqual(record['stop_line_boundary'], observed['stop_line_boundary'])

    def test_complete_sdk_chain_stops_at_join_holds_then_restarts_on_green(self):
        class JunctionSDK(integration.SimulatedSDK):
            def install(self, adapter):
                super(JunctionSDK, self).install(adapter)
                lines = {'1_0_-1': [(0, 0), (39.89, 0)],
                         '2_0_-1': [(39.89, 0), (80, 0)],
                         '3_0_-1': [(80, 0), (200, 0)]}
                def sample(identity):
                    center = Vector([SimpleNamespace(x=x, y=y, z=0) for x, y in lines[identity]])
                    return SimpleNamespace(exists=True, laneInfo=SimpleNamespace(
                        centerLine=center, leftBoundary=Vector(), rightBoundary=Vector()))
                def links(identity):
                    ids = list(lines)
                    index = ids.index(identity)
                    return SimpleNamespace(exists=True, laneLink=SimpleNamespace(
                        leftNeighborLaneId='', rightNeighborLaneId='',
                        successorLaneIds=Vector(ids[index+1:index+2]),
                        predecessorLaneIds=Vector(ids[max(0, index-1):index])))
                adapter.hdmap.getLaneSample, adapter.hdmap.getLaneLink = sample, links
                adapter.hdmap.getNearMostLane = lambda *args: SimpleNamespace(
                    exists=True, laneId='1_0_-1' if self.x < 39.89 else '2_0_-1')
                stop = SimpleNamespace(pt=SimpleNamespace(x=40, y=5), boundaryKnots=Vector([
                    SimpleNamespace(x=40, y=5, z=0), SimpleNamespace(x=40, y=-2, z=0)]))
                adapter.hdmap.getStoplineList = lambda signal, identity: (
                    Vector([stop]) if identity == '1_0_-1' else Vector())
                adapter.hdmap.getTrafficLightList = lambda: Vector([
                    SimpleNamespace(id=identity, pt=SimpleNamespace(x=45, y=y),
                        validities=Vector([SimpleNamespace(roadId=1, sectionIndex=0,
                                                           fromLaneId=-1, toLaneId=-1)]))
                    for identity, y in ((51, 1), (52, -1))])
                def state(vehicle, identity, native):
                    native.status, native.countDown = (1 if self.reads < 440 else 2), 10
                    return True
                adapter.sensor_api.SoGetTrafficLights = state
        with patch('tests.test_pipeline_integration.SimulatedSDK', JunctionSDK), \
                patch.dict('os.environ', {'NEVC_DECISION_CRUISE_SPEED': '6.0',
                    'NEVC_VEHICLE_FRONT_OFFSET_M': '3.9187', 'NEVC_VEHICLE_HALF_WIDTH_M': '.9'}):
            sdk = integration.PipelineIntegrationTests().run_pipeline(
                scene=10, red_light=True, frames=560, initial_offset=0)
        held = sdk.positions[420:439]
        self.assertTrue(all(speed < .05 for x, y, speed in held), held[-1])
        self.assertLess(max(x + 3.9187 for x, y, speed
                            in sdk.positions[:439]), 40)
        self.assertTrue(any('STOP_TRAFFIC' in p['decision']['reason'] for p in sdk.pipelines[:439]))
        self.assertGreater(sdk.speed, 1)
        self.assertGreater(sdk.x, 40)


if __name__ == '__main__':
    unittest.main()
