"""Applicable lamp read failures, bounded approach and green release."""
import unittest
from types import SimpleNamespace

from core.interfaces import DecisionMode
from core.safety_supervisor import SafetySupervisor
from core.serialization import perception_to_dict
from members.decision.engine import DecisionEngine
from members.decision.settings import DecisionSettings
from members.planning.lane_planner import PlannerSettings, build_trajectory
from perception.perception_builder import PerceptionBuilder
from simone_platform.simone_adapter import SimOneAdapter
from tests.test_decision import perception, add_target
from tests.test_perception import FakeAdapter, FakeRouteManager
from tests.test_route_continuation import Vector, NativeId


class SignalQualityMotionTests(unittest.TestCase):
    def setUp(self):
        self.ds = DecisionSettings(front_offset_m=3.9187)
        self.ps = PlannerSettings(front_offset_m=3.9187, half_width_m=.9)

    def signal_frame(self, state='UNKNOWN', usable=False, distance=30, speed=3, frame=1):
        p = perception(speed=speed, frame_id=frame)
        p.scene_id, p.ego.frame_id, p.ego.age_ms = 10, frame, 0
        traffic = p.traffic
        traffic.observed = traffic.required = traffic.association_valid = True
        traffic.signal_presence, traffic.signal_state = 'present', state
        traffic.stop_line_distance, traffic.valid = distance, usable
        p.source_status['traffic'] = dict(association_valid=True, usable=usable,
                                        quality='ok' if usable else 'unavailable')
        return p

    def test_failed_lamp_with_known_line_approaches_then_stops_before_line(self):
        p = self.signal_frame()
        d = DecisionEngine(self.ds).run(p)
        self.assertIn('STOP_TRAFFIC', d.reason)
        self.assertGreater(d.stop_distance, 0)
        t = build_trajectory(p, d, self.ps)
        self.assertTrue(t.valid, t.reason)
        self.assertFalse(t.emergency_stop)
        self.assertLess(t.points[-1].x + self.ds.front_offset_m, 30)
        self.assertEqual(0, t.points[-1].speed)
        self.assertEqual('normal', SafetySupervisor(SimpleNamespace()).evaluate(p, d, t).mode)

    def test_old_green_with_unusable_receipt_is_a_known_line_stop(self):
        p = self.signal_frame(state='GREEN')
        p.traffic.valid = True  # A stale payload may still contain a green.
        d = DecisionEngine(self.ds).run(p)
        self.assertIn('STOP_TRAFFIC', d.reason)
        self.assertGreater(d.stop_distance, 0)
        self.assertTrue(build_trajectory(p, d, self.ps).valid)

    def test_signal_scene_unknown_line_or_unverified_association_stops(self):
        for change in ('line', 'association', 'missing'):
            p = self.signal_frame(speed=3)
            if change == 'line': p.traffic.stop_line_distance = -1
            if change == 'association': p.source_status['traffic']['association_valid'] = False
            if change == 'missing': p.traffic.observed = False
            d = DecisionEngine(self.ds).run(p)
            self.assertEqual(DecisionMode.EMERGENCY_BRAKE, d.mode)

    def test_safety_does_not_accept_unknown_lamp_with_unbounded_or_invalid_stop(self):
        p = self.signal_frame()
        d = DecisionEngine(self.ds).run(p)
        t = build_trajectory(p, d, self.ps)
        for change in ('invalid', 'distance', 'trajectory', 'mode'):
            d.valid, d.mode, d.stop_distance = True, DecisionMode.STOP, 20
            t.stop_required, t.stop_distance = True, 20
            if change == 'invalid': d.valid = False
            if change == 'distance': d.stop_distance = 35
            if change == 'trajectory': t.stop_distance = 35
            if change == 'mode': d.stop_distance = -1
            result = SafetySupervisor(SimpleNamespace()).evaluate(p, d, t)
            self.assertEqual('required_traffic_unavailable', result.reason)

    def test_confirmed_signal_absence_differs_from_unavailable_map_api(self):
        p = perception(speed=0)
        p.scene_id = 10
        self.assertEqual(DecisionMode.STOP, DecisionEngine(self.ds).run(p).mode)
        p.traffic.association_valid, p.traffic.signal_presence = True, 'absent'
        self.assertEqual(DecisionMode.KEEP_LANE, DecisionEngine(self.ds).run(p).mode)

    def test_fresh_green_releases_hold_but_does_not_clear_a_real_nearby_target(self):
        engine = DecisionEngine(self.ds)
        red = self.signal_frame(state='RED', usable=True, distance=4, speed=0, frame=1)
        self.assertEqual(DecisionMode.STOP, engine.run(red).mode)
        decisions = []
        for frame in range(2, 7):
            p = self.signal_frame(state='GREEN', usable=True, distance=4, speed=0, frame=frame)
            add_target(p, longitudinal=30, speed=-5, lane_id='other').y = 8
            decisions.append(engine.run(p))
        self.assertGreater(decisions[-1].target_speed, 0)
        self.assertTrue(build_trajectory(p, decisions[-1], self.ps).valid)
        p.frame_id += 1
        add_target(p, longitudinal=5, speed=0)
        self.assertEqual(DecisionMode.STOP, engine.run(p).mode)

    def test_dynamic_api_failure_keeps_static_candidate_and_never_reuses_green(self):
        adapter = SimOneAdapter(SimpleNamespace(vehicle_id='0'), SimpleNamespace())
        adapter.map_loaded = True
        adapter.hdmap = SimpleNamespace(getTrafficLightList=lambda:Vector(),
                                      getStoplineList=lambda *args:Vector())
        adapter._traffic_candidates['lane'] = [(42, 30, 0, [(25, 0)])]
        adapter.structs = SimpleNamespace(SimOne_Data_TrafficLight=lambda:SimpleNamespace())
        def green(vehicle, identity, native):
            native.status, native.countDown = 2, 5
            return True
        adapter.sensor_api = SimpleNamespace(SoGetTrafficLights=green)
        self.assertEqual(2, adapter.read_traffic('lane')[0]['status'])
        adapter.sensor_api.SoGetTrafficLights = lambda *args:False
        failed = adapter.read_traffic('lane')[0]
        self.assertEqual((0, False, 25), (failed['status'], failed['read_ok'], failed['stop_line_x']))
        self.assertFalse(adapter.last_traffic_query['read_ok'])

    def test_builder_preserves_failed_nearest_line_instead_of_farther_green(self):
        class Lights(FakeAdapter):
            last_traffic_query = {'association_valid': True, 'read_ok': False}
            def read_traffic(self, lane_id):
                return [dict(opendrive_id=1, status=0, read_ok=False,
                             stop_line_x=25, stop_line_y=0),
                        dict(opendrive_id=2, status=2, read_ok=True,
                             stop_line_x=80, stop_line_y=0)]
        p = perception()
        builder = PerceptionBuilder(Lights(), FakeRouteManager())
        traffic = builder._build_traffic(p.ego, FakeRouteManager().update(p.ego), [])
        self.assertEqual('UNKNOWN', traffic.signal_state)
        self.assertFalse(traffic.valid)
        self.assertEqual(25, traffic.stop_line_distance)
        p.traffic = traffic
        snapshot = perception_to_dict(p)
        self.assertTrue(snapshot['traffic']['required'])
        self.assertTrue(snapshot['traffic']['association_valid'])
