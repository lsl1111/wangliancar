"""Signal fixtures must not become leads or bypass real obstacle protection."""
import copy
import unittest

from core.interfaces import DecisionMode
from core.safety_supervisor import SafetySupervisor
from core.target_semantics import mapped_traffic_light
from members.control.controller import ControlEngine
from members.decision.engine import DecisionEngine
from members.decision.settings import DecisionSettings
from members.planning.lane_planner import PlannerSettings, build_trajectory
from tests.control_benchmark import vehicle
from tests.test_decision import perception, add_target


def signal_observation(state='GREEN'):
    p = perception(speed=0)
    p.scene_id = 10
    p.ego.frame_id, p.ego.age_ms = p.frame_id, 0
    # The lamp is just beyond the known route endpoint, where ordinary unknown
    # objects must still stop the car. Its associated stop line is within it.
    p.lane.center_line = [(float(x), 0.) for x in range(51)]
    p.lane.forward_reference_status = 'ambiguous_successor'
    target = add_target(p, longitudinal=52, same_lane_valid=False, lane_id='')
    target.id, target.type, target.z, target.height = 51, 15, 4., .9
    target.length = target.width = .9
    p.traffic.observed = p.traffic.association_valid = True
    p.traffic.signal_presence, p.traffic.required = 'present', True
    p.traffic.signal_state, p.traffic.signal_id = state, 51
    p.traffic.valid = state != 'UNKNOWN'
    p.traffic.stop_line_distance = 40.
    p.traffic.candidates = [dict(opendrive_id=51, x=52., y=0.,
        stop_distance=40., signal_state=state, read_ok=state != 'UNKNOWN')]
    p.source_status['traffic'] = dict(association_valid=True,
        usable=state != 'UNKNOWN', quality='ok' if state != 'UNKNOWN' else 'unavailable')
    return p


class MappedSignalTargetTests(unittest.TestCase):
    def pipeline(self, p):
        d = DecisionEngine(DecisionSettings(front_offset_m=3.9187)).run(p)
        t = build_trajectory(p, d, PlannerSettings(front_offset_m=3.9187, half_width_m=.9))
        c = ControlEngine(vehicle()).compute(p, t)
        return d, t, c

    def test_green_light_beyond_route_does_not_block_startup(self):
        p = signal_observation()
        d, t, c = self.pipeline(p)
        self.assertEqual(DecisionMode.KEEP_LANE, d.mode, d.reason)
        self.assertGreater(d.target_speed, 0)
        self.assertTrue(t.valid, t.reason)
        self.assertFalse(t.emergency_stop)
        self.assertTrue(c.valid)
        self.assertGreater(c.throttle, 0)
        self.assertEqual(1, len(p.targets))
        self.assertEqual(51, p.targets[0].id)

    def test_red_and_failed_signal_read_still_have_bounded_stop(self):
        for state in ('RED', 'UNKNOWN'):
            p = signal_observation(state)
            d, t, c = self.pipeline(p)
            self.assertGreater(d.target_speed, 0, d.reason)
            self.assertTrue(d.precision_stop)
            self.assertGreater(d.stop_distance, 0)
            self.assertLess(d.stop_distance, 40)
            self.assertTrue(t.valid, t.reason)
            self.assertTrue(t.stop_required)
            self.assertLessEqual(t.stop_distance, d.stop_distance)
            self.assertTrue(c.valid)
            p.traffic.stop_line_distance = 3.9187
            p.traffic.candidates[0]['stop_distance'] = 3.9187
            d, t, c = self.pipeline(p)
            self.assertEqual(DecisionMode.STOP, d.mode, d.reason)
            self.assertEqual(0, c.throttle)
            self.assertGreater(c.brake, 0)

    def test_real_vehicle_or_pole_with_identical_id_is_not_exempt(self):
        for kind in (0, 5, 6, 7, 13, 29):
            p = signal_observation()
            p.targets[0].type = kind
            self.assertFalse(mapped_traffic_light(p.targets[0], p))
            d, t, c = self.pipeline(p)
            self.assertEqual(DecisionMode.STOP, d.mode, d.reason)
            self.assertIn('TARGET_UNKNOWN', d.reason)
            self.assertEqual(0, c.throttle)

    def test_lamp_and_real_lead_are_processed_independently(self):
        p = signal_observation()
        lead = add_target(p, longitudinal=30, speed=2)
        lead.type, lead.id = 6, 52
        d, t, c = self.pipeline(p)
        self.assertEqual(DecisionMode.FOLLOW, d.mode, d.reason)
        self.assertTrue(t.valid, t.reason)
        self.assertGreater(d.target_speed, 0)
        self.assertGreater(t.stop_distance, 0)
        self.assertLess(t.stop_distance, 30)
        self.assertEqual(2, len(p.targets))

    def test_missing_contradictory_or_invalid_evidence_preserves_stop(self):
        base = signal_observation()
        changes = [lambda p: setattr(p.targets[0], 'id', 52),
            lambda p: setattr(p.targets[0], 'y', .6),
            lambda p: setattr(p.targets[0], 'vx', .1),
            lambda p: setattr(p.targets[0], 'vz', .1),
            lambda p: setattr(p.targets[0], 'valid', False),
            lambda p: setattr(p.targets[0], 'z', float('nan')),
            lambda p: setattr(p.targets[0], 'length', -1),
            lambda p: setattr(p.traffic, 'association_valid', False),
            lambda p: p.source_status['traffic'].pop('association_valid'),
            lambda p: setattr(p.traffic, 'candidates', []),
            lambda p: p.traffic.candidates[0].pop('x')]
        for index, change in enumerate(changes):
            p = copy.deepcopy(base)
            change(p)
            self.assertFalse(mapped_traffic_light(p.targets[0], p))
            d, t, c = self.pipeline(p)
            self.assertEqual(0, c.throttle, (index, d.reason))

    def test_diagnostic_or_unusable_target_stream_cannot_supply_exemption(self):
        for source, usable in (('ground_truth', True), ('sensor:test', False)):
            p = signal_observation()
            p.target_source = source
            p.source_status['targets']['usable'] = usable
            self.assertFalse(mapped_traffic_light(p.targets[0], p))

    def test_independent_ttc_guard_keeps_real_obstacles_and_bad_records(self):
        p = signal_observation()
        target = p.targets[0]
        target.lateral_band_match, target.ttc = True, .5
        self.assertFalse(SafetySupervisor._imminent_collision(p))
        target.type = 6
        self.assertTrue(SafetySupervisor._imminent_collision(p))
        target.type, target.valid = 15, False
        self.assertFalse(SafetySupervisor._target_records_valid(p))
        target.valid, target.ttc = True, float('nan')
        self.assertFalse(SafetySupervisor._target_records_valid(p))

    def test_empty_dimensions_need_all_semantic_evidence_to_avoid_planner_fault(self):
        p = signal_observation()
        p.targets[0].length = p.targets[0].width = p.targets[0].height = 0
        d, t, c = self.pipeline(p)
        self.assertTrue(t.valid, t.reason)
        self.assertGreater(c.throttle, 0)
        p.traffic.candidates[0]['opendrive_id'] = 99
        d, t, c = self.pipeline(p)
        self.assertEqual(0, c.throttle)
        self.assertEqual(DecisionMode.STOP, d.mode)
