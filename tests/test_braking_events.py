"""Capture a one-frame brake even when the next pipeline snapshot is clear."""
import glob
import json
import os
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from core.interfaces import DecisionMode, Target
from core.safety_supervisor import SafetyAssessment
from runtime import CaptainRuntime
from tests.test_safety_runtime import FakeAdapter, perception, decision, trajectory, controller


class BrakingEventTests(unittest.TestCase):
    def runtime(self, directory):
        config = SimpleNamespace(runtime_dir=directory, loop_hz=20, send_control=True,
                                 safety_brake_enabled=True, publish_json=True,
                                 pipeline_timeout_ms=200, sensor_timeout_ms=500)
        return CaptainRuntime(config, SimpleNamespace(info=lambda *args: None,
                                                       warning=lambda *args: None))

    def test_transient_plan_failure_keeps_full_target_frame_after_recovery(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = self.runtime(directory)
            frames = [perception(i) for i in range(10, 16)]
            for p in frames:
                p.lane.center_line = [(0, 0), (100, 0)]
                target = Target()
                target.id, target.x, target.valid = 11646222, 78, True
                target.lane_id, target.lane_width_m = 'successor', 3.97
                p.targets = [target]
            def follow(p):
                d = decision(p)
                d.mode, d.target_speed, d.reason = DecisionMode.FOLLOW, 8.33, 'FOLLOW_TARGET'
                return d
            def plan(p, d):
                t = trajectory(p, d)
                if p.frame_id == 11:
                    t.valid = False
                    t.reason = 'crossing or oncoming obstacle motion unsupported'
                return t
            runtime.adapter = FakeAdapter(len(frames))
            iterator = iter(frames)
            with patch('runtime.decide', side_effect=follow), \
                 patch('runtime.plan', side_effect=plan), \
                 patch('runtime.compute_control', side_effect=controller), \
                 patch.object(runtime, '_sleep_remaining', return_value=None):
                runtime._loop(SimpleNamespace(build=lambda: next(iterator)), False)
            files = glob.glob(os.path.join(directory, 'braking_events', '*.json'))
            events = []
            for path in files:
                with open(path, encoding='utf-8') as stream:
                    events.append(json.load(stream))
            onset = next(e for e in events if e['event'] == 'brake_start')
            self.assertEqual(11, onset['perception']['frame_id'])
            self.assertEqual(DecisionMode.FOLLOW, onset['decision']['mode'])
            self.assertEqual(3.97, onset['perception']['targets'][0]['lane_width_m'])
            self.assertEqual([[0, 0], [100, 0]], onset['perception']['lane']['center_line'])
            self.assertEqual(10, onset['preceding_frames'][-1]['frame_id'])
            self.assertEqual('safety_sent', onset['send']['reason'])
            self.assertTrue(any(e['event'] == 'brake_release' for e in events))
            with open(runtime.pipeline_path, encoding='utf-8') as stream:
                self.assertTrue(json.load(stream)['trajectory']['valid'])
            self.assertEqual(0.2, runtime.adapter.sent[-1][1])

    def test_controller_brake_and_release_are_saved_without_safety_override(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = self.runtime(directory)
            for frame, brake in ((1, 0), (2, 0.2), (3, 0)):
                p = perception(frame)
                d, t = decision(p), trajectory(p, decision(p))
                c = controller(p, t)
                c.brake, c.throttle, c.source = brake, 0 if brake else 0.2, 'control:TRACK'
                c.diagnostics['reference_speed_mps'] = 0 if brake else 2.0
                runtime._trace_braking(p, d, t, c,
                                       {'reason': 'sent', 'attempted': True, 'ok': True},
                                       SafetyAssessment())
            files = sorted(glob.glob(os.path.join(directory, 'braking_events', '*.json')))
            self.assertEqual(2, len(files))
            with open(files[0], encoding='utf-8') as stream:
                event = json.load(stream)
            self.assertEqual('normal', event['safety']['mode'])
            self.assertEqual(0.2, event['control']['brake'])
            self.assertEqual(0, event['control']['diagnostics']['reference_speed_mps'])

    def test_diagnostic_write_failure_does_not_change_command_and_capture_is_bounded(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = self.runtime(directory)
            p = perception()
            d, t = decision(p), trajectory(p, decision(p))
            c = controller(p, t)
            c.brake, c.throttle = 0.2, 0
            receipt = {'reason': 'sent', 'attempted': True, 'ok': True}
            with patch.object(runtime, '_publish_json', side_effect=PermissionError('busy')):
                runtime._trace_braking(p, d, t, c, receipt, SafetyAssessment())
            self.assertEqual((0.2, 0, True), (c.brake, c.throttle, receipt['ok']))
            with patch.object(runtime, '_publish_json', return_value=True) as publish:
                for i in range(160):
                    c.brake = 0.2 if i % 2 else 0
                    runtime._trace_braking(p, d, t, c, receipt, SafetyAssessment())
                self.assertLessEqual(publish.call_count, 79)
            self.assertEqual(8, len(runtime._braking_history))

    def events(self, directory):
        records = []
        for path in sorted(glob.glob(os.path.join(directory, 'braking_events', '*.json'))):
            with open(path, encoding='utf-8') as stream:
                records.append(json.load(stream))
        return records

    def test_repeated_frame_and_failed_send_cannot_release_last_sent_brake(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = self.runtime(directory)
            frames = [perception(i) for i in (10, 10, 11, 12)]
            for p in frames:
                p.ego.brake = 0.4
            calls = [0]
            def compute(p, t):
                calls[0] += 1
                c = controller(p, t)
                if calls[0] == 1:
                    c.brake, c.throttle = 0.4, 0
                elif calls[0] == 2:
                    c.valid = False
                return c
            class Adapter(FakeAdapter):
                def send_control(self, command):
                    super(Adapter, self).send_control(command)
                    return command.frame_id != 11
            runtime.adapter = Adapter(len(frames))
            iterator = iter(frames)
            with patch('runtime.decide', side_effect=decision), \
                 patch('runtime.plan', side_effect=trajectory), \
                 patch('runtime.compute_control', side_effect=compute), \
                 patch.object(runtime, '_sleep_remaining', return_value=None):
                runtime._loop(SimpleNamespace(build=lambda: next(iterator)), False)
            events = self.events(directory)
            self.assertEqual(['brake_start', 'brake_command_unavailable',
                              'brake_release_requested', 'brake_release'],
                             [e['event'] for e in events])
            self.assertEqual([10, 11, 12], [c[0] for c in runtime.adapter.sent])
            self.assertEqual('gps_frame_repeated', events[1]['send']['reason'])
            self.assertIsNone(events[1]['braking']['candidate_braking'])
            self.assertEqual(10, events[2]['braking']['last_successful_command']['frame_id'])
            self.assertTrue(events[2]['braking']['last_successful_command']['braking'])
            self.assertFalse(events[2]['braking']['release_sent'])
            self.assertEqual(12, events[3]['braking']['last_successful_command']['frame_id'])
            self.assertTrue(events[3]['braking']['release_sent'])
            # Sending a release does not claim the car already executed it.
            self.assertEqual(0.4, events[3]['braking']['observed_brake'])
            with open(runtime.pipeline_path, encoding='utf-8') as stream:
                self.assertEqual(events[-1]['braking'], json.load(stream)['braking'])

    def test_observe_mode_clear_and_failed_brake_are_never_confirmed_releases(self):
        for first_receipt in ({'reason': 'observe_mode', 'attempted': False, 'ok': False},
                              {'reason': 'send_failed', 'attempted': True, 'ok': False}):
            with self.subTest(first_receipt=first_receipt):
                with tempfile.TemporaryDirectory() as directory:
                    runtime = self.runtime(directory)
                    p = perception()
                    d, t = decision(p), trajectory(p, decision(p))
                    c = controller(p, t)
                    c.brake, c.throttle = 0.3, 0
                    runtime._trace_braking(p, d, t, c, first_receipt, SafetyAssessment())
                    self.assertIsNone(runtime._braking_state['last_successful_command'])
                    c.brake, c.throttle = 0, 0.2
                    runtime._trace_braking(p, d, t, c,
                                           {'reason': 'sent', 'attempted': True, 'ok': True},
                                           SafetyAssessment())
                    self.assertEqual(['brake_start', 'brake_release_requested'],
                                     [e['event'] for e in self.events(directory)])

    def test_dominant_gps_fault_does_not_fill_capture_with_unused_control_errors(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = self.runtime(directory)
            p = perception()
            p.valid, p.ego.valid = False, False
            d, t = decision(p), trajectory(p, decision(p))
            t.valid = False
            c = controller(p, t)
            c.valid = False
            safety = SafetyAssessment('fault_stop', 'gps_invalid_or_expired')
            receipt = {'reason': 'safety_command_unavailable', 'attempted': False, 'ok': False}
            with patch('runtime.time.monotonic', return_value=100.0):
                for i in range(150):
                    c.errors = ['ego perception invalid or expired' if i % 2 else
                                'control loop dt outside bound']
                    runtime._trace_braking(p, d, t, c, receipt, safety)
                self.assertEqual(1, len(self.events(directory)))
                safety.reason = 'required_targets_unavailable'
                runtime._trace_braking(p, d, t, c, receipt, safety)
            events = self.events(directory)
            self.assertEqual(['brake_start', 'brake_change'], [e['event'] for e in events])
            self.assertEqual(c.errors, events[-1]['control']['errors'])
            self.assertEqual(8, len(events[-1]['preceding_frames']))
            self.assertFalse(events[-1]['braking']['observed_valid'])
            self.assertIsNone(events[-1]['braking']['candidate_braking'])

    def test_slow_diagnostic_flush_preserves_component_clocks_and_original_deadline(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = self.runtime(directory)
            p = perception()
            p.valid_until = 101.0
            clock = [100.0]
            def step_decision(p):
                clock[0] += 0.01
                return decision(p)
            def step_plan(p, d):
                clock[0] += 0.01
                return trajectory(p, d)
            def step_control(p, t):
                clock[0] += 0.01
                c = controller(p, t)
                c.brake, c.throttle = 0.2, 0
                return c
            def slow_flush():
                clock[0] = 105.0
            runtime.adapter = FakeAdapter(1)
            with patch('runtime.time.monotonic', side_effect=lambda: clock[0]), \
                 patch('runtime.decide', side_effect=step_decision), \
                 patch('runtime.plan', side_effect=step_plan), \
                 patch('runtime.compute_control', side_effect=step_control), \
                 patch.object(runtime, '_flush_evaluation', side_effect=slow_flush):
                runtime._loop(SimpleNamespace(build=lambda: p), True)
            event = self.events(directory)[0]
            timing = event['pipeline_timing']
            self.assertEqual(100.0, timing['decision_started_monotonic'])
            self.assertAlmostEqual(100.01, timing['planning_started_monotonic'])
            self.assertAlmostEqual(100.02, timing['control_started_monotonic'])
            self.assertAlmostEqual(100.03, timing['safety_started_monotonic'])
            self.assertAlmostEqual(100.03, timing['send_finished_monotonic'])
            self.assertEqual(105.0, event['recorded_monotonic'])
            self.assertEqual(101.0, event['perception']['valid_until'])
            self.assertEqual(1, len(runtime.adapter.sent))
            with open(runtime.pipeline_path, encoding='utf-8') as stream:
                self.assertEqual(timing, json.load(stream)['pipeline_timing'])
