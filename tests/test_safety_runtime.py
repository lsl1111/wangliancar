import json
import os
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from core.config import load_config
from core.interfaces import (ControlOut, DecisionMode, DecisionTarget, Perception, Trajectory,
                             TrajectoryPoint)
from runtime import CaptainRuntime


def perception(frame=10):
    p = Perception()
    p.valid, p.frame_id, p.timestamp = True, frame, frame * 1000
    p.valid_until = time.monotonic() + 10
    p.case_id, p.case_name = "case-06", "06.车道居中控制"
    p.scene_id = 6
    p.ego.valid, p.ego.frame_id, p.ego.gear = True, frame, 1
    p.ego.age_ms = 0
    p.targets_valid, p.lane.valid = True, True
    p.target_source = "sensor:perfectPerception1"
    p.source_status = {"gps": {"usable": True}, "targets": {"usable": True}}
    return p


def decision(p):
    d = DecisionTarget().bind(p)
    d.valid = True
    return d


def trajectory(p, d):
    t = Trajectory().bind(d)
    t.valid = True
    t.points = [TrajectoryPoint(0, 0, 1, 0, 0),
                TrajectoryPoint(1, 0, 1, 0, 1)]
    return t


def controller(p, t):
    c = ControlOut().bind(t)
    c.throttle, c.valid = 0.2, True
    return c


class FakeAdapter(object):
    CASE_STOP, CASE_RUNNING = 1, 2

    def __init__(self, count):
        self.statuses = [self.CASE_RUNNING] * count + [self.CASE_STOP]
        self.sent = []
        self.last_send_result = {}

    def get_case_status(self):
        return self.statuses.pop(0)

    def send_control(self, control):
        self.sent.append((control.frame_id, control.throttle, control.brake,
                          control.source))
        return True


class SafetyRuntimeTests(unittest.TestCase):
    def run_frames(self, frames, armed=True, send_control=True, decision_fn=decision,
                   trajectory_fn=trajectory, control_fn=controller):
        with tempfile.TemporaryDirectory() as directory:
            config = SimpleNamespace(runtime_dir=directory, loop_hz=20,
                                     send_control=send_control, safety_brake_enabled=armed,
                                     publish_json=True, pipeline_timeout_ms=200,
                                     sensor_timeout_ms=500)
            runtime = CaptainRuntime(config, SimpleNamespace(
                info=lambda *args: None, warning=lambda *args: None))
            runtime.adapter = FakeAdapter(len(frames))
            iterator = iter(frames)
            with patch("runtime.decide", side_effect=decision_fn), \
                 patch("runtime.plan", side_effect=trajectory_fn), \
                 patch("runtime.compute_control", side_effect=control_fn), \
                 patch.object(runtime, "_sleep_remaining", return_value=None):
                runtime._loop(SimpleNamespace(build=lambda: next(iterator)), False)
            with open(os.path.join(directory, "latest_pipeline.json"),
                      encoding="utf-8") as stream:
                pipeline = json.load(stream)
            return runtime.adapter.sent, pipeline

    def test_missing_target_suppresses_normal_throttle_when_unarmed(self):
        p = perception()
        p.scene_id = 1
        p.targets_valid = False
        sent, pipeline = self.run_frames([p], armed=False)
        self.assertEqual([], sent)
        self.assertEqual("controlled_stop", pipeline["safety"]["mode"])
        self.assertEqual("safety_observe_only", pipeline["send"]["reason"])
        self.assertEqual(0.3, pipeline["safety"]["candidate"]["brake"])

    def test_default_configuration_and_observe_mode_never_send(self):
        project = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config = load_config(project)
        self.assertFalse(config.send_control)
        self.assertFalse(config.safety_brake_enabled)
        p = perception()
        p.scene_id = 1
        p.targets_valid = False
        sent, pipeline = self.run_frames([p], armed=True, send_control=False)
        self.assertEqual([], sent)
        self.assertEqual("safety_observe_only", pipeline["send"]["reason"])

    def test_armed_target_loss_sends_brake_instead_of_normal_throttle(self):
        p = perception()
        p.scene_id = 1
        p.targets_valid = False
        sent, pipeline = self.run_frames([p], armed=True)
        self.assertEqual(1, len(sent))
        self.assertEqual((0.0, 0.3), sent[0][1:3])
        self.assertEqual("safety_sent", pipeline["send"]["reason"])

    def test_optional_sensor_error_does_not_override_normal_control(self):
        p = perception()
        p.targets_valid = False
        p.source_status["targets"] = {"usable": False,
                                       "sensor_presence": "not_configured"}
        p.sensor_errors = ["imu:timing_unknown"]
        sent, pipeline = self.run_frames([p])
        self.assertEqual(0.2, sent[0][1])
        self.assertEqual("normal", pipeline["safety"]["mode"])

    def test_normal_control_is_gated_by_armed_safety_exit(self):
        sent, pipeline = self.run_frames([perception()], armed=False)
        self.assertEqual([], sent)
        self.assertEqual("safety_brake_not_armed", pipeline["send"]["reason"])

    def test_one_repeated_frame_does_not_trigger_controller_fault_brake(self):
        calls = [0]
        def once(p, t):
            calls[0] += 1
            return controller(p, t) if calls[0] == 1 else ControlOut().bind(t)
        p = perception()
        sent, pipeline = self.run_frames([p, p], control_fn=once)
        self.assertEqual(1, len(sent))
        self.assertEqual("normal", pipeline["safety"]["mode"])
        self.assertEqual("gps_frame_repeated", pipeline["send"]["reason"])

    def test_emergency_intent_overrides_normal_throttle(self):
        def emergency(p):
            d = decision(p)
            d.mode, d.target_speed = DecisionMode.EMERGENCY_BRAKE, 0.0
            return d
        sent, pipeline = self.run_frames([perception()], decision_fn=emergency)
        self.assertEqual((0.0, 1.0), sent[0][1:3])
        self.assertEqual("emergency_stop", pipeline["safety"]["mode"])

    def test_invalid_trajectory_uses_separate_fault_exit(self):
        def invalid_plan(p, d):
            return Trajectory().bind(d)
        sent, pipeline = self.run_frames([perception()], trajectory_fn=invalid_plan)
        self.assertEqual((0.0, 1.0), sent[0][1:3])
        self.assertEqual("trajectory_invalid_or_expired", pipeline["safety"]["reason"])

    def test_invalid_controller_output_uses_fault_brake_when_armed(self):
        def invalid_control(p, t):
            return ControlOut().bind(t)
        sent, pipeline = self.run_frames([perception()], control_fn=invalid_control)
        self.assertEqual((0.0, 1.0), sent[0][1:3])
        self.assertEqual("controller_invalid_or_expired", pipeline["safety"]["reason"])

    def test_gps_failure_uses_only_recent_trusted_header(self):
        first, second = perception(10), perception(11)
        second.valid, second.ego.valid = False, False
        sent, pipeline = self.run_frames([first, second])
        self.assertEqual(2, len(sent))
        self.assertEqual((10, 0.0, 1.0), sent[-1][:3])
        self.assertEqual("gps_invalid_or_expired", pipeline["safety"]["reason"])

    def test_stalled_frame_may_send_fault_brake_with_explicit_arming(self):
        p = perception()
        clock = time.monotonic()
        with tempfile.TemporaryDirectory() as directory:
            config = SimpleNamespace(runtime_dir=directory, loop_hz=20,
                                     send_control=True, safety_brake_enabled=True,
                                     publish_json=True, pipeline_timeout_ms=200,
                                     sensor_timeout_ms=500)
            runtime = CaptainRuntime(config, SimpleNamespace(
                info=lambda *args: None, warning=lambda *args: None))
            runtime.adapter = FakeAdapter(2)
            times = iter((clock, clock + 0.25))
            runtime.safety.clock = lambda: next(times)
            with patch("runtime.decide", side_effect=decision), \
                 patch("runtime.plan", side_effect=trajectory), \
                 patch("runtime.compute_control", side_effect=controller), \
                 patch.object(runtime, "_sleep_remaining", return_value=None):
                runtime._loop(SimpleNamespace(build=lambda: p), False)
            self.assertEqual(2, len(runtime.adapter.sent))
            self.assertEqual((0.0, 1.0), runtime.adapter.sent[-1][1:3])
            with open(os.path.join(directory, "latest_pipeline.json"),
                      encoding="utf-8") as stream:
                pipeline = json.load(stream)
            self.assertEqual("gps_frame_stalled", pipeline["safety"]["reason"])


if __name__ == "__main__":
    unittest.main()
