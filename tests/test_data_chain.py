import json
import math
import os
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from core.config import load_config
from core.interfaces import ControlOut, DecisionMode, DecisionTarget, LaneContext, Trajectory
from core.serialization import perception_to_dict
from members.decision_stub import decide, reset_decision
from members.decision.settings import DecisionSettings
from members.control_stub import compute_control, configure_control
from members.planning_stub import plan
from perception.perception_builder import PerceptionBuilder
from perception.route_manager import RouteManager, _orient_forward
from runtime import CaptainRuntime
from simone_platform.simone_adapter import SimOneAdapter


class EmptyTraffic(object):
    config = SimpleNamespace(sensor_timeout_ms=500, max_sensor_frame_gap=2,
                             pipeline_timeout_ms=200)

    def get_case_info(self):
        return {"case_name": "06.车道居中控制-测试"}

    def read_traffic(self, lane_id):
        return []


class StraightLane(object):
    def update(self, ego):
        lane = LaneContext()
        lane.valid = True
        lane.lane_id = "road1"
        lane.center_line = [(float(x), 0.0, 0.0) for x in range(201)]
        lane.lane_width = 3.5
        lane.lane_width_valid = True
        return lane


class DataChainTests(unittest.TestCase):
    def setUp(self):
        # These contract tests exercise an explicitly configured low-speed run.
        reset_decision(DecisionSettings(cruise_speed=2.0,
                                        front_offset_m=3.5))
        self.adapter = EmptyTraffic()
        self.builder = PerceptionBuilder(self.adapter, StraightLane())
        self.builder.update_case_info()

    def raw(self, frame=10, x=0.0):
        return {"gps": {"frame": frame, "timestamp": 1000, "x": x,
                        "y": 0.0, "heading": 0.0, "vx": 5.0,
                        "vy": 0.0, "gear": 1, "age_ms": 0},
                "targets": [], "targets_valid": True,
                "targets_frame": frame, "targets_timestamp": 1000,
                "targets_age_ms": 0}

    def test_invalid_gps_is_isolated_and_stale_frame_fails(self):
        for heading in (float("inf"), float("nan")):
            raw = self.raw()
            raw["gps"]["heading"] = heading
            value = self.builder.build_from_raw(raw)
            self.assertFalse(value.valid)
            json.dumps(perception_to_dict(value), allow_nan=False)
        raw = self.raw()
        raw["gps"]["age_ms"] = 60000
        self.assertFalse(self.builder.build_from_raw(raw).valid)
        raw["gps"]["age_ms"] = -2
        self.assertFalse(self.builder.build_from_raw(raw).valid)

    def test_bad_target_and_radar_cannot_masquerade_as_empty(self):
        raw = self.raw()
        raw["targets"] = [{"id": 1, "x": 12, "y": 0,
                           "vz": float("nan"), "probability": 1.0}]
        raw["radar_detections"] = [{"sensor_id": "radar1", "range": float("inf")}]
        raw["radar_status"] = {"radar1": True}
        raw["source_status"] = {"radar:radar1": {"read_ok": True,
                                  "frame_id": 10, "age_ms": 0}}
        value = self.builder.build_from_raw(raw)
        self.assertEqual([], value.targets)
        self.assertFalse(value.targets_valid)
        self.assertFalse(value.radar_status["radar1"])
        self.assertEqual("invalid", value.source_status["radar:radar1"]["quality"])
        json.dumps(perception_to_dict(value), allow_nan=False)

    def test_malformed_source_metadata_is_marked_invalid(self):
        raw = self.raw()
        raw["source_status"] = {"gps": "bad", "targets": "bad"}
        raw["sensor_errors"] = None
        value = self.builder.build_from_raw(raw)
        self.assertFalse(value.source_status["gps"]["usable"])
        self.assertFalse(value.targets_valid)
        self.assertIn("SENSOR_ERRORS_INVALID", value.sensor_errors)

    def test_empty_target_frame_is_valid_only_when_fresh(self):
        fresh = self.builder.build_from_raw(self.raw())
        self.assertTrue(fresh.targets_valid)
        self.assertEqual([], fresh.targets)
        self.assertEqual("ok", fresh.source_status["targets"]["quality"])
        raw = self.raw()
        raw["targets_age_ms"] = 600
        self.assertFalse(self.builder.build_from_raw(raw).targets_valid)

    def test_scene_six_decision_ignores_unavailable_targets(self):
        raw = self.raw()
        raw["targets_valid"] = False
        raw["targets"] = [{"id": 7, "x": 1, "y": 0, "probability": 1.0}]
        raw["source_status"] = {"targets": {"read_ok": False,
                                                "sensor_presence": "not_configured"}}
        value = self.builder.build_from_raw(raw)
        self.assertEqual(6, value.scene_id)
        self.assertNotIn("TARGETS_UNAVAILABLE", value.errors)
        self.assertEqual("not_configured", value.source_status["targets"]["quality"])
        self.assertEqual(DecisionMode.KEEP_LANE, decide(value).mode)
        value.scene_id = 1
        decide(value)
        value.frame_id += 1
        value.timestamp += 1
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, decide(value).mode)
        raw = self.raw()
        raw["targets_frame"] = 5
        self.assertFalse(self.builder.build_from_raw(raw).targets_valid)

    def test_headerless_imu_retains_values_with_unknown_timing(self):
        raw = self.raw()
        raw["imu"], raw["imu_valid"] = {"vx": 2.0, "roll_rate": 0.1}, True
        raw["source_status"] = {"imu": {"read_ok": True,
                                        "frame_id": -1, "age_ms": -1}}
        value = self.builder.build_from_raw(raw)
        self.assertTrue(value.imu_valid)
        self.assertEqual("timing_unknown", value.source_status["imu"]["quality"])
        self.assertFalse(value.source_status["imu"]["usable"])

    def test_ambiguous_stopline_remains_visible_and_not_green(self):
        class MixedLights(EmptyTraffic):
            def read_traffic(self, lane_id):
                return [{"status": 2, "opendrive_id": 1, "stop_line_x": 40.0,
                         "stop_line_y": 0.0, "count_down": 10},
                        {"status": 1, "opendrive_id": 2, "stop_line_x": 40.1,
                         "stop_line_y": 0.0, "count_down": 10}]
        builder = PerceptionBuilder(MixedLights(), StraightLane())
        builder.update_case_info()
        value = builder.build_from_raw(self.raw())
        self.assertTrue(value.traffic.observed)
        self.assertTrue(value.traffic.ambiguous)
        self.assertFalse(value.traffic.valid)
        self.assertEqual(2, len(value.traffic.candidates))
        decision = decide(value)
        self.assertEqual(DecisionMode.KEEP_LANE, decision.mode)
        self.assertGreater(decision.stop_distance, 0.0)

    def test_reference_starts_ahead_and_emergency_is_transmitted(self):
        p = self.builder.build_from_raw(self.raw(x=150))
        d = decide(p)
        trajectory = plan(p, d)
        self.assertTrue(trajectory.valid, trajectory.reason)
        self.assertGreaterEqual(min(item.x for item in trajectory.points), 150)
        self.assertEqual(p.frame_id, trajectory.frame_id)
        d.mode, d.target_speed, d.stop_distance = DecisionMode.EMERGENCY_BRAKE, 0.0, 0.0
        emergency = plan(p, d)
        self.assertTrue(emergency.valid)
        self.assertTrue(emergency.emergency_stop)
        self.assertEqual(0.0, emergency.target_speed)
        self.assertTrue(all(point.speed == 0.0 for point in emergency.points))

    def test_scene_six_reaches_control_diagnostics_without_target_sensor(self):
        raw = self.raw(x=50)
        raw["gps"]["vx"] = 0.0
        raw["targets_valid"] = False
        raw["source_status"] = {"targets": {"read_ok": False,
                                                "sensor_presence": "not_configured"}}
        p = self.builder.build_from_raw(raw)
        d = decide(p)
        self.assertEqual(DecisionMode.KEEP_LANE, d.mode)
        self.assertEqual(2.0, d.target_speed)
        t = plan(p, d)
        self.assertTrue(t.valid, t.errors)
        self.assertEqual(2.0, t.target_speed)
        self.assertEqual(0.0, t.points[0].speed)
        self.assertGreater(t.points[1].speed, 0.0)
        configure_control(SimpleNamespace(control_calibrated=False))
        control = compute_control(p, t)
        self.assertFalse(control.valid)
        self.assertIn("reference_speed_mps", control.diagnostics)
        self.assertIn("vehicle calibration required", control.errors)
        project = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config = load_config(project)
        self.assertFalse(config.send_control)
        configure_control(config)
        trial = compute_control(p, t)
        self.assertTrue(trial.valid, trial.errors)
        self.assertGreater(trial.throttle, 0.0)

    def test_shared_low_speed_target_requires_scene_inputs(self):
        raw = self.raw(x=50)
        raw["gps"]["vx"] = 0.0
        p = self.builder.build_from_raw(raw)
        p.scene_id = 4
        self.assertEqual(2.0, decide(p).target_speed)
        p.scene_id = 1
        decide(p)
        p.frame_id += 1
        p.timestamp += 1
        self.assertEqual(DecisionMode.STOP, decide(p).mode)
        p.target_source = "sensor:perfectPerception1"
        p.source_status["targets"]["usable"] = True
        for unused in range(3):
            p.frame_id += 1
            p.timestamp += 1
            decision = decide(p)
        self.assertEqual(DecisionMode.KEEP_LANE, decision.mode)
        self.assertEqual(2.0, decision.target_speed)

    def test_scene_six_launch_yields_to_required_stop(self):
        raw = self.raw(x=50)
        raw["gps"]["vx"] = 0.0
        p = self.builder.build_from_raw(raw)
        p.traffic.observed = True
        p.traffic.signal_state = "RED"
        p.traffic.stop_line_distance = 20.0
        decision = decide(p)
        self.assertEqual(DecisionMode.KEEP_LANE, decision.mode)
        self.assertEqual(2.0, decision.target_speed)
        self.assertAlmostEqual(13.5, decision.stop_distance)
        p.traffic.observed = False
        p.lane.valid = False
        decision = decide(p)
        self.assertEqual(DecisionMode.STOP, decision.mode)
        self.assertEqual(0.0, decision.target_speed)

    def test_curved_lane_uses_local_direction_and_reverses_boundaries(self):
        points = [(0, 0, 0), (10, 0, 0), (10, 10, 0), (-10, 10, 0)]
        self.assertEqual(points, _orient_forward(points, 0.0, 5, 0))

        class Vector(list):
            def Size(self):
                return len(self)
            def GetElement(self, i):
                return self[i]
        def line(y):
            return Vector([SimpleNamespace(x=x, y=y, z=0.0) for x in (0, 10)])
        map_api = SimpleNamespace(
            pySimPoint3D=lambda *args: args,
            getNearMostLane=lambda pos: SimpleNamespace(exists=True, laneId="road1"),
            getLaneSample=lambda lane: SimpleNamespace(exists=True, laneInfo=SimpleNamespace(
                centerLine=line(0), leftBoundary=line(2), rightBoundary=line(-2))),
            getLaneLink=lambda lane: SimpleNamespace(exists=True, laneLink=SimpleNamespace(
                leftNeighborLaneId="left", rightNeighborLaneId="right")),
            getLaneWidth=lambda lane, pos: SimpleNamespace(exists=True, width=4.0),
            getRoadMark=lambda pos, lane: SimpleNamespace(exists=True,
                left=SimpleNamespace(type="solid"), right=SimpleNamespace(type="broken")))
        route = RouteManager(SimpleNamespace(map_loaded=True, hdmap=map_api),
                             SimpleNamespace(warning=lambda *args: None))
        ego = PerceptionBuilder._build_ego(self.raw()["gps"])
        ego.x, ego.heading = 5.0, math.pi
        lane = route.update(ego, force=True)
        self.assertTrue(lane.valid)
        self.assertEqual((10.0, -2.0, 0.0), lane.left_boundary[0])
        self.assertEqual("right", lane.left_lane_id)
        self.assertEqual("broken", lane.left_mark_type)
        target = SimpleNamespace(x=6.0, y=0.0, z=0.0, height=1.5)
        self.assertEqual(("road1", True), route.locate_target(target))
        target.y = 5.0
        self.assertEqual(("", False), route.locate_target(target))

    def test_bad_or_expired_control_never_reaches_sdk(self):
        adapter = SimOneAdapter(SimpleNamespace(vehicle_id="0", sensor_id="x"), None)
        calls = []
        adapter.structs = SimpleNamespace(SimOne_Data_Control=lambda: SimpleNamespace(),
                                          SimOne_Data_Signal_Lights=lambda: SimpleNamespace())
        adapter.pnc_api = SimpleNamespace(
            SoSetDriveMode=lambda *args: calls.append("mode") or True,
            SoSetDrive=lambda *args: calls.append("drive") or True,
            SoSetSignalLights=lambda *args: calls.append("lights") or False)
        command = ControlOut()
        command.valid, command.frame_id, command.timestamp = True, 10, 1000
        command.valid_until = time.monotonic() + 1
        command.throttle = float("nan")
        self.assertFalse(adapter.send_control(command))
        self.assertEqual([], calls)
        command = ControlOut()
        command.valid, command.frame_id, command.timestamp = True, 10, 1000
        command.valid_until = time.monotonic() + 1
        command.throttle, command.brake = 0.8, 0.3
        self.assertFalse(adapter.send_control(command))
        self.assertEqual(0.0, command.throttle)
        self.assertEqual(["mode", "drive", "lights"], calls)
        self.assertFalse(adapter.last_send_result["signals_ok"])
        command.valid_until = time.monotonic() - 1
        calls[:] = []
        self.assertFalse(adapter.send_control(command))
        self.assertEqual([], calls)

    def test_duplicate_frame_updates_pipeline_but_is_not_resent(self):
        p = self.builder.build_from_raw(self.raw(x=50))
        statuses = [2, 2, 1]
        sends = []
        class MockAdapter(object):
            CASE_STOP, CASE_RUNNING = 1, 2
            last_send_result = {}
            def get_case_status(self):
                return statuses.pop(0)
            def send_control(self, control):
                sends.append(control.frame_id)
                return True
        with tempfile.TemporaryDirectory() as directory:
            config = SimpleNamespace(runtime_dir=directory, loop_hz=20,
                                     send_control=True, safety_brake_enabled=True,
                                     publish_json=True)
            runtime = CaptainRuntime(config, SimpleNamespace(
                info=lambda *args: None, warning=lambda *args: None))
            runtime.adapter = MockAdapter()
            observed = []
            def controller(perception, trajectory):
                result = ControlOut().bind(trajectory)
                result.valid, result.brake = True, 0.2
                return result
            with patch("runtime.compute_control", side_effect=controller), \
                 patch.object(runtime, "_sleep_remaining", return_value=None), \
                 patch.object(runtime, "_publish_pipeline", wraps=runtime._publish_pipeline) as publish:
                runtime._loop(SimpleNamespace(build=lambda: observed.append(p) or p), False)
            self.assertEqual(2, len(observed))
            self.assertEqual(2, publish.call_count)
            self.assertEqual([10], sends)
            with open(os.path.join(directory, "latest_pipeline.json"), encoding="utf-8") as stream:
                data = json.load(stream)
            self.assertEqual("gps_frame_repeated", data["send"]["reason"])


if __name__ == "__main__":
    unittest.main()
