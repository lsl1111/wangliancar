"""Captain-owned process lifecycle and four-member integration pipeline."""

import json
import os
import signal
import time

from core.serialization import perception_to_dict, to_dict
from core.validation import validate_output
from core.interfaces import DecisionTarget, Trajectory, ControlOut
from core.safety_supervisor import SafetySupervisor
from members.control_stub import compute_control, configure_control
from members.decision_stub import decide, decision_info, reset_decision
from members.planning_stub import plan
from perception.perception_builder import PerceptionBuilder
from perception.route_manager import RouteManager, ROUTE_REFERENCE_VERSION
from simone_platform.simone_adapter import SimOneAdapter
from simone_platform.sensor_catalog import TARGET_INGESTION_VERSION


def _member_output(value, expected, source):
    # An explicit failure is already unusable. Preserve its diagnostic instead
    # of replacing it with the generic validator's invalid-frame exception.
    if isinstance(value, expected) and value.valid is False:
        return value.bind(source)
    return validate_output(value, expected, source)


class CaptainRuntime(object):
    def __init__(self, config, logger):
        self.config = config
        self.logger = logger
        self.adapter = SimOneAdapter(config, logger)
        self.stop_requested = False
        self.pid_path = os.path.join(config.runtime_dir, "captain.pid")
        self.lock_path = os.path.join(config.runtime_dir, "captain.lock")
        self._lock_stream = None
        self.snapshot_path = os.path.join(config.runtime_dir, "latest_perception.json")
        self.pipeline_path = os.path.join(config.runtime_dir, "latest_pipeline.json")
        self._warned_no_control = False
        self._snapshot_failures = set()
        self.safety = SafetySupervisor(config)
        reset_decision()
        self.runtime_info = decision_info()
        self.runtime_info.update({"pid": os.getpid(),
                                  "target_ingestion_version": TARGET_INGESTION_VERSION,
                                  "route_reference_version": ROUTE_REFERENCE_VERSION,
                                  "started_at": time.strftime("%Y-%m-%d %H:%M:%S")})
        configure_control(config)

    def request_stop(self, unused_signal=None, unused_frame=None):
        self.stop_requested = True

    def run(self, once=False):
        self._install_signals()
        if not self._acquire_instance_lock():
            self.logger.warning("已有队长进程运行，忽略重复启动 pid=%s", os.getpid())
            return
        try:
            self._write_pid()
            self.logger.info(
                "队长启动 pid=%s send_control=%s safety_brake_enabled=%s 日志=%s",
                os.getpid(), self.config.send_control,
                self.config.safety_brake_enabled,
                os.path.join(self.config.runtime_dir, "captain.log"),
            )
            info = decision_info()
            self.logger.info(
                "决策实现 version=%s engine=%s cruise_speed=%.3f m/s (%.1f km/h)",
                info["version"], info["engine"], info["cruise_speed"],
                info["cruise_speed"] * 3.6,
            )
            self.logger.info("路线接续实现 version=%s", ROUTE_REFERENCE_VERSION)
            try:
                self.adapter.bootstrap()
                self.adapter.initialize()
                self.adapter.load_hdmap()
                route_manager = RouteManager(self.adapter, self.logger)
                builder = PerceptionBuilder(
                    self.adapter, route_manager, self.config.scene_id_override
                )
                info = builder.update_case_info()
                self.logger.info(
                    "案例: %s (scene_id=%s)", info.get("case_name", ""), builder.scene_id
                )
                self._wait_until_running()
                self._loop(builder, once)
            finally:
                self.adapter.shutdown()
                self._remove_pid()
        finally:
            self._release_instance_lock()

    def _loop(self, builder, once):
        period = 1.0 / self.config.loop_hz
        last_frame = None
        last_route_signature = None
        processed = 0
        while not self.stop_requested:
            start = time.time()
            if processed == 0:
                self.logger.info("首帧检查: 查询案例状态")
            status = self.adapter.get_case_status()
            if status == self.adapter.CASE_STOP:
                self.logger.info("案例已停止，队长进程退出")
                break
            if status != self.adapter.CASE_RUNNING:
                time.sleep(0.2)
                continue
            if processed == 0:
                self.logger.info("首帧检查: 案例运行中，开始读取感知")
            perception = builder.build()
            lane = perception.lane
            route_signature = (lane.lane_id, tuple(lane.forward_lane_ids),
                               lane.forward_reference_valid, lane.forward_reference_status)
            if route_signature != last_route_signature:
                self.logger.info("前方参考 lane=%s lanes=%s valid=%s status=%s points=%s",
                                 lane.lane_id, ",".join(lane.forward_lane_ids),
                                 lane.forward_reference_valid, lane.forward_reference_status,
                                 len(lane.forward_reference))
                last_route_signature = route_signature
            if processed == 0:
                self.logger.info("首帧检查: 感知读取完成 frame=%s valid=%s",
                                 perception.frame_id, perception.valid)
            repeated = last_frame == perception.frame_id
            last_frame = perception.frame_id
            try:
                decision = _member_output(decide(perception), DecisionTarget, perception)
            except Exception as exc:
                decision = DecisionTarget().bind(perception)
                decision.errors.append("DECISION_INVALID:" + type(exc).__name__ + ":" + str(exc))
            try:
                trajectory = _member_output(plan(perception, decision), Trajectory, decision)
            except Exception as exc:
                trajectory = Trajectory().bind(decision)
                trajectory.errors.append("TRAJECTORY_INVALID:" + type(exc).__name__ + ":" + str(exc))
            try:
                control = compute_control(perception, trajectory)
                if control.valid:
                    control = validate_output(control, ControlOut, trajectory)
            except Exception as exc:
                control = ControlOut().bind(trajectory)
                control.errors.append("CONTROL_INVALID:" + type(exc).__name__ + ":" + str(exc))
            safety = self.safety.evaluate(perception, decision, trajectory, control)
            receipt = {"attempted": False, "ok": False, "reason": "observe_mode",
                       "safety_mode": safety.mode, "safety_reason": safety.reason,
                       "safety_candidate": safety.control is not None}
            if safety.active:
                if (self.config.send_control and
                        getattr(self.config, "safety_brake_enabled", False) and
                        safety.control is not None):
                    receipt["attempted"] = True
                    receipt["ok"] = bool(self.adapter.send_control(safety.control))
                    receipt["reason"] = ("safety_sent" if receipt["ok"]
                                         else "safety_send_failed")
                    receipt["adapter"] = to_dict(getattr(self.adapter, "last_send_result", {}))
                elif safety.control is None:
                    receipt["reason"] = "safety_command_unavailable"
                else:
                    receipt["reason"] = "safety_observe_only"
            elif self.config.send_control:
                if not getattr(self.config, "safety_brake_enabled", False):
                    receipt["reason"] = "safety_brake_not_armed"
                elif repeated:
                    receipt["reason"] = "gps_frame_repeated"
                elif decision.valid and trajectory.valid and control.valid:
                    receipt["attempted"] = True
                    receipt["ok"] = bool(self.adapter.send_control(control))
                    receipt["reason"] = "sent" if receipt["ok"] else "send_failed"
                    receipt["adapter"] = to_dict(getattr(self.adapter, "last_send_result", {}))
                    if not receipt["ok"]:
                        self.logger.warning("控制指令发送失败 frame=%s receipt=%s",
                                            perception.frame_id, receipt["adapter"])
                elif not self._warned_no_control:
                    receipt["reason"] = "pipeline_invalid"
                    self.logger.warning("数据链或控制输出无效，不发送控制 frame=%s", perception.frame_id)
                    self._warned_no_control = True
                else:
                    receipt["reason"] = "pipeline_invalid"
            if self.config.publish_json:
                self._publish(perception)
                self._publish_pipeline(perception, decision, trajectory, control, receipt,
                                       safety)
            processed += 1
            if processed == 1 or processed % 100 == 0:
                target_status = perception.source_status.get("targets", {})
                targets_usable = (perception.targets_valid
                                  and isinstance(perception.target_source, str)
                                  and perception.target_source.startswith("sensor:")
                                  and target_status.get("usable", False))
                self.logger.info(
                    "frame=%s valid=%s lane=%s targets=%s target_source=%s targets_usable=%s "
                    "target_reason=%s speed=%.2fm/s decision=%s mode=%s decision_reason=%s "
                    "trajectory=%s control=%s send=%s reason=%s",
                    perception.frame_id,
                    perception.valid,
                    perception.lane.lane_id,
                    len(perception.targets),
                    perception.target_source, targets_usable,
                    target_status.get("reason", "unknown"),
                    perception.ego.speed,
                    decision.valid, decision.mode, decision.reason,
                    trajectory.valid, control.valid, receipt["reason"],
                    trajectory.reason if not trajectory.valid else "; ".join(control.errors),
                )
            if once:
                break
            self._sleep_remaining(start, period)

    def _wait_until_running(self):
        last_status = None
        self.logger.info("等待案例运行: 开始查询 SDK 状态")
        while not self.stop_requested:
            status = self.adapter.get_case_status()
            if status == self.adapter.CASE_RUNNING:
                self.logger.info("案例状态已运行，进入感知/控制循环")
                return
            if status != last_status:
                self.logger.info("等待案例运行，当前状态=%s", status)
                last_status = status
            time.sleep(0.5)

    def _publish(self, perception):
        return self._publish_json(self.snapshot_path, perception_to_dict(perception))

    def _publish_pipeline(self, perception, decision, trajectory, control, receipt, safety):
        return self._publish_json(self.pipeline_path, {
            "runtime": dict(self.runtime_info),
            "perception_frame_id": perception.frame_id,
            "route_reference": {
                "valid": perception.lane.forward_reference_valid,
                "lane_ids": list(perception.lane.forward_lane_ids),
                "status": perception.lane.forward_reference_status,
                "point_count": len(perception.lane.forward_reference)},
            "map_observations": {
                "parking_count": len(perception.parking_spaces),
                "stop_line_count": len(perception.map_stop_lines),
                "crosswalk_count": len(perception.map_crosswalks),
                "speed_limits": to_dict(perception.speed_limit_observations),
                "status": to_dict(perception.map_observation_status)},
            "decision": to_dict(decision), "trajectory": to_dict(trajectory),
            "control": to_dict(control), "safety": {
                "mode": safety.mode, "reason": safety.reason,
                "candidate": to_dict(safety.control)}, "send": receipt})

    def _publish_json(self, path, data):
        # Snapshots are diagnostics, not the member-to-member transport. On
        # Windows an open reader can temporarily prevent atomic replacement.
        # Never let this optional I/O failure terminate the driving loop.
        try:
            temporary = path + ".tmp"
            with open(temporary, "w", encoding="utf-8") as stream:
                json.dump(data, stream, ensure_ascii=False, indent=2,
                          sort_keys=True, allow_nan=False)
            os.replace(temporary, path)
        except OSError as exc:
            if path not in self._snapshot_failures:
                self.logger.warning(
                    "诊断快照写入失败，本帧跳过该快照，控制循环继续: %s: %s",
                    path, exc)
            self._snapshot_failures.add(path)
            return False
        if path in self._snapshot_failures:
            self._snapshot_failures.remove(path)
            self.logger.info("诊断快照写入已恢复: %s", path)
        return True

    @staticmethod
    def _sleep_remaining(start, period):
        remaining = period - (time.time() - start)
        if remaining > 0.0:
            time.sleep(remaining)

    def _install_signals(self):
        signal.signal(signal.SIGINT, self.request_stop)
        if hasattr(signal, "SIGTERM"):
            signal.signal(signal.SIGTERM, self.request_stop)

    def _acquire_instance_lock(self):
        if not os.path.isdir(self.config.runtime_dir):
            os.makedirs(self.config.runtime_dir)
        stream = open(self.lock_path, "a+b")
        try:
            if os.path.getsize(self.lock_path) == 0:
                stream.write(b"\0")
                stream.flush()
            stream.seek(0)
            if os.name == "nt":
                import msvcrt
                msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            stream.close()
            return False
        self._lock_stream = stream
        return True

    def _release_instance_lock(self):
        if self._lock_stream is None:
            return
        try:
            self._lock_stream.seek(0)
            if os.name == "nt":
                import msvcrt
                msvcrt.locking(self._lock_stream.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                import fcntl
                fcntl.flock(self._lock_stream.fileno(), fcntl.LOCK_UN)
        finally:
            self._lock_stream.close()
            self._lock_stream = None

    def _write_pid(self):
        if not os.path.isdir(self.config.runtime_dir):
            os.makedirs(self.config.runtime_dir)
        with open(self.pid_path, "w", encoding="ascii") as stream:
            stream.write(str(os.getpid()))

    def _remove_pid(self):
        if os.path.exists(self.pid_path):
            try:
                os.remove(self.pid_path)
            except OSError:
                pass
