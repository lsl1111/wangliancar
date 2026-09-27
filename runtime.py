"""Captain-owned process lifecycle and four-member integration pipeline."""

import json
import os
import signal
import time

from core.serialization import perception_to_dict, to_dict
from core.validation import validate_output
from core.interfaces import DecisionTarget, Trajectory, ControlOut
from core.safety_supervisor import SafetySupervisor
from members.control_stub import compute_control
from members.decision_stub import decide
from members.planning_stub import plan
from perception.perception_builder import PerceptionBuilder
from perception.route_manager import RouteManager
from simone_platform.simone_adapter import SimOneAdapter


class CaptainRuntime(object):
    def __init__(self, config, logger):
        self.config = config
        self.logger = logger
        self.adapter = SimOneAdapter(config, logger)
        self.stop_requested = False
        self.pid_path = os.path.join(config.runtime_dir, "captain.pid")
        self.snapshot_path = os.path.join(config.runtime_dir, "latest_perception.json")
        self.pipeline_path = os.path.join(config.runtime_dir, "latest_pipeline.json")
        self._warned_no_control = False
        self.safety = SafetySupervisor(config)

    def request_stop(self, unused_signal=None, unused_frame=None):
        self.stop_requested = True

    def run(self, once=False):
        self._install_signals()
        self._write_pid()
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

    def _loop(self, builder, once):
        period = 1.0 / self.config.loop_hz
        last_frame = None
        processed = 0
        while not self.stop_requested:
            start = time.time()
            status = self.adapter.get_case_status()
            if status == self.adapter.CASE_STOP:
                self.logger.info("案例已停止，队长进程退出")
                break
            if status != self.adapter.CASE_RUNNING:
                time.sleep(0.2)
                continue
            perception = builder.build()
            repeated = last_frame == perception.frame_id
            last_frame = perception.frame_id
            try:
                decision = validate_output(decide(perception), DecisionTarget, perception)
            except Exception as exc:
                decision = DecisionTarget().bind(perception)
                decision.errors.append("DECISION_INVALID:" + type(exc).__name__ + ":" + str(exc))
            try:
                trajectory = validate_output(plan(perception, decision), Trajectory, decision)
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
            safety = self.safety.evaluate(perception, decision, trajectory)
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
                if repeated:
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
                self.logger.info(
                    "frame=%s valid=%s lane=%s targets=%s speed=%.2fm/s",
                    perception.frame_id,
                    perception.valid,
                    perception.lane.lane_id,
                    len(perception.targets),
                    perception.ego.speed,
                )
            if once:
                break
            self._sleep_remaining(start, period)

    def _wait_until_running(self):
        last_status = None
        while not self.stop_requested:
            status = self.adapter.get_case_status()
            if status == self.adapter.CASE_RUNNING:
                return
            if status != last_status:
                self.logger.info("等待案例运行，当前状态=%s", status)
                last_status = status
            time.sleep(0.5)

    def _publish(self, perception):
        temporary = self.snapshot_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(
                perception_to_dict(perception),
                stream,
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
        os.replace(temporary, self.snapshot_path)

    def _publish_pipeline(self, perception, decision, trajectory, control, receipt, safety):
        temporary = self.pipeline_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump({"perception_frame_id": perception.frame_id,
                       "decision": to_dict(decision), "trajectory": to_dict(trajectory),
                       "control": to_dict(control), "safety": {
                           "mode": safety.mode, "reason": safety.reason,
                           "candidate": to_dict(safety.control)}, "send": receipt},
                      stream, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
        os.replace(temporary, self.pipeline_path)

    @staticmethod
    def _sleep_remaining(start, period):
        remaining = period - (time.time() - start)
        if remaining > 0.0:
            time.sleep(remaining)

    def _install_signals(self):
        signal.signal(signal.SIGINT, self.request_stop)
        if hasattr(signal, "SIGTERM"):
            signal.signal(signal.SIGTERM, self.request_stop)

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
