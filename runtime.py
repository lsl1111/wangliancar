"""Captain-owned process lifecycle and four-member integration pipeline."""

import json
import os
import signal
import time

from core.serialization import perception_to_dict
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
        self._warned_no_control = False

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
            if last_frame == perception.frame_id and not once:
                self._sleep_remaining(start, period)
                continue
            last_frame = perception.frame_id
            decision = decide(perception)
            trajectory = plan(perception, decision)
            control = compute_control(perception, trajectory)
            if self.config.send_control:
                if control.valid:
                    if not self.adapter.send_control(control):
                        self.logger.warning("控制指令发送失败 frame=%s", perception.frame_id)
                elif not self._warned_no_control:
                    self.logger.warning("控制模块仍为占位实现，因此不会向车辆发送控制")
                    self._warned_no_control = True
            if self.config.publish_json:
                self._publish(perception)
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
            )
        os.replace(temporary, self.snapshot_path)

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

