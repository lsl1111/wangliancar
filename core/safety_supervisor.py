"""Bounded safety overrides for the captain's perception-to-control chain.

The supervisor is not a path tracker. It never commands propulsion, and a
candidate brake command is not proof that the simulator executed it.
"""

import math
import time

from core.interfaces import ControlOut, DecisionMode
from core.scene_requirements import requires_targets
from core.validation import current


class SafetyAssessment(object):
    def __init__(self, mode="normal", reason="", control=None):
        self.mode = mode
        self.reason = reason
        self.control = control

    @property
    def active(self):
        return self.mode != "normal"


class SafetySupervisor(object):
    """Classify missing inputs and prepare a brake-only candidate.

    A previous GPS header can only be reused for a short, explicit interval.
    SimOne may reject repeated headers; the sender must report its receipt.
    """

    def __init__(self, config, clock=None):
        self.clock = clock or time.monotonic
        self.repeat_fault_ms = max(1, int(getattr(config, "pipeline_timeout_ms", 200)))
        self.last_gps_ms = max(1, int(getattr(config, "sensor_timeout_ms", 500)))
        self.recovery_frames = max(1, int(getattr(config, "safety_recovery_frames", 3)))
        self.controlled_brake = self._brake(getattr(config, "safety_controlled_brake", 0.3))
        self.emergency_brake = self._brake(getattr(config, "safety_emergency_brake", 1.0))
        self.normal_control_required = bool(getattr(config, "send_control", False))
        self.last_case = None
        self.last_frame = None
        self.last_frame_at = None
        self.last_good_header = None
        self.last_good_at = None
        self.recovering = False
        self.good_frames = 0

    @staticmethod
    def _brake(value):
        value = float(value)
        if not math.isfinite(value) or not 0.0 < value <= 1.0:
            raise ValueError("safety brake must be finite and in (0, 1]")
        return value

    def reset(self, require_recovery=False):
        self.last_frame = None
        self.last_frame_at = None
        self.last_good_header = None
        self.last_good_at = None
        self.recovering = require_recovery
        self.good_frames = 0

    @staticmethod
    def _imminent_collision(perception):
        """Use only a fresh target stream for an independent brake override."""
        statuses = getattr(perception, "source_status", {})
        target_status = statuses.get("targets", {}) if isinstance(statuses, dict) else {}
        if (not getattr(perception, "targets_valid", False)
                or not getattr(perception, "target_source", "").startswith("sensor:")
                or not isinstance(target_status, dict)
                or target_status.get("usable") is not True):
            return False
        for target in getattr(perception, "targets", []):
            relevant = (getattr(target, "same_lane", False)
                        if getattr(target, "same_lane_valid", False)
                        else getattr(target, "lateral_band_match", False))
            distance = getattr(target, "longitudinal_distance", None)
            ttc = getattr(target, "ttc", None)
            if (getattr(target, "valid", False) and relevant
                    and type(distance) in (int, float) and math.isfinite(distance)
                    and distance > 0 and type(ttc) in (int, float)
                    and math.isfinite(ttc) and 0 <= ttc <= 2.0):
                return True
        return False

    @staticmethod
    def _target_records_valid(perception):
        for target in getattr(perception, "targets", []):
            distance = getattr(target, "longitudinal_distance", None)
            ttc = getattr(target, "ttc", None)
            if (not getattr(target, "valid", False)
                    or type(distance) not in (int, float) or not math.isfinite(distance)
                    or type(ttc) not in (int, float) or not math.isfinite(ttc)):
                return False
        return True

    def evaluate(self, perception, decision, trajectory, control=None):
        now = self.clock()
        case = (getattr(perception, "case_id", ""), getattr(perception, "case_name", ""))
        if self.last_case is not None and case != self.last_case:
            self.reset(require_recovery=True)
        self.last_case = case
        frame = getattr(perception, "frame_id", -1)
        new_frame = frame != self.last_frame
        if new_frame:
            self.last_frame, self.last_frame_at = frame, now
        repeat_ms = ((now - self.last_frame_at) * 1000.0
                     if self.last_frame_at is not None else 0.0)
        source_status = getattr(perception, "source_status", {})
        if not isinstance(source_status, dict):
            source_status = {}
        gps_status = source_status.get("gps", {})
        gps_ok = (current(perception) and getattr(perception.ego, "valid", False)
                  and isinstance(gps_status, dict) and gps_status.get("usable") is True)
        if (gps_ok and new_frame and 0 <= perception.ego.age_ms < self.repeat_fault_ms):
            self.last_good_header = (perception.frame_id, perception.timestamp,
                                     perception.ego.gear)
            self.last_good_at = now

        mode, reason = "normal", ""
        if not gps_ok:
            mode, reason = "fault_stop", "gps_invalid_or_expired"
        elif (getattr(perception.ego, "age_ms", -1) >= self.repeat_fault_ms
              or repeat_ms >= self.repeat_fault_ms):
            mode, reason = "fault_stop", "gps_frame_stalled"
        elif self._imminent_collision(perception):
            mode, reason = "emergency_stop", "imminent_target_collision"
        elif not getattr(perception, "lane", None) or not perception.lane.valid:
            mode, reason = "controlled_stop", "lane_unavailable"
        elif (requires_targets(getattr(perception, "scene_id", 0))
              and (not getattr(perception, "targets_valid", False)
                   or not isinstance(source_status.get("targets"), dict)
                   or source_status["targets"].get("usable") is not True
                   or not getattr(perception, "target_source", "").startswith("sensor:")
                   or not self._target_records_valid(perception))):
            mode, reason = "controlled_stop", "required_targets_unavailable"
        elif (current(decision, perception)
              and decision.mode == DecisionMode.EMERGENCY_BRAKE):
            mode, reason = "emergency_stop", "collision_or_emergency_request"
        elif not current(trajectory, decision):
            mode, reason = "fault_stop", "trajectory_invalid_or_expired"
        elif trajectory.emergency_stop:
            mode, reason = "emergency_stop", "trajectory_emergency_request"
        elif (self.normal_control_required and new_frame and control is not None
              and not current(control, trajectory)):
            mode, reason = "fault_stop", "controller_invalid_or_expired"
        # Independent IMU/radar/ultrasonic/environment errors are deliberately
        # absent from these gates. Only channels needed for this baseline stop it.
        if mode != "normal":
            self.recovering, self.good_frames = True, 0
        elif self.recovering:
            if new_frame:
                self.good_frames += 1
            if self.good_frames < self.recovery_frames:
                mode, reason = "controlled_stop", "recovery_waiting_for_fresh_frames"
            else:
                self.recovering, self.good_frames = False, 0
        control = self._candidate(mode, reason, now,
                                  perception.valid_until if gps_ok else None)
        return SafetyAssessment(mode, reason, control)

    def _candidate(self, mode, reason, now, source_deadline):
        if mode == "normal" or self.last_good_header is None:
            return None
        if (self.last_good_at is None or
                (now - self.last_good_at) * 1000.0 > self.last_gps_ms):
            return None
        frame, timestamp, gear = self.last_good_header
        if type(frame) is not int or frame < 0 or type(timestamp) is not int or timestamp < 0:
            return None
        output = ControlOut()
        output.frame_id, output.timestamp = frame, timestamp
        output.valid_until = min(
            now + min(self.repeat_fault_ms, 100) / 1000.0,
            self.last_good_at + self.last_gps_ms / 1000.0)
        if source_deadline is not None:
            output.valid_until = min(output.valid_until, source_deadline)
        if output.valid_until <= now:
            return None
        output.gear = gear if type(gear) is int and gear in (0, 1, 2, 3) else 0
        output.throttle = 0.0
        output.brake = (self.controlled_brake if mode == "controlled_stop"
                        else self.emergency_brake)
        output.steering = 0.0
        output.source = "safety:" + mode + ":" + reason
        output.valid = True
        output.clamp()
        return output
