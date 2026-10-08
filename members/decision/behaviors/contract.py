"""R05/R07 discussion types, never attached to public Perception/DecisionTarget.

All times are seconds in an explicitly shared process-monotonic clock. This is
an executable decision-side proposal, not an approved runtime transport.
"""

import hashlib
import json
import math

CONTRACT_VERSION = "r05-r09-decision-proposal-v1"
STATUSES = ("PLANNED", "EXECUTING", "ARRIVED", "DWELLING", "COMPLETED", "REJECTED", "FAILED")
PRODUCERS = ("planning", "control", "runtime")


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


class TaskContext(object):
    __slots__ = ("case_id", "task_id", "scene_id", "session_id")

    def __init__(self, case_id, task_id, scene_id, session_id):
        require(all(isinstance(v, str) for v in (case_id, task_id, session_id))
                and bool(session_id) and type(scene_id) is int, "invalid task context")
        self.case_id, self.task_id = case_id, task_id
        self.scene_id, self.session_id = scene_id, session_id

    def key(self):
        return (self.case_id, self.task_id, self.scene_id, self.session_id)

    def token(self):
        return hashlib.sha256(json.dumps(self.key(), ensure_ascii=True).encode("utf-8")).hexdigest()[:16]

    def to_dict(self):
        return dict(case_id=self.case_id, task_id=self.task_id, scene_id=self.scene_id,
                    session_id=self.session_id)


class BehaviorFrame(object):
    __slots__ = ("context", "frame_id", "observed_at_s", "valid_until_s", "clock_id",
                 "usable", "paused", "ego_x", "ego_y", "ego_heading", "ego_speed")

    def __init__(self, context, frame_id, observed_at_s, valid_until_s,
                 ego_x=0.0, ego_y=0.0, ego_heading=0.0, ego_speed=0.0,
                 usable=False, paused=False, clock_id="process_monotonic"):
        require(isinstance(context, TaskContext) and type(frame_id) is int and frame_id >= 0,
                "invalid behavior frame identity")
        require(all(finite(v) for v in (observed_at_s, valid_until_s, ego_x, ego_y,
                                        ego_heading, ego_speed)) and ego_speed >= 0,
                "invalid behavior frame numbers")
        require(type(usable) is bool and type(paused) is bool and isinstance(clock_id, str)
                and bool(clock_id), "invalid behavior quality")
        self.context, self.frame_id = context, frame_id
        self.observed_at_s, self.valid_until_s = observed_at_s, valid_until_s
        self.clock_id, self.usable, self.paused = clock_id, usable, paused
        self.ego_x, self.ego_y = ego_x, ego_y
        self.ego_heading, self.ego_speed = ego_heading, ego_speed

    def current(self, now):
        return self.usable and finite(now) and self.observed_at_s <= now < self.valid_until_s


class LightIntent(object):
    __slots__ = ("left_signal", "right_signal", "hazard_signal")

    def __init__(self, left_signal=False, right_signal=False, hazard_signal=False):
        require(all(type(v) is bool for v in (left_signal, right_signal, hazard_signal)),
                "light intentions must be bool")
        require(not (left_signal and right_signal), "use hazard rather than both indicators")
        self.left_signal, self.right_signal, self.hazard_signal = left_signal, right_signal, hazard_signal

    def to_dict(self):
        return dict(left_signal=self.left_signal, right_signal=self.right_signal,
                    hazard_signal=self.hazard_signal)


class GoalPose(object):
    __slots__ = ("x", "y", "body_heading_rad")

    def __init__(self, x, y, body_heading_rad):
        require(all(finite(v) for v in (x, y, body_heading_rad)), "invalid goal pose")
        self.x, self.y, self.body_heading_rad = x, y, body_heading_rad

    def to_dict(self):
        return dict(x=self.x, y=self.y, body_heading_rad=self.body_heading_rad,
                    reference_point="ego_rear_axle")


class BehaviorGoal(object):
    __slots__ = ("target_lane_id", "parking_space_id", "goal_pose", "motion_direction",
                 "speed_cap_mps", "stop_distance_m", "precision_stop",
                 "minimum_standstill_duration_s", "parking_brake_at_stop", "light_intent", "stop_obligation_id")

    def __init__(self, target_lane_id=None, parking_space_id=None, goal_pose=None,
                 motion_direction=1, speed_cap_mps=0.0, stop_distance_m=-1.0,
                 precision_stop=False, minimum_standstill_duration_s=0.0,
                 parking_brake_at_stop=False, light_intent=None, stop_obligation_id=None):
        require(target_lane_id is None or isinstance(target_lane_id, str), "invalid target lane")
        require(parking_space_id is None or type(parking_space_id) is int, "invalid parking ID")
        require(goal_pose is None or isinstance(goal_pose, GoalPose), "invalid goal pose type")
        require(type(motion_direction) is int and motion_direction in (-1, 1), "invalid direction")
        require(finite(speed_cap_mps) and speed_cap_mps >= 0 and finite(stop_distance_m)
                and (stop_distance_m == -1 or stop_distance_m >= 0), "invalid goal speed/distance")
        require(finite(minimum_standstill_duration_s) and minimum_standstill_duration_s >= 0,
                "invalid minimum standstill duration")
        require(type(precision_stop) is bool and type(parking_brake_at_stop) is bool,
                "invalid stop flags")
        self.target_lane_id, self.parking_space_id, self.goal_pose = target_lane_id, parking_space_id, goal_pose
        self.motion_direction, self.speed_cap_mps = motion_direction, speed_cap_mps
        self.stop_distance_m, self.precision_stop = stop_distance_m, precision_stop
        self.minimum_standstill_duration_s = minimum_standstill_duration_s
        self.parking_brake_at_stop = parking_brake_at_stop
        require(stop_obligation_id is None or isinstance(stop_obligation_id, str), "invalid stop obligation ID")
        self.stop_obligation_id = stop_obligation_id
        self.light_intent = light_intent if light_intent is not None else LightIntent()
        require(isinstance(self.light_intent, LightIntent), "invalid lights type")

    def to_dict(self):
        return dict(target_lane_id=self.target_lane_id, parking_space_id=self.parking_space_id,
                    goal_pose=None if self.goal_pose is None else self.goal_pose.to_dict(),
                    motion_direction=self.motion_direction, speed_cap_mps=self.speed_cap_mps,
                    stop_distance_m=self.stop_distance_m, precision_stop=self.precision_stop,
                    minimum_standstill_duration_s=self.minimum_standstill_duration_s,
                    parking_brake_at_stop=self.parking_brake_at_stop,
                    light_intent=self.light_intent.to_dict(), stop_obligation_id=self.stop_obligation_id)


class BehaviorRequest(object):
    __slots__ = ("context", "intent_id", "maneuver", "stage", "revision", "source_frame_id",
                 "produced_frame_id", "produced_at_s", "issued_at_s", "valid_until_s", "clock_id", "goal",
                 "dispatch_allowed", "status", "reason_code", "attempt")

    def __init__(self, frame, intent_id, maneuver, stage, revision, source_frame_id,
                 goal, dispatch_allowed=False, status="PROPOSED", reason_code="CONTRACT_UNCONNECTED", attempt=0, issued_at_s=None):
        require(isinstance(goal, BehaviorGoal) and type(revision) is int and revision >= 1,
                "invalid behavior request")
        self.context, self.intent_id, self.maneuver, self.stage = frame.context, intent_id, maneuver, stage
        self.revision, self.source_frame_id = revision, source_frame_id
        self.produced_frame_id, self.produced_at_s = frame.frame_id, frame.observed_at_s
        self.issued_at_s = frame.observed_at_s if issued_at_s is None else issued_at_s
        self.valid_until_s, self.clock_id = frame.valid_until_s, frame.clock_id
        self.goal, self.dispatch_allowed = goal, dispatch_allowed
        self.status, self.reason_code, self.attempt = status, reason_code, attempt

    def to_dict(self):
        result = dict(contract_version=CONTRACT_VERSION, interface_status="proposal_not_runtime_connected",
                      task_context=self.context.to_dict(), intent_id=self.intent_id,
                      maneuver=self.maneuver, stage=self.stage, revision=self.revision,
                      source_frame_id=self.source_frame_id, produced_frame_id=self.produced_frame_id,
                      produced_at_s=self.produced_at_s, issued_at_s=self.issued_at_s, valid_until_s=self.valid_until_s,
                      clock_id=self.clock_id, dispatch_allowed=self.dispatch_allowed,
                      status=self.status, reason_code=self.reason_code, attempt=self.attempt)
        result.update(self.goal.to_dict())
        return result


class Capabilities(object):
    __slots__ = ("context", "actions", "produced_at_s", "valid_until_s", "clock_id", "usable", "version")

    def __init__(self, context, actions=(), produced_at_s=0.0, valid_until_s=0.0,
                 usable=False, clock_id="process_monotonic", version=CONTRACT_VERSION):
        require(isinstance(context, TaskContext) and type(usable) is bool,
                "invalid capabilities")
        require(all(isinstance(v, str) for v in actions)
                and finite(produced_at_s) and finite(valid_until_s), "invalid capability values")
        self.context, self.actions = context, frozenset(actions)
        self.produced_at_s, self.valid_until_s = produced_at_s, valid_until_s
        self.clock_id, self.usable, self.version = clock_id, usable, version

    def allows(self, frame, required):
        return (self.usable and self.version == CONTRACT_VERSION
                and self.context.key() == frame.context.key() and self.clock_id == frame.clock_id
                and self.produced_at_s <= frame.observed_at_s < self.valid_until_s
                and set(required) <= self.actions)


class ExecutionFeedback(object):
    __slots__ = ("context", "intent_id", "stage", "revision", "producer", "source_frame_id",
                 "produced_frame_id", "produced_at_s", "valid_until_s", "clock_id", "usable",
                 "status", "reason_code", "retryable", "progress", "actual_standstill_confirmed",
                 "goal_pose_arrived", "standstill_duration_s", "hold_completed", "lights_confirmed",
                 "motion_direction", "actual_pose", "actual_lane_id", "lights_duration_s", "candidate_id")

    def __init__(self, context, intent_id, stage, revision, producer, source_frame_id,
                 produced_frame_id, produced_at_s, valid_until_s, status,
                 usable=False, reason_code="", retryable=False, progress=0.0,
                 actual_standstill_confirmed=False, goal_pose_arrived=False,
                 standstill_duration_s=0.0, hold_completed=False, lights_confirmed=False,
                 motion_direction=1, actual_pose=None, clock_id="process_monotonic",
                 actual_lane_id=None, lights_duration_s=0.0, candidate_id=None):
        self.context, self.intent_id, self.stage, self.revision = context, intent_id, stage, revision
        self.producer, self.source_frame_id, self.produced_frame_id = producer, source_frame_id, produced_frame_id
        self.produced_at_s, self.valid_until_s, self.clock_id = produced_at_s, valid_until_s, clock_id
        self.usable, self.status, self.reason_code, self.retryable = usable, status, reason_code, retryable
        self.progress = progress
        self.actual_standstill_confirmed, self.goal_pose_arrived = actual_standstill_confirmed, goal_pose_arrived
        self.standstill_duration_s, self.hold_completed = standstill_duration_s, hold_completed
        self.lights_confirmed, self.motion_direction, self.actual_pose = lights_confirmed, motion_direction, actual_pose
        self.actual_lane_id, self.lights_duration_s = actual_lane_id, lights_duration_s
        self.candidate_id = candidate_id


class FeedbackValidator(object):
    def __init__(self, max_age_s=0.5):
        require(finite(max_age_s) and max_age_s > 0, "invalid feedback max age")
        self.max_age_s = max_age_s
        self._last = {}

    def reset(self):
        self._last.clear()

    def check(self, feedback, request, frame):
        if not isinstance(feedback, ExecutionFeedback):
            return False, "FEEDBACK_TYPE", False
        if not isinstance(feedback.context, TaskContext) or feedback.context.key() != frame.context.key():
            return False, "FEEDBACK_CONTEXT", False
        if (feedback.intent_id != request.intent_id or feedback.stage != request.stage
                or type(feedback.revision) is not int or feedback.revision != request.revision):
            return False, "FEEDBACK_INTENT_STAGE_REVISION", False
        producer_statuses = {"planning": ("PLANNED", "REJECTED", "FAILED"),
                             "control": ("EXECUTING", "ARRIVED", "DWELLING", "COMPLETED", "REJECTED", "FAILED"),
                             "runtime": ("EXECUTING", "ARRIVED", "DWELLING", "COMPLETED", "REJECTED", "FAILED")}
        if (feedback.producer not in PRODUCERS or feedback.status not in STATUSES
                or feedback.status not in producer_statuses.get(feedback.producer, ())):
            return False, "FEEDBACK_PRODUCER_STATUS", False
        if (type(feedback.source_frame_id) is not int or type(feedback.produced_frame_id) is not int
                or feedback.source_frame_id < request.source_frame_id
                or feedback.produced_frame_id < feedback.source_frame_id
                or feedback.produced_frame_id > frame.frame_id):
            return False, "FEEDBACK_FRAME", False
        if (feedback.clock_id != frame.clock_id or not finite(feedback.produced_at_s)
                or not finite(feedback.valid_until_s)
                or feedback.produced_at_s > frame.observed_at_s
                or feedback.produced_at_s < request.issued_at_s
                or frame.observed_at_s >= feedback.valid_until_s
                or frame.observed_at_s - feedback.produced_at_s > self.max_age_s):
            return False, "FEEDBACK_TIME", False
        flags = (feedback.usable, feedback.retryable, feedback.actual_standstill_confirmed,
                 feedback.goal_pose_arrived, feedback.hold_completed, feedback.lights_confirmed)
        if (any(type(v) is not bool for v in flags) or not feedback.usable
                or not finite(feedback.progress) or not 0 <= feedback.progress <= 1
                or not finite(feedback.standstill_duration_s) or feedback.standstill_duration_s < 0
                or type(feedback.motion_direction) is not int or feedback.motion_direction not in (-1, 1)
                or (feedback.actual_pose is not None and (not isinstance(feedback.actual_pose, GoalPose)
                    or not all(finite(v) for v in (feedback.actual_pose.x, feedback.actual_pose.y,
                                                   feedback.actual_pose.body_heading_rad))))
                or (feedback.actual_lane_id is not None and not isinstance(feedback.actual_lane_id, str))
                or not finite(feedback.lights_duration_s) or feedback.lights_duration_s < 0
                or not isinstance(feedback.reason_code, str)
                or (feedback.candidate_id is not None and not isinstance(feedback.candidate_id, str))):
            return False, "FEEDBACK_VALUES", False
        key = (feedback.producer, feedback.intent_id, feedback.stage, feedback.revision)
        sequence = (feedback.produced_frame_id, feedback.produced_at_s)
        previous = self._last.get(key)
        if previous is not None and sequence < previous:
            return False, "FEEDBACK_OUT_OF_ORDER", False
        distinct = previous is None or feedback.produced_frame_id > previous[0]
        if distinct:
            self._last[key] = sequence
            if len(self._last) > 128:
                self._last = {key: sequence}
        return True, "FEEDBACK_VALID" if distinct else "FEEDBACK_DUPLICATE", distinct
