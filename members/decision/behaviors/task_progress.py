"""D09 explicit segment progress and actual destination completion."""

import math
from core.geometry import normalize_angle
from members.decision.behaviors.contract import BehaviorGoal, GoalPose, finite, require
from members.decision.behaviors.observations import Evidence, polygon, inside
from members.decision.behaviors.session import BehaviorSession
from members.decision.behaviors.stop_policies import PolicyResult


class TaskRouteFacts(object):
    def __init__(self, evidence, route_id, segments, segment_id, progress_s_m,
                 reference_end_reason, destination_region=None, destination_goal=None,
                 speed_cap_mps=3.0, selected_branch_verified=True):
        require(isinstance(evidence, Evidence) and isinstance(route_id, str) and bool(route_id), "invalid task route")
        require(isinstance(segments, (tuple, list)) and bool(segments), "task segments unavailable")
        normalized = []
        for segment in segments:
            require(isinstance(segment, (tuple, list)) and len(segment) == 4,
                    "segment requires identity,lane,start,end")
            identity, lane, start, end = segment
            require(isinstance(identity, str) and isinstance(lane, str) and finite(start) and finite(end)
                    and 0 <= start < end, "invalid task segment")
            require(not normalized or start >= normalized[-1][3], "task segment overlap")
            require(all(identity != s[0] for s in normalized), "ambiguous repeated segment identity")
            normalized.append((identity, lane, start, end))
        require(isinstance(segment_id, str) and finite(progress_s_m) and progress_s_m >= 0,
                "invalid task progress")
        require(reference_end_reason in ("task_destination", "window_end", "map_end", "branch_ambiguous", "map_query_failed"),
                "invalid reference end meaning")
        require(destination_goal is None or isinstance(destination_goal, GoalPose), "invalid task destination")
        require(finite(speed_cap_mps) and speed_cap_mps >= 0 and type(selected_branch_verified) is bool, "invalid task cap")
        self.evidence, self.route_id, self.segments, self.segment_id = evidence, route_id, tuple(normalized), segment_id
        self.progress_s_m, self.reference_end_reason = progress_s_m, reference_end_reason
        self.destination_region = polygon(destination_region) if destination_region is not None else None
        self.destination_goal, self.speed_cap_mps = destination_goal, speed_cap_mps
        self.selected_branch_verified = selected_branch_verified


class TaskProgressPolicy(object):
    def __init__(self, progress_slack_m=0.75, session=None):
        require(finite(progress_slack_m) and progress_slack_m > 0, "invalid task progress tolerance")
        self.progress_slack_m, self.session = progress_slack_m, session or BehaviorSession()
        self.reset()

    def reset(self):
        self.session.reset()
        self._context, self._route_id, self._progress, self._time, self._frame, self._index = None, None, None, None, None, None
        self._speed = 0.0

    def _deny(self, frame, capabilities, reason, phase):
        request = self.session.tick(frame, capabilities, safety_override=True) if self.session.intent_id else None
        return PolicyResult(request, True, reason, phase)

    def evaluate(self, frame, facts, capabilities=None, feedback=None, safety_override=False,
                 temporary_maneuver_active=False):
        if not frame.current(frame.observed_at_s):
            request = self.session.tick(frame, capabilities, safety_override=True) if self.session.intent_id else None
            return PolicyResult(request, True, "TASK_OBSERVATION_UNUSABLE", "RECOVER")
        if self._context != frame.context.key():
            self.reset()
            self._context = frame.context.key()
        if not isinstance(facts, TaskRouteFacts) or not facts.evidence.verified(frame, allowed_source_kinds=("task", "verified_fusion", "synthetic")):
            request = self.session.tick(frame, capabilities, safety_override=True) if self.session.intent_id else None
            return PolicyResult(request, True, "TASK_ROUTE_PROGRESS_UNKNOWN", "RECOVER" if request else "BLOCKED")
        if not facts.selected_branch_verified:
            return self._deny(frame, capabilities, "TASK_BRANCH_UNSELECTED", "WAIT_ROUTE")
        index = next((i for i,s in enumerate(facts.segments) if s[0] == facts.segment_id), None)
        if index is None or not facts.segments[index][2] <= facts.progress_s_m <= facts.segments[index][3]:
            return self._deny(frame, capabilities, "TASK_SEGMENT_PROGRESS_MISMATCH", "RECOVER")
        if self._route_id is not None and facts.route_id != self._route_id:
            return self._deny(frame, capabilities, "TASK_ROUTE_CHANGED_WITHOUT_CONTEXT_RESET", "RECOVER")
        if self._frame is not None and frame.frame_id < self._frame:
            self.reset()
            self._context = frame.context.key()
            return PolicyResult(reason="TASK_FRAME_ROLLBACK_REANCHOR", hold_required=True, phase="RECOVER")
        distinct = self._frame is None or frame.frame_id > self._frame
        if self._progress is not None and distinct:
            dt = frame.observed_at_s - self._time
            if (dt < 0 or index < self._index or facts.progress_s_m + self.progress_slack_m < self._progress
                    or facts.progress_s_m - self._progress > max(frame.ego_speed, self._speed) * dt + self.progress_slack_m):
                return self._deny(frame, capabilities, "TASK_PROGRESS_UNREACHABLE_OR_RETURN_LEG_JUMP", "RECOVER")
        if distinct:
            self._progress, self._time, self._frame, self._index = facts.progress_s_m, frame.observed_at_s, frame.frame_id, index
            self._speed = frame.ego_speed
        self._route_id = facts.route_id
        at_local_end = facts.progress_s_m >= facts.segments[index][3] - 1e-6
        if at_local_end and facts.reference_end_reason in ("window_end", "branch_ambiguous", "map_query_failed"):
            return self._deny(frame, capabilities, "LOCAL_REFERENCE_END_NOT_TASK_COMPLETE", "WAIT_ROUTE")
        if temporary_maneuver_active:
            return self._deny(frame, capabilities, "TEMPORARY_MANEUVER_TASK_ROUTE_PRESERVED", "WAIT_MANEUVER")
        goal = BehaviorGoal(target_lane_id=facts.segments[index][1], goal_pose=facts.destination_goal,
                            speed_cap_mps=facts.speed_cap_mps, precision_stop=facts.destination_goal is not None)
        if self.session.intent_id is None:
            self.session.start(frame, "task:" + facts.route_id, "CONTINUOUS_TASK", "FOLLOW_ROUTE", goal,
                               ("TASK_ROUTE", "PATH", "FEEDBACK"))
        elif self.session.status == "COMPLETED":
            return PolicyResult(self.session.snapshot(frame), False, "TASK_ALREADY_COMPLETED", "COMPLETE")
        else:
            self.session.start(frame, "task:" + facts.route_id, "CONTINUOUS_TASK", "FOLLOW_ROUTE", goal,
                               ("TASK_ROUTE", "PATH", "FEEDBACK"))
        request = self.session.tick(frame, capabilities, feedback, safety_override)
        reply = self.session.last_feedback
        actual_destination = (facts.reference_end_reason == "task_destination"
                              and index == len(facts.segments) - 1
                              and facts.destination_region is not None and facts.destination_goal is not None
                              and reply is not None and reply.producer in ("control", "runtime")
                              and reply.actual_pose is not None and reply.goal_pose_arrived
                              and reply.actual_standstill_confirmed
                              and inside((reply.actual_pose.x, reply.actual_pose.y), facts.destination_region)
                              and math.hypot(reply.actual_pose.x - facts.destination_goal.x,
                                             reply.actual_pose.y - facts.destination_goal.y) <= 0.15
                              and abs(normalize_angle(reply.actual_pose.body_heading_rad
                                                      - facts.destination_goal.body_heading_rad)) <= 0.15)
        if actual_destination:
            request = self.session.finish(frame, True, "TASK_DESTINATION_ACTUALLY_REACHED_AND_STOPPED")
            return PolicyResult(request, True, request.reason_code, "COMPLETE")
        return PolicyResult(request, request.status in ("BLOCKED", "SUSPENDED", "CANCELLED", "RETRY_WAIT"),
                            request.reason_code, "FOLLOW_ROUTE")
