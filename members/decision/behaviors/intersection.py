"""D05–D07 occupancy, verified entry window and exit-space admission."""

from members.decision.behaviors.contract import BehaviorGoal, GoalPose, finite, require
from members.decision.behaviors.observations import Evidence, MotionObject, polygon, occupancy_windows, inside
from members.decision.behaviors.session import BehaviorSession
from members.decision.behaviors.stop_policies import PolicyResult


class OccupancyResult(object):
    def __init__(self, state, identifiers=(), windows=(), reason=""):
        self.state, self.identifiers, self.windows, self.reason = state, tuple(sorted(identifiers)), tuple(windows), reason


class OccupancyTracker(object):
    def __init__(self, clear_frames=2, return_guard_s=0.5, horizon_s=3.0):
        require(type(clear_frames) is int and clear_frames > 0
                and finite(return_guard_s) and return_guard_s >= 0
                and finite(horizon_s) and horizon_s > 0, "invalid occupancy policy")
        self.clear_frames, self.return_guard_s, self.horizon_s = clear_frames, return_guard_s, horizon_s
        self.reset()

    def reset(self):
        self._context, self._last_frame, self._last_occupied, self._clear = None, None, None, 0
        self._ids = set()

    def evaluate(self, frame, evidence, objects, region, ego_window=None):
        if not isinstance(evidence, Evidence) or not evidence.verified(frame, allowed_source_kinds=("sensor", "verified_fusion", "synthetic")):
            self._clear = 0
            return OccupancyResult("UNKNOWN", self._ids, reason="DYNAMIC_COVERAGE_UNVERIFIED")
        if self._context != frame.context.key() or self._last_frame is not None and frame.frame_id < self._last_frame:
            self.reset()
            self._context = frame.context.key()
        distinct = self._last_frame is None or frame.frame_id > self._last_frame
        self._last_frame = frame.frame_id
        if ego_window is not None:
            require(len(ego_window) == 2 and all(finite(v) and v >= 0 for v in ego_window)
                    and ego_window[0] <= ego_window[1], "invalid ego occupancy window")
            if ego_window[1] > self.horizon_s:
                return OccupancyResult("UNKNOWN", reason="EGO_WINDOW_OUTSIDE_PREDICTION_COVERAGE")
        region = polygon(region)
        windows = occupancy_windows(objects, region, self.horizon_s)
        relevant = [w for w in windows if ego_window is None or not (w[2] < ego_window[0] or w[1] > ego_window[1])]
        if relevant:
            self._ids = set(w[0] for w in relevant)
            self._last_occupied, self._clear = frame.observed_at_s, 0
            return OccupancyResult("OCCUPIED", self._ids, relevant, "CONFLICT_OCCUPANCY")
        if self._last_occupied is not None and frame.observed_at_s - self._last_occupied < self.return_guard_s:
            return OccupancyResult("OCCUPIED", self._ids, reason="SHORT_DISAPPEARANCE_OR_RETURN_GUARD")
        if distinct:
            self._clear += 1
        if self._clear < self.clear_frames:
            return OccupancyResult("UNKNOWN", self._ids, reason="CLEAR_COVERAGE_CONFIRMING")
        self._ids.clear()
        return OccupancyResult("CLEAR", reason="FRESH_COVERAGE_CONFIRMED_CLEAR")


class IntersectionFacts(object):
    def __init__(self, identifier, evidence, conflict_region, objects, stop_distance_m,
                 signal_permits, rule_verified, right_of_way, exit_clearance_m,
                 exit_coverage_verified, ego_window, inside_intersection=False,
                 exit_region=None, exit_goal=None, exit_path_verified=False,
                 speed_cap_mps=3.0):
        require(isinstance(identifier, str) and bool(identifier) and isinstance(evidence, Evidence), "invalid intersection identity")
        require(all(isinstance(o, MotionObject) for o in objects), "invalid intersection objects")
        require(stop_distance_m is None or finite(stop_distance_m) and stop_distance_m >= 0, "invalid entry boundary")
        require(right_of_way in ("EGO_PRIORITY", "YIELD", "UNKNOWN"), "invalid rule result")
        require(all(type(v) is bool for v in (signal_permits, rule_verified, exit_coverage_verified,
                                              inside_intersection, exit_path_verified)), "invalid intersection gates")
        require(exit_clearance_m is None or finite(exit_clearance_m) and exit_clearance_m >= 0, "invalid exit space")
        require(exit_goal is None or isinstance(exit_goal, GoalPose), "invalid exit target")
        require(finite(speed_cap_mps) and speed_cap_mps >= 0, "invalid entry cap")
        self.identifier, self.evidence = identifier, evidence
        self.conflict_region, self.objects = polygon(conflict_region), tuple(objects)
        self.stop_distance_m, self.signal_permits = stop_distance_m, signal_permits
        self.rule_verified, self.right_of_way = rule_verified, right_of_way
        self.exit_clearance_m, self.exit_coverage_verified = exit_clearance_m, exit_coverage_verified
        self.ego_window, self.inside_intersection = ego_window, inside_intersection
        self.exit_region = polygon(exit_region) if exit_region is not None else None
        self.exit_goal, self.exit_path_verified, self.speed_cap_mps = exit_goal, exit_path_verified, speed_cap_mps


class IntersectionPolicy(object):
    def __init__(self, vehicle_length_m, exit_gap_m=4.0, occupancy=None, session=None):
        require(finite(vehicle_length_m) and vehicle_length_m > 0
                and finite(exit_gap_m) and exit_gap_m >= 0, "invalid entry space requirement")
        self.vehicle_length_m, self.exit_gap_m = vehicle_length_m, exit_gap_m
        self.occupancy, self.session = occupancy or OccupancyTracker(), session or BehaviorSession()
        self.reset()

    def reset(self):
        self.occupancy.reset()
        self.session.reset()
        self._context = None

    def evaluate(self, frame, facts, capabilities=None, feedback=None, safety_override=False, purpose="INTERSECTION"):
        if not frame.current(frame.observed_at_s):
            request = self.session.tick(frame, capabilities, safety_override=True) if self.session.intent_id else None
            return PolicyResult(request, True, "OBSERVATION_UNUSABLE", "RECOVER")
        if self._context != frame.context.key():
            self.reset()
            self._context = frame.context.key()
        if not isinstance(facts, IntersectionFacts) or not facts.evidence.verified(frame, allowed_source_kinds=("sensor", "verified_fusion", "synthetic")):
            request = self.session.tick(frame, capabilities, safety_override=True) if self.session.intent_id else None
            return PolicyResult(request, True, "INTERSECTION_INPUT_COVERAGE_UNKNOWN", "RECOVER" if request else "BLOCKED")
        require(purpose in ("INTERSECTION", "CROSSWALK", "JUNCTION_ENTRY"), "invalid passage purpose")
        occupied = self.occupancy.evaluate(frame, facts.evidence, facts.objects,
                                           facts.conflict_region, facts.ego_window)
        enough_exit = (facts.exit_coverage_verified and facts.exit_clearance_m is not None
                       and facts.exit_clearance_m >= self.vehicle_length_m + self.exit_gap_m)
        allowed = (facts.rule_verified and facts.right_of_way != "UNKNOWN"
                   and facts.signal_permits and occupied.state == "CLEAR" and enough_exit)
        reason = ("EXIT_COVERAGE_OR_SPACE_UNKNOWN" if not facts.exit_coverage_verified or facts.exit_clearance_m is None else
                  "EXIT_TOO_SHORT_FOR_VEHICLE" if not enough_exit else
                  "RIGHT_OF_WAY_UNKNOWN" if not facts.rule_verified or facts.right_of_way == "UNKNOWN" else
                  "SIGNAL_REQUIRES_STOP" if not facts.signal_permits else occupied.reason)
        if self.session.status == "COMPLETED" and self.session.key == facts.identifier:
            return PolicyResult(self.session.snapshot(frame), False, "PASSAGE_ALREADY_COMPLETED", "COMPLETE")
        if self.session.status == "BLOCKED":
            recovered = ((self.session.reason_code == "ENTRY_STOP_BOUNDARY_UNKNOWN"
                          and facts.stop_distance_m is not None)
                         or (self.session.reason_code == "INSIDE_JUNCTION_MINIMUM_RISK_PATH_REQUIRED"
                             and facts.inside_intersection and facts.exit_path_verified
                             and facts.exit_goal is not None))
            if recovered:
                self.session.retry_after_evidence(frame, "PASSAGE_EVIDENCE_RECOVERED_REVALIDATE")
        if facts.inside_intersection:
            if not facts.exit_path_verified or facts.exit_goal is None:
                if self.session.intent_id is None:
                    self.session.start(frame, facts.identifier, purpose, "INSIDE_RECOVER", BehaviorGoal(),
                                       ("INTERSECTION", "PATH", "FEEDBACK"))
                request = self.session.block(frame, "INSIDE_JUNCTION_MINIMUM_RISK_PATH_REQUIRED")
                return PolicyResult(request, True, request.reason_code, "INSIDE_RECOVER")
            goal = BehaviorGoal(goal_pose=facts.exit_goal, speed_cap_mps=facts.speed_cap_mps)
            if self.session.intent_id is None:
                self.session.start(frame, facts.identifier, purpose, "EXIT", goal,
                                   ("INTERSECTION", "PATH", "FEEDBACK"))
            elif self.session.stage != "EXIT":
                self.session.advance(frame, "EXIT", goal, reason="VERIFIED_EXIT_PATH_WHILE_INSIDE")
        else:
            if not allowed and facts.stop_distance_m is None:
                if self.session.intent_id is None:
                    self.session.start(frame, facts.identifier, purpose, "WAIT_ENTRY", BehaviorGoal(),
                                       ("INTERSECTION", "PATH_STOP", "FEEDBACK"))
                request = self.session.block(frame, "ENTRY_STOP_BOUNDARY_UNKNOWN")
                return PolicyResult(request, True, request.reason_code, "WAIT_ENTRY")
            stage = "ENTER" if allowed else "WAIT_ENTRY"
            goal = BehaviorGoal(speed_cap_mps=facts.speed_cap_mps,
                                stop_distance_m=-1.0 if allowed else facts.stop_distance_m,
                                precision_stop=not allowed)
            if self.session.intent_id is None or self.session.key != facts.identifier:
                self.session.start(frame, facts.identifier, purpose, stage, goal,
                                   ("INTERSECTION", "PATH", "FEEDBACK"))
            elif self.session.stage != stage and self.session.stage != "EXIT":
                self.session.advance(frame, stage, goal, reason="ENTRY_WINDOW_OPEN" if allowed else reason)
            elif self.session.stage != "EXIT":
                self.session.start(frame, facts.identifier, purpose, stage, goal,
                                   ("INTERSECTION", "PATH", "FEEDBACK"))
        request = self.session.tick(frame, capabilities, feedback, safety_override)
        reply = self.session.last_feedback
        if (reply is not None and reply.producer in ("control", "runtime")
                and reply.actual_pose is not None and reply.progress >= 1.0
                and facts.exit_region is not None
                and inside((reply.actual_pose.x, reply.actual_pose.y), facts.exit_region)):
            request = self.session.finish(frame, True, "PASSAGE_ACTUALLY_EXITED")
            return PolicyResult(request, False, request.reason_code, "COMPLETE")
        return PolicyResult(request, not allowed and not facts.inside_intersection, reason, self.session.stage)
