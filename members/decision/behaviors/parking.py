"""D04 parking goal selection and actual-feedback stage progression."""

import math
from core.geometry import normalize_angle
from members.decision.behaviors.contract import BehaviorGoal, GoalPose, finite, require
from members.decision.behaviors.observations import Evidence, MotionObject, polygon, inside, intersects, vehicle_footprint
from members.decision.behaviors.session import BehaviorSession
from members.decision.behaviors.stop_policies import PolicyResult


class ParkingSpace(object):
    def __init__(self, identifier, boundary, body_heading_rad, occupancy, evidence,
                 approach_pose=None, position_pose=None):
        require(type(identifier) is int and identifier >= 0 and finite(body_heading_rad), "invalid parking space")
        require(occupancy in ("empty", "occupied", "unknown") and isinstance(evidence, Evidence), "invalid occupancy")
        require(all(p is None or isinstance(p, GoalPose) for p in (approach_pose, position_pose)), "invalid entry pose")
        self.identifier, self.boundary, self.body_heading_rad = identifier, polygon(boundary), body_heading_rad
        self.occupancy, self.evidence = occupancy, evidence
        self.approach_pose, self.position_pose = approach_pose, position_pose


class ParkingFacts(object):
    def __init__(self, evidence, spaces, free_area, objects=(), rear_coverage_verified=False,
                 front_coverage_verified=False, exit_goal=None, signed_body_speed_mps=0.0):
        require(isinstance(evidence, Evidence) and all(isinstance(s, ParkingSpace) for s in spaces), "invalid parking facts")
        require(all(isinstance(o, MotionObject) for o in objects) and finite(signed_body_speed_mps), "invalid parking objects/motion")
        require(type(rear_coverage_verified) is bool and type(front_coverage_verified) is bool,
                "invalid parking coverage flags")
        require(exit_goal is None or isinstance(exit_goal, GoalPose), "invalid exit goal")
        self.evidence, self.spaces, self.free_area, self.objects = evidence, tuple(spaces), polygon(free_area), tuple(objects)
        self.rear_coverage_verified, self.front_coverage_verified = rear_coverage_verified, front_coverage_verified
        self.exit_goal, self.signed_body_speed_mps = exit_goal, signed_body_speed_mps


class ParkingPolicy(object):
    def __init__(self, front_m, rear_m, half_width_m, dwell_s=10.0,
                 position_tolerance_m=0.15, heading_tolerance_rad=0.15,
                 speed_cap_mps=1.0, session=None):
        require(all(finite(v) and v > 0 for v in (front_m, rear_m, half_width_m, dwell_s,
                position_tolerance_m, heading_tolerance_rad, speed_cap_mps)) and dwell_s >= 10.0,
                "invalid parking limits")
        self.front_m, self.rear_m, self.half_width_m = front_m, rear_m, half_width_m
        self.dwell_s, self.position_tolerance_m, self.heading_tolerance_rad = dwell_s, position_tolerance_m, heading_tolerance_rad
        self.speed_cap_mps, self.session = speed_cap_mps, session or BehaviorSession()
        self.reset()

    def reset(self):
        self.session.reset()
        self._context, self.space, self.park_goal = None, None, None

    def rear_axle_goal(self, space):
        x = sum(p[0] for p in space.boundary) / len(space.boundary)
        y = sum(p[1] for p in space.boundary) / len(space.boundary)
        offset = (self.front_m - self.rear_m) / 2.0
        return GoalPose(x - offset * math.cos(space.body_heading_rad),
                        y - offset * math.sin(space.body_heading_rad), space.body_heading_rad)

    def _body_clear(self, pose, facts, space=None):
        body = vehicle_footprint(pose, self.front_m, self.rear_m, self.half_width_m)
        if not all(inside(point, facts.free_area) for point in body):
            return False
        if space is not None and not all(inside(point, space.boundary) for point in body):
            return False
        return not any(intersects(body, obj.footprint()) for obj in facts.objects)

    def _arrived(self, reply, goal):
        return (reply is not None and reply.producer in ("control", "runtime")
                and reply.actual_pose is not None and reply.goal_pose_arrived
                and reply.actual_standstill_confirmed
                and math.hypot(reply.actual_pose.x - goal.x, reply.actual_pose.y - goal.y) <= self.position_tolerance_m
                and abs(normalize_angle(reply.actual_pose.body_heading_rad - goal.body_heading_rad)) <= self.heading_tolerance_rad)

    def _goal(self, pose, direction=1, dwell=False):
        return BehaviorGoal(parking_space_id=self.space.identifier, goal_pose=pose,
                            motion_direction=direction, speed_cap_mps=self.speed_cap_mps,
                            precision_stop=True, minimum_standstill_duration_s=self.dwell_s if dwell else 0.0,
                            parking_brake_at_stop=dwell,
                            stop_obligation_id="parking:" + str(self.space.identifier) if dwell else None)

    def evaluate(self, frame, facts, capabilities=None, feedback=None, safety_override=False, intent_key="park"):
        if not frame.current(frame.observed_at_s):
            request = self.session.tick(frame, capabilities, safety_override=True) if self.session.intent_id else None
            return PolicyResult(request, True, "OBSERVATION_UNUSABLE", "RECOVER")
        if self._context != frame.context.key():
            self.reset()
            self._context = frame.context.key()
        if self.session.status in ("COMPLETED", "CANCELLED"):
            request = self.session.snapshot(frame)
            return PolicyResult(request, request.status == "CANCELLED", request.reason_code,
                                "COMPLETE" if request.status == "COMPLETED" else "CANCELLED")
        if not isinstance(facts, ParkingFacts) or not facts.evidence.verified(frame, allowed_source_kinds=("sensor", "verified_fusion", "synthetic")):
            request = self.session.tick(frame, capabilities, safety_override=True) if self.session.intent_id else None
            return PolicyResult(request, bool(request), "FREE_SPACE_OCCUPANCY_COVERAGE_UNKNOWN", "RECOVER" if request else "SEARCH")
        if not facts.rear_coverage_verified or not facts.front_coverage_verified:
            request = self.session.block(frame, "PARKING_SURROUND_COVERAGE_UNKNOWN") if self.session.intent_id else None
            return PolicyResult(request, True, "PARKING_SURROUND_COVERAGE_UNKNOWN", "BLOCKED")
        if self.space is None:
            options = []
            fitting_but_entry_unknown = False
            for space in facts.spaces:
                if not space.evidence.verified(frame) or space.occupancy != "empty":
                    continue
                goal = self.rear_axle_goal(space)
                if self._body_clear(goal, facts, space):
                    if (space.approach_pose is not None and space.position_pose is not None
                            and self._body_clear(space.approach_pose, facts)
                            and self._body_clear(space.position_pose, facts)):
                        options.append((space.identifier, space, goal))
                    else:
                        fitting_but_entry_unknown = True
            if not options:
                return PolicyResult(reason="VERIFIED_APPROACH_AND_ENTRY_POSES_UNAVAILABLE" if fitting_but_entry_unknown
                                    else "NO_VERIFIED_FITTING_FREE_PARKING_SPACE", phase="SELECT")
            _, self.space, self.park_goal = min(options, key=lambda item: item[0])
            if self.space.approach_pose is None or self.space.position_pose is None:
                self.space, self.park_goal = None, None
                return PolicyResult(reason="VERIFIED_APPROACH_AND_ENTRY_POSES_UNAVAILABLE", phase="BLOCKED")
            self.session.start(frame, intent_key, "PARK", "APPROACH", self._goal(self.space.approach_pose),
                               ("PARK", "PATH", "FEEDBACK"))
        observed_space = next((s for s in facts.spaces if s.identifier == self.space.identifier), None)
        if (observed_space is None or not observed_space.evidence.verified(frame)
                or observed_space.occupancy != "empty"):
            request = self.session.block(frame, "SELECTED_PARKING_OCCUPANCY_UNKNOWN_OR_OCCUPIED")
            return PolicyResult(request, True, request.reason_code, "BLOCKED")
        # Keep the selected poses stable, but validate them against fresh space
        # boundaries. A refined observation must not certify the old geometry.
        if ((self.session.stage != "EXIT" and not self._body_clear(self.park_goal, facts, observed_space))
                or not self._body_clear(self.session.goal.goal_pose, facts,
                    observed_space if self.session.stage in ("REVERSE_ENTRY", "ALIGN", "PARKED_DWELL") else None)):
            request = self.session.block(frame, "NEW_PARKING_GOAL_OBSTRUCTION")
            return PolicyResult(request, True, request.reason_code, "BLOCKED")
        if (self.session.status == "BLOCKED" and self.session.reason_code in
                ("PARKING_SURROUND_COVERAGE_UNKNOWN", "SELECTED_PARKING_OCCUPANCY_UNKNOWN_OR_OCCUPIED",
                 "NEW_PARKING_GOAL_OBSTRUCTION")):
            if self.session.retry_after_evidence(frame, "PARKING_EVIDENCE_RECOVERED_REPLAN"):
                old = self.session.goal
                self.session.goal = self._goal(old.goal_pose, old.motion_direction,
                                              old.minimum_standstill_duration_s > 0)
        authorized_reverse = self.session.stage in ("REVERSE_ENTRY", "ALIGN") and self.session.status in ("ACCEPTED", "EXECUTING")
        if facts.signed_body_speed_mps < -0.05 and not authorized_reverse:
            request = self.session.tick(frame, capabilities, safety_override=True)
            return PolicyResult(request, True, "UNEXPECTED_REVERSE_MOTION", "RECOVER")
        request = self.session.tick(frame, capabilities, feedback, safety_override)
        if request.status == "COMPLETED":
            return PolicyResult(request, False, request.reason_code, "COMPLETE")
        if request.status in ("BLOCKED", "SUSPENDED", "CANCELLED"):
            return PolicyResult(request, True, request.reason_code, request.status)
        reply = self.session.last_feedback
        stage = self.session.stage
        parked = (self._arrived(reply, self.park_goal)
                  and self._body_clear(reply.actual_pose, facts, observed_space))
        if stage == "APPROACH" and self._arrived(reply, self.space.approach_pose):
            request = self.session.advance(frame, "POSITION", self._goal(self.space.position_pose), reason="APPROACH_ACTUALLY_STOPPED")
        elif stage == "POSITION" and self._arrived(reply, self.space.position_pose):
            request = self.session.advance(frame, "REVERSE_ENTRY", self._goal(self.park_goal, -1, True),
                                           required=("PARK", "PATH", "REVERSE", "DWELL", "FEEDBACK"),
                                           reason="ENTRY_POSITION_ACTUALLY_STOPPED")
        elif stage in ("REVERSE_ENTRY", "ALIGN"):
            if parked:
                request = self.session.advance(frame, "PARKED_DWELL", self._goal(self.park_goal, -1, True), reason="WHOLE_BODY_ACTUALLY_PARKED")
            elif (reply is not None and reply.producer in ("control", "runtime")
                  and reply.actual_standstill_confirmed and reply.progress >= 1.0 and stage == "REVERSE_ENTRY"):
                request = self.session.advance(frame, "ALIGN", self._goal(self.park_goal, -1, True), reason="PARKING_POSE_ALIGNMENT_REQUIRED")
        elif stage == "PARKED_DWELL":
            if parked and reply.hold_completed and reply.standstill_duration_s >= self.dwell_s:
                request = self.session.advance(frame, "EXIT_PREPARE", self._goal(self.park_goal, -1), reason="TEN_SECOND_DWELL_ACTUALLY_COMPLETED")
        elif stage == "EXIT_PREPARE":
            if (parked and facts.exit_goal is not None
                    and self._body_clear(facts.exit_goal, facts)):
                request = self.session.advance(frame, "EXIT", self._goal(facts.exit_goal, 1), reason="STOPPED_AND_EXIT_GOAL_VERIFIED")
        elif stage == "EXIT" and self._arrived(reply, self.session.goal.goal_pose):
            request = self.session.finish(frame, True, "PARKING_EXIT_ACTUALLY_COMPLETED")
        return PolicyResult(request, stage in ("PARKED_DWELL", "EXIT_PREPARE"), request.reason_code, self.session.stage)
