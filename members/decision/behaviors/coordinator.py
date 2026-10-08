"""Reuse D01 policies through the optional captain-owned runtime channel."""

from core.traffic_quality import signal_stop_requirement, traffic_usable
from core.validation import number
from members.decision.behaviors.contract import TaskContext, BehaviorFrame
from members.decision.behaviors.session import BehaviorSession
from core.behavior_channel import read_channel
from members.decision.behaviors.stop_policies import (
    SignalDwellPolicy, SignalStopObservation, BlockedRoadPolicy, BlockingObservation,
)


class BehaviorCoordinator(object):
    def __init__(self, settings):
        self.settings = settings
        self._generation = 0
        self.reset()

    def _session(self):
        s = self.settings
        return BehaviorSession(s.behavior_feedback_max_age_s, s.behavior_ack_timeout_s,
                               s.behavior_progress_timeout_s, s.behavior_retry_delay_s,
                               s.behavior_max_retries, s.behavior_observation_gap_s)

    def reset(self):
        self._generation += 1
        self.signal = SignalDwellPolicy(self.settings.signal_dwell_duration_s, self._session())
        self.blocked = BlockedRoadPolicy(self.settings.release_frames, self._session())
        self._signal_seen = set()
        self._runtime_context = None
        self._latest = dict(interface_status="proposal_not_runtime_connected", requests=[],
                            dependencies=["R05_EXECUTION_FEEDBACK", "R07_BEHAVIOR_GOAL_CHANNEL"],
                            frame_id=-1)

    def _signal_observation(self, perception, output):
        traffic = perception.traffic
        candidates = []
        if traffic.signal_groups_valid:
            for group in traffic.signal_groups:
                if (not isinstance(group, dict) or group.get("boundary_kind") != "stop_line"
                        or group.get("association_valid") is not True
                        or not number(group.get("stop_line_distance")) or group["stop_line_distance"] < 0
                        or not group.get("signal_ids") or not all(type(i) is int and i >= 0 for i in group["signal_ids"])):
                    continue
                key = "signals:" + ",".join(str(i) for i in sorted(group["signal_ids"]))
                known = group.get("valid") is True and group.get("ambiguous") is False
                red = not known or group.get("signal_state") != "GREEN"
                candidates.append((group["stop_line_distance"], key, red, known))
        elif (traffic.observed and traffic.signal_id >= 0 and traffic_usable(perception)
              and number(traffic.stop_line_distance) and traffic.stop_line_distance >= 0):
            candidates.append((traffic.stop_line_distance, "signal:" + str(traffic.signal_id),
                               traffic.signal_state != "GREEN", True))
        if not candidates or self.settings.front_offset_m is None:
            return None
        distance, key, red, known = min(candidates)
        if red and known:
            self._signal_seen.add(key)
        if key not in self._signal_seen:
            return None
        stop = max(0.0, distance - self.settings.front_offset_m - self.settings.traffic_stop_margin)
        if stop<1e-6:
            stop = 0.
        return SignalStopObservation(key, stop, red, usable=known,
                                     speed_cap_mps=output.target_speed)

    def observe_legacy(self, perception, output, constraints, now_s):
        context = TaskContext(perception.case_id, perception.task_id, perception.scene_id,
                              "decision-session:" + str(self._generation))
        capabilities,feedbacks,connected = None,[],False
        try:
            context,capabilities,feedbacks = read_channel(perception,now_s)
            connected = True
            self._runtime_context = context
        except (ValueError,TypeError,AttributeError,KeyError):
            if self._runtime_context is not None:
                context = self._runtime_context
        def reply(session):
            current = [f for f in feedbacks if f.intent_id==session.intent_id and f.revision==session.revision]
            return max(current,key=lambda f:(f.produced_at_s,f.producer!="planning")) if current else None
        frame = BehaviorFrame(context, perception.frame_id, now_s, perception.valid_until,
                              perception.ego.x, perception.ego.y, perception.ego.heading,
                              perception.ego.speed, usable=True)
        requests, policy_states = [], []
        # Scene IDs select the supplied stop/light duty only; geometry and
        # collision/stop determination are still the existing shared facts.
        signal = self.signal.evaluate(frame, self._signal_observation(perception, output),
                                      capabilities=capabilities,feedback=reply(self.signal.session),
                                      dwell_required=perception.scene_id == 20,
                                      safety_override=output.mode == "EMERGENCY_BRAKE")
        policy_states.append(dict(policy="SIGNAL_DWELL", phase=signal.phase, reason_code=signal.reason))
        if signal.request is not None:
            requests.append(signal.request.to_dict("runtime_connected" if connected else "proposal_not_runtime_connected"))
            if connected and signal.request.dispatch_allowed and output.mode!="EMERGENCY_BRAKE":
                output.behavior_request = signal.request.to_dict("runtime_connected")
                output.behavior_active_identity = {key:output.behavior_request[key] for key in
                    ("intent_id","stage","revision","source_frame_id","issued_at_s")}
        if connected and signal.hold_required and output.mode!="EMERGENCY_BRAKE":
            output.mode,output.target_speed,output.stop_distance = "STOP",0.,0.
            output.precision_stop = True
            output.reason = "SIGNAL_DWELL:"+signal.reason
        if self.execution_pending() and (not connected or signal.request is None
                                         or not signal.request.dispatch_allowed):
            output.mode = "EMERGENCY_BRAKE" if perception.ego.speed>self.settings.standstill_speed else "STOP"
            output.target_speed,output.stop_distance = 0.,0.
            output.behavior_request,output.behavior_active_identity = None,None
            output.reason = "SIGNAL_OBLIGATION_PENDING:"+signal.reason
        obstruction = None
        if perception.scene_id == 11 and constraints is not None and constraints.blockage_candidates:
            identifier, distance = min(constraints.blockage_candidates, key=lambda item: (item[1], item[0]))
            obstruction = BlockingObservation("target:" + str(identifier), distance, True,
                                               speed_cap_mps=output.target_speed)
        if obstruction is None and self.blocked.session.intent_id:
            # A legacy empty frame is not verified full-corridor clearance.
            obstruction = BlockingObservation("prior", None, False, usable=False)
        blockage = self.blocked.evaluate(frame, obstruction,
                                         capabilities=capabilities,feedback=reply(self.blocked.session),
                                         safety_override=output.mode == "EMERGENCY_BRAKE")
        policy_states.append(dict(policy="BLOCKED_ROAD", phase=blockage.phase, reason_code=blockage.reason))
        if blockage.request is not None:
            requests.append(blockage.request.to_dict())
        self._latest = dict(interface_status="runtime_connected" if connected else "proposal_not_runtime_connected", frame_id=perception.frame_id,
                            requests=requests, policies=policy_states,
                            dependencies=([] if connected else ["R05_EXECUTION_FEEDBACK", "R07_BEHAVIOR_GOAL_CHANNEL",
                                          "DOWNSTREAM_CAPABILITIES"]))

    def execution_pending(self):
        session = self.signal.session
        return bool(self._runtime_context is not None and session.intent_id
                    and session.status not in ("COMPLETED","CANCELLED"))

    def diagnostic_error(self, frame_id, reason):
        self._latest = dict(interface_status="proposal_not_runtime_connected", frame_id=frame_id,
                            requests=[], reason_code="BEHAVIOR_DIAGNOSTIC_ERROR:" + reason,
                            dependencies=["R05_EXECUTION_FEEDBACK", "R07_BEHAVIOR_GOAL_CHANNEL"])

    def snapshot(self):
        import copy
        return copy.deepcopy(self._latest)
