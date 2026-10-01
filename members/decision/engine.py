"""Decision member entry logic: choose a behaviour and a numeric demand.

Input is the captain's `Perception`; output is a `DecisionTarget`. This member
never touches the SDK, never outputs actuators and never plans geometry.

Design rules, in priority order:

1. Unknown is not safe. A source the captain marked unusable is treated as
   absent evidence, not as an empty road.
2. Fail safe, not silent. Every degraded decision says why in `reason`.
3. Latched safety. A blind-stop or following state is not released by a
   single frame of good news.
4. No silent behaviour invention. Situations this member cannot resolve
   (unverified lane membership, ambiguous signal groups, shared-lane
   obstacles that would need steering) become a documented stop.
"""

from core.interfaces import DecisionMode, DecisionTarget

from members.decision import candidates as candidate_rules
from members.decision import protocol, speed_policy
from members.decision.settings import DecisionSettings


def _link_frame(perception, output):
    """Copy the source frame identity without trusting the argument's type.

    A malformed perception must still produce a usable-shaped decision, so the
    frame fields are set defensively and the caller's validation catches the
    rest. `Perception.bind` cannot be used directly here because it would
    raise before the engine's own error handling is in place.
    """
    frame_id = getattr(perception, "frame_id", None)
    timestamp = getattr(perception, "timestamp", None)
    valid_until = getattr(perception, "valid_until", None)
    output.frame_id = frame_id if type(frame_id) is int else -1
    output.timestamp = timestamp if type(timestamp) is int else 0
    output.valid_until = valid_until if type(valid_until) in (int, float) else 0.0


class DecisionEngine(object):
    """Stateful behaviour selector.

    The state exists only to protect against single-frame flapping:

    - `_blind_fault_count`: consecutive frames with unusable target data
    - `_blind_stop`: a sustained blind fault, latched until the vehicle is slow
    """

    def __init__(self, settings=None):
        # Environment overrides are read here so field tuning needs no code
        # change and no edit to the captain's shared config file.
        self.settings = settings if settings is not None else DecisionSettings.from_environment()
        self.settings.validate()
        self._blind_fault_count = 0
        self._blind_stop = False

    # -- public entry ----------------------------------------------------

    def run(self, perception):
        output = DecisionTarget()
        try:
            _link_frame(perception, output)
            self._arbitrate(perception, output)
        except Exception as exc:  # never leave a half-filled decision
            output.valid = False
            output.mode = DecisionMode.STOP
            output.target_speed = 0.0
            output.stop_distance = -1.0
            output.reason = "decision error: " + type(exc).__name__ + ": " + str(exc)
            output.errors.append("DECISION_EXCEPTION:" + type(exc).__name__)
        return output

    # -- arbitration -----------------------------------------------------

    def _arbitrate(self, perception, output):
        settings = self.settings
        if not protocol.perception_usable(perception):
            self._blind_fault_count += 1
            self._blind_stop = True
            self._stop(output, "ego frame unavailable, invalid or expired", None, True)
            return

        ego = perception.ego
        ego_speed = float(ego.speed)
        cruising = speed_policy.desired_speed(perception, settings)

        # Traffic control outranks obstacles: a signal applies to the whole
        # lane, while obstacle handling below can only stop earlier.
        traffic = perception.traffic
        if protocol.traffic_requires_stop(traffic):
            self._blind_fault_count = 0
            distance = protocol.traffic_stop_distance(traffic, settings.traffic_stop_margin)
            if traffic.ambiguous:
                reason = "signal group unresolved; stop until the applicable lamp is known"
            elif traffic.observed:
                reason = "signal state {0}".format(traffic.signal_state)
            else:
                reason = "traffic control reported without an observed signal"
            if distance is None:
                self._stop(output, reason + "; stop line position unknown, holding",
                           None, self._braking_required(ego_speed))
            else:
                self._stop(output, reason, distance)
            return

        if not protocol.lane_usable(perception):
            self._blind_fault_count = 0
            if ego_speed > settings.blind_speed_tolerance:
                self._stop(output, "lane unavailable; no steerable reference", None, True)
            else:
                self._stop(output, "lane unavailable; holding at standstill")
            return

        # The vehicle keeps rolling unless the captain's per-source gate says
        # the observations behind a stop decision are trustworthy. Losing the
        # target stream is a loss of evidence, not a clear road.
        if not protocol.targets_usable(perception):
            self._blind_fault_count += 1
            if self._blind_fault_count >= 2:
                self._blind_stop = True
            if self._blind_stop:
                self._stop(output, "target observations unusable; blind stop latched "
                                   "until standstill", None, self._braking_required(ego_speed))
                return

        built = []
        for target in perception.targets:
            candidate = candidate_rules.build_candidate(target, ego, perception)
            if candidate is not None:
                built.append(candidate)
        lead = candidate_rules.select_lead(built)

        if speed_policy.is_emergency(lead, ego_speed, settings):
            self._blind_fault_count = 0
            self._stop(output, "obstacle emergency envelope", 0.0, True)
            return

        if lead is not None:
            self._blind_fault_count = 0
            if lead.clearance is None:
                # The gap behind this target cannot be measured, so neither
                # following nor a safe approach speed can be justified. Stop
                # at the configured minimum gap and say the extent is missing.
                self._stop(output, "same-lane obstacle with unpublished extent; "
                                   "gap unmeasurable, stopping at the minimum gap",
                           speed_policy.obstacle_stop_distance(lead, settings))
                return
            # A map-verified lead always yields a following demand. There is
            # no "too far to follow" state: the speed policy already returns
            # cruise speed when the time gap is wide open.
            self._follow(output, speed_policy.follow_speed(lead, ego, settings), lead, ego_speed)
            return

        unverified = candidate_rules.unverified_close_targets(built, settings)
        if unverified:
            self._blind_fault_count = 0
            nearest = min(unverified, key=lambda item: item.clearance)
            self._stop(output, "obstacle {0:.1f}m away is not map-verified as in-lane; "
                               "lane membership unknown, stopping instead of guessing"
                       .format(nearest.clearance))
            return

        if self._blind_stop:
            if ego_speed > settings.blind_speed_tolerance:
                self._stop(output, "recovering from blind stop; still moving")
                return
            self._blind_stop = False
            self._blind_fault_count = 0

        self._cruise(output, cruising, perception)

    # -- behaviour writers -----------------------------------------------

    def _braking_required(self, ego_speed):
        """Emergency braking is only meaningful when the vehicle is rolling.

        At or near a standstill the same situation is a hold: requesting
        emergency braking would assert a collision risk that does not exist
        and would block a legitimate launch.
        """
        return ego_speed > self.settings.blind_speed_tolerance

    def _cruise(self, output, speed, perception):
        output.mode = DecisionMode.KEEP_LANE
        output.target_speed = float(speed)
        lane_id = perception.lane.lane_id
        output.target_lane_id = lane_id if isinstance(lane_id, str) else ""
        output.stop_distance = -1.0
        limit = protocol.speed_limit(perception)
        if limit is None:
            output.reason = "keep lane at provisional cruise speed; no published speed limit"
        else:
            output.reason = "keep lane at published speed limit"
        output.valid = True

    def _follow(self, output, speed, lead, ego_speed):
        output.mode = DecisionMode.FOLLOW
        output.target_speed = float(speed)
        output.target_lane_id = lead.target.lane_id
        output.stop_distance = -1.0
        if lead.clearance is None:
            gap = "gap unknown (target extent unpublished)"
        else:
            gap = "gap {0:.1f}m".format(lead.clearance)
        if lead.target.ttc >= 0.0:
            gap += ", ttc {0:.1f}s".format(lead.target.ttc)
        else:
            gap += ", ttc unavailable"
        output.reason = "following same-lane lead (map-verified); {0}; " \
                        "ego {1:.1f}m/s".format(gap, ego_speed)
        output.valid = True

    def _stop(self, output, reason, stop_distance=None, emergency=False):
        """Command a stop, optionally at a known distance.

        `stop_distance` stays -1 when the stopping point is unknown, matching
        the public contract where a negative value means "not measured". A
        zero would assert a measured stop point at the current position.
        """
        output.mode = DecisionMode.EMERGENCY_BRAKE if emergency else DecisionMode.STOP
        output.target_speed = 0.0
        output.stop_distance = -1.0 if stop_distance is None else float(stop_distance)
        output.reason = reason
        output.valid = True
