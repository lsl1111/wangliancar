"""Decision member entry logic: choose a behaviour and a numeric demand.

Input is the captain's `Perception`; output is a `DecisionTarget`. This member
never touches the SDK, never outputs actuators and never plans geometry.

Design rules, in priority order:

1. Unknown is not safe. A source the captain marked unusable is treated as
   absent evidence, not as an empty road.
2. Fail safe, not silent. Every degraded decision says why in `reason`.
3. Latched safety. A blind-stop is released only after required evidence
   returns and the vehicle is slow enough to resume.
4. No silent behaviour invention. Situations this member cannot resolve
   (unverified lane membership, ambiguous signal groups, shared-lane
   obstacles that would need steering) become a documented stop.
"""

from core.interfaces import DecisionMode, DecisionTarget
from core.scene_requirements import requires_targets

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
        self.reset()

    def reset(self):
        """Discard state belonging to an earlier simulation run."""
        self._blind_fault_count = 0
        self._blind_stop = False
        self._context = None
        self._last_frame = None

    def _observe_frame(self, perception):
        """Count source frames once and reset state when a new run is seen."""
        context = (getattr(perception, "case_id", ""),
                   getattr(perception, "task_id", ""),
                   getattr(perception, "scene_id", 0))
        frame = (getattr(perception, "frame_id", -1),
                 getattr(perception, "timestamp", 0))
        frame_id = frame[0]
        rolled_back = (self._last_frame is not None
                       and type(frame_id) is int and frame_id >= 0
                       and type(self._last_frame[0]) is int
                       and frame_id < self._last_frame[0])
        if context != self._context or rolled_back:
            self.reset()
            self._context = context
        distinct = frame != self._last_frame
        self._last_frame = frame
        return distinct

    # -- public entry ----------------------------------------------------

    def run(self, perception):
        output = DecisionTarget()
        try:
            _link_frame(perception, output)
            self._arbitrate(perception, output, self._observe_frame(perception))
        except Exception as exc:  # never leave a half-filled decision
            output.valid = False
            output.mode = DecisionMode.STOP
            output.target_speed = 0.0
            output.stop_distance = -1.0
            output.reason = "decision error: " + type(exc).__name__ + ": " + str(exc)
            output.errors.append("DECISION_EXCEPTION:" + type(exc).__name__)
        return output

    # -- arbitration -----------------------------------------------------

    def _arbitrate(self, perception, output, distinct_frame):
        settings = self.settings
        if not protocol.perception_usable(perception):
            if distinct_frame:
                self._blind_fault_count += 1
            self._blind_stop = True
            self._stop(output, "ego frame unavailable, invalid or expired", None, True)
            return

        ego = perception.ego
        ego_speed = float(ego.speed)
        cruising = speed_policy.desired_speed(perception, settings)

        # Only required sensor streams can latch a blind stop. Optional
        # streams may be absent in lane-only scenes, and stale lists never
        # count as obstacle evidence.
        targets_fresh = protocol.targets_usable(perception)
        targets_required = requires_targets(perception.scene_id)
        if targets_required and not targets_fresh:
            if distinct_frame:
                self._blind_fault_count += 1
            if self._blind_fault_count >= 2:
                self._blind_stop = True
        else:
            self._blind_fault_count = 0

        if self._blind_stop:
            if targets_required and not targets_fresh:
                self._stop(output, "target observations unusable; blind stop latched "
                                   "until standstill", None, self._braking_required(ego_speed))
                return
            if ego_speed > settings.blind_speed_tolerance:
                self._stop(output, "recovering from blind stop; still moving", None, True)
                return
            self._blind_stop = False

        if not protocol.lane_usable(perception):
            if ego_speed > settings.blind_speed_tolerance:
                self._stop(output, "lane unavailable; no steerable reference", None, True)
            else:
                self._stop(output, "lane unavailable; holding at standstill")
            return

        built = []
        for target in perception.targets if targets_fresh else []:
            candidate = candidate_rules.build_candidate(target, ego, perception)
            if candidate is not None:
                built.append(candidate)
        lead = candidate_rules.select_lead(built)
        unverified = candidate_rules.unverified_close_targets(built, settings)

        if speed_policy.is_emergency(lead, ego_speed, settings):
            self._stop(output, "obstacle emergency envelope", 0.0, True)
            return

        traffic = perception.traffic
        if protocol.traffic_requires_stop(traffic):
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
                return
            obstacles = ([lead] if lead is not None else []) + unverified
            if obstacles:
                nearest = min(obstacles, key=lambda item:
                              speed_policy.obstacle_stop_distance(item, settings))
                obstacle_distance = speed_policy.obstacle_stop_distance(nearest, settings)
                if obstacle_distance < distance:
                    distance = obstacle_distance
                    reason += "; nearer obstacle requires an earlier stop"
            self._stop(output, reason, distance)
            return

        if targets_required and not targets_fresh:
            # The first failed frame does not latch a stop, but it cannot
            # authorize acceleration or reuse the stale target list.
            self._cruise(output, min(cruising, ego_speed), perception)
            output.reason = "target observations temporarily unusable; acceleration held"
            return

        if lead is not None:
            if lead.clearance is None:
                self._stop(output, "same-lane obstacle with unpublished extent; "
                                   "gap unmeasurable, stopping at the minimum gap",
                           speed_policy.obstacle_stop_distance(lead, settings))
                return
            self._follow(output, speed_policy.follow_speed(lead, ego, cruising, settings),
                         lead, ego_speed)
            return

        if unverified:
            nearest = min(unverified, key=lambda item: item.clearance)
            self._stop(output, "obstacle {0:.1f}m away is not map-verified as in-lane; "
                               "lane membership unknown, stopping instead of guessing"
                       .format(nearest.clearance),
                       speed_policy.obstacle_stop_distance(nearest, settings))
            return

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
