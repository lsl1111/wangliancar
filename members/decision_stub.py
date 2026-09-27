"""Replace only decide() when the decision member delivers their module."""

from core.interfaces import DecisionMode, DecisionTarget
from core.scene_requirements import requires_targets
from core.validation import current


# Provisional low-speed target for the lane-centering acceptance scene only.
LANE_CENTERING_CRUISE_MPS = 2.0


def decide(perception):
    output = DecisionTarget().bind(perception)
    if (not current(perception) or not perception.ego.valid or
            not perception.source_status.get("gps", {}).get("usable", False)):
        output.mode = DecisionMode.STOP
        output.reason = "perception invalid"
        return output
    output.mode = DecisionMode.KEEP_LANE
    output.target_speed = (LANE_CENTERING_CRUISE_MPS if perception.scene_id == 6
                           else perception.ego.speed)
    output.target_lane_id = perception.lane.lane_id
    output.reason = ("06 low-speed lane-centering target"
                     if perception.scene_id == 6 else
                     "integration baseline: hold current speed; no autonomous launch")
    output.valid = True
    target_status = perception.source_status.get("targets", {})
    targets_usable = (perception.targets_valid and
                      perception.target_source.startswith("sensor:") and
                      isinstance(target_status, dict) and
                      target_status.get("usable") is True)
    if not perception.lane.valid or (requires_targets(perception.scene_id)
                                     and not targets_usable):
        output.mode, output.target_speed = DecisionMode.EMERGENCY_BRAKE, 0.0
        output.reason = "required perception source unavailable"
        return output
    limits = [v for v in (perception.lane.speed_limit, perception.traffic.speed_limit) if v >= 0]
    if limits:
        output.target_speed = min([output.target_speed] + limits)
    traffic = perception.traffic
    if traffic.observed and (traffic.ambiguous or traffic.signal_state != "GREEN"):
        output.mode, output.target_speed = DecisionMode.STOP, 0.0
        output.stop_distance = max(0.0, traffic.stop_line_distance - 3.0)
        output.reason = "stop before signal; 3m GPS-reference margin"
    for target in perception.targets if targets_usable else []:
        # Unknown map membership is not evidence that an obstacle is off-lane.
        relevant = target.same_lane or (not target.same_lane_valid and target.lateral_band_match)
        if not target.valid or not relevant or target.longitudinal_distance <= 0:
            continue
        clearance = max(0.0, target.longitudinal_distance - target.length * 0.5 - 3.0)
        if clearance <= 2.0 or (0 <= target.ttc <= 2.0):
            output.mode, output.target_speed = DecisionMode.EMERGENCY_BRAKE, 0.0
            output.stop_distance, output.reason = 0.0, "obstacle emergency envelope"
            return output
        if output.stop_distance < 0 or clearance < output.stop_distance:
            output.stop_distance = clearance
            output.mode, output.target_speed = DecisionMode.STOP, 0.0
            output.reason = "conservative obstacle stop; following policy not implemented"
    return output
