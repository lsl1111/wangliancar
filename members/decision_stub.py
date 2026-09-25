"""Replace only decide() when the decision member delivers their module."""

from core.interfaces import DecisionMode, DecisionTarget


def decide(perception):
    output = DecisionTarget()
    if not perception.valid:
        output.mode = DecisionMode.STOP
        output.reason = "perception invalid"
        return output
    output.mode = DecisionMode.KEEP_LANE
    output.target_speed = perception.ego.speed
    output.target_lane_id = perception.lane.lane_id
    output.reason = "captain placeholder: hold current state"
    output.valid = True
    return output

