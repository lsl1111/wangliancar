"""Fixed planning entry for the clear-road lane-reference baseline."""

from members.planning.lane_planner import build_trajectory


def plan(perception, decision):
    return build_trajectory(perception, decision)
