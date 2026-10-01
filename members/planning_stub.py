"""Fixed planning entry for the lane reference and guarded obstacles."""

from members.planning.lane_planner import build_trajectory


def plan(perception, decision):
    return build_trajectory(perception, decision)
