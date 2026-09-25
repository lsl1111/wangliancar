"""Replace only plan() when the planning member delivers their module."""

import math

from core.interfaces import Trajectory, TrajectoryPoint


def plan(perception, decision):
    output = Trajectory()
    if not perception.valid or not decision.valid or not perception.lane.valid:
        return output
    center_line = perception.lane.center_line[:80]
    for index, point in enumerate(center_line):
        if index + 1 < len(center_line):
            following = center_line[index + 1]
            heading = math.atan2(following[1] - point[1], following[0] - point[0])
        else:
            heading = perception.ego.heading
        output.points.append(
            TrajectoryPoint(point[0], point[1], decision.target_speed, heading, index * 0.1)
        )
    output.target_speed = decision.target_speed
    output.valid = len(output.points) >= 2
    return output

