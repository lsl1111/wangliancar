"""Bounded lane-reference preview for integration, not a competition planner.

No lane changes, route stitching or reverse support. All distances reference GPS.
STOP profiles use a 2 m/s^2 preview envelope; physical tuning remains pending.
"""

import math

from core.geometry import project_polyline, normalize_angle
from core.interfaces import DecisionMode, Trajectory, TrajectoryPoint, DecisionTarget
from core.validation import current, validate_output


def stationary(output, ego, emergency, reason):
    output.emergency_stop = emergency
    output.stop_required = True
    output.target_speed = 0.0
    output.points = [TrajectoryPoint(ego.x, ego.y, 0, ego.heading, 0),
                     TrajectoryPoint(ego.x, ego.y, 0, ego.heading, 0.1)]
    output.valid = True
    output.reason = reason
    return output


def build_preview(perception, decision):
    output = Trajectory().bind(decision)
    output.target_lane_id = decision.target_lane_id
    output.stop_distance = decision.stop_distance
    output.emergency_stop = decision.mode == DecisionMode.EMERGENCY_BRAKE
    output.stop_required = decision.mode in (DecisionMode.STOP, DecisionMode.EMERGENCY_BRAKE)
    try:
        validate_output(decision, DecisionTarget, perception)
        if not current(perception) or not perception.ego.valid:
            raise ValueError("perception unavailable")
        if output.emergency_stop:
            return stationary(output, perception.ego, True, "emergency stop requested")
        if not perception.lane.valid:
            raise ValueError("map lane unavailable")
        if decision.target_lane_id and decision.target_lane_id != perception.lane.lane_id:
            raise ValueError("target lane geometry unavailable; lane change unsupported")
        ego = perception.ego
        if (not all(math.isfinite(v) for v in (ego.vx, ego.vy, ego.heading)) or
                ego.vx * math.cos(ego.heading) + ego.vy * math.sin(ego.heading) < -1e-6):
            raise ValueError("reverse trajectory unsupported")
        points = []
        for point in perception.lane.center_line:
            x, y = point[:2]
            if not all(math.isfinite(v) for v in (x, y)):
                raise ValueError("nonfinite centerline")
            if not points or math.hypot(x - points[-1][0], y - points[-1][1]) > 1e-6:
                points.append((x, y))
        ego = perception.ego
        projection = project_polyline(points, ego.x, ego.y)
        if projection is None:
            raise ValueError("degenerate centerline")
        if (projection["distance"] > perception.lane.lane_width or
                abs(normalize_angle(projection["heading"] - ego.heading)) > math.pi / 2):
            raise ValueError("reference does not match vehicle pose")
        if projection["index"] == len(points) - 2 and projection["raw_ratio"] >= 1:
            return stationary(output, ego, True, "end of mapped reference")
        path = [projection["point"]] + points[projection["index"] + 1:]
        remaining = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(path, path[1:]))
        if remaining < 1e-6:
            return stationary(output, ego, True, "end of mapped reference")
        stop = remaining  # Always stop at the end of known map coverage.
        if output.stop_required:
            stop = max(0.0, decision.stop_distance)
        stop = min(stop, remaining)
        output.stop_distance = stop
        output.stop_required = output.stop_required or remaining <= 80.0
        if stop <= 0 or (decision.target_speed == 0 and not output.stop_required):
            return stationary(output, ego, output.stop_required and ego.speed > 0, "stationary request")
        if output.stop_required and ego.speed * ego.speed > 4.0 * stop:
            return stationary(output, ego, True, "stop exceeds preview deceleration envelope")
        desired = ego.speed if decision.mode == DecisionMode.STOP else decision.target_speed
        if desired <= 0:
            return stationary(output, ego, False, "already stopped")
        horizon = min(80.0, stop)
        samples = [(0.0, path[0])]
        along = 0.0
        for a, b in zip(path, path[1:]):
            distance = math.hypot(b[0]-a[0], b[1]-a[1])
            if distance <= 1e-6:
                continue
            offset = min(1.0, distance)
            while along + offset < horizon - 1e-6:
                samples.append((along + offset, (a[0]+(b[0]-a[0])*offset/distance,
                                                a[1]+(b[1]-a[1])*offset/distance)))
                if offset >= distance:
                    break
                offset = min(offset + 1.0, distance)
            if along + distance >= horizon:
                ratio = (horizon - along) / distance
                samples.append((horizon, (a[0]+(b[0]-a[0])*ratio, a[1]+(b[1]-a[1])*ratio)))
                break
            along += distance
        elapsed = 0.0
        for i, (s, point) in enumerate(samples):
            speed = min(desired, math.sqrt(max(0.0, 4.0 * (stop - s))))
            if i:
                previous = output.points[-1]
                ds = math.hypot(point[0]-previous.x, point[1]-previous.y)
                if speed + previous.speed <= 1e-9:
                    break
                elapsed += 2.0 * ds / (speed + previous.speed)
            heading = (math.atan2(samples[i+1][1][1]-point[1], samples[i+1][1][0]-point[0])
                       if i+1 < len(samples) else output.points[-1].heading if output.points else ego.heading)
            output.points.append(TrajectoryPoint(point[0], point[1], speed, heading, elapsed))
        output.target_speed = 0.0 if decision.mode == DecisionMode.STOP else desired
        output.valid = len(output.points) >= 2
        output.reason = "lane reference preview; vehicle tracking not validated"
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        output.valid = False
        output.reason = str(exc)
        output.errors.append(str(exc))
    return output
