"""Speed and distance demands for each behaviour.

Pure functions over normalized inputs. Nothing here reads the SDK, mutates
its arguments or decides which behaviour applies; the engine owns arbitration.
"""

import math

from core.geometry import project_polyline

from members.decision import protocol


def desired_speed(perception, settings):
    """Cruise demand, lowered by any trustworthy published speed limit."""
    limit = protocol.speed_limit(perception)
    if limit is None:
        return float(settings.cruise_speed)
    return min(float(settings.cruise_speed), float(limit))


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def lead_forward_speed(target, ego):
    """Lead vehicle speed projected on the ego vehicle's forward axis.

    `Target.relative_speed` is the closing speed published by the captain, so
    the lead speed is derived from its absolute velocity instead. A negative
    projection means the lead vehicle is coming towards us; that is reported
    as zero here and left to the clearance and time-to-collision rules.
    """
    if not (_finite(target.vx) and _finite(target.vy) and _finite(ego.heading)):
        return 0.0
    projected = math.cos(ego.heading) * target.vx + math.sin(ego.heading) * target.vy
    return max(0.0, projected)


def is_emergency(candidate, ego_speed, settings):
    """Whether a lead vehicle is close enough to require braking.

    The time-to-collision rule is suppressed below a minimum speed: at a
    standstill the closing speed is not meaningful, and a small time gap
    reflects parking distance rather than a collision risk. Using raw TTC
    there would demand an emergency stop at the moment the vehicle tries to
    launch, so the threshold is the speed at which the configured TTC
    corresponds to the configured clearance.
    """
    if candidate is None:
        return False
    if candidate.clearance is not None and candidate.clearance <= settings.emergency_clearance:
        return True
    if not math.isfinite(candidate.ttc) or candidate.ttc < 0.0:
        return False
    minimum_speed = settings.emergency_clearance / settings.emergency_ttc
    if ego_speed < minimum_speed:
        return False
    return candidate.ttc <= settings.emergency_ttc


def follow_speed(lead, ego, settings):
    """Target speed behind a lead vehicle.

    The lead vehicle's own speed is always the ceiling: this member does not
    drive faster than the vehicle it is following in order to close a gap.
    Below the ceiling, the approach is bounded by a proportional gap demand.
    """
    lead_speed = lead_forward_speed(lead.target, ego)
    if lead.clearance is None:
        # Target extent unknown: the gap cannot be measured, so only matching
        # the lead speed is defensible. Braking distance is not guessed.
        return min(lead_speed, settings.cruise_speed)
    safe_gap = settings.min_gap + settings.time_headway * lead_speed
    if lead.clearance <= safe_gap:
        return min(lead_speed, 0.0)
    demand = (lead.clearance - safe_gap) / max(settings.time_headway, 1e-6)
    return max(0.0, min(lead_speed, lead_speed + settings.resume_margin, demand))


def obstacle_stop_distance(candidate, settings):
    """Path distance to stop before an obstacle.

    `decision.stop_distance` is read by the planner as a distance from the
    GPS reference point along the lane, so the margin is subtracted here.
    When extent or offset is unknown a fixed gap is used and the caller says
    so in the reason text, rather than driving on an unmeasured clearance.
    """
    if candidate.clearance is None:
        return max(0.0, float(settings.min_gap))
    return max(0.0, candidate.clearance - settings.obstacle_stop_margin)


def remaining_reference(perception, ego):
    """Distance to the end of the known lane centre line, or None.

    The planner owns map ends, so this is only used to qualify the reason
    text; the decision member never invents a stop for a map end.
    """
    points = perception.lane.center_line
    if not isinstance(points, (list, tuple)) or len(points) < 2:
        return None
    projection = project_polyline(points, ego.x, ego.y)
    if projection is None:
        return None
    total = 0.0
    for a, b in zip(points, points[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1])
    return max(0.0, total - projection["s"])
