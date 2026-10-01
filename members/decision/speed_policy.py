"""Pure longitudinal demands over route-relative decision constraints."""

import math

from members.decision import protocol


def desired_speed(perception, settings):
    limit = protocol.speed_limit(perception)
    return float(settings.cruise_speed) if limit is None else min(
        float(settings.cruise_speed), float(limit))


def is_emergency(candidate, ego_speed, settings):
    """Use signed closing speed through the candidate TTC, even at ego rest."""
    if candidate is None or not candidate.conflict or candidate.gap is None:
        return False
    if (candidate.gap <= settings.emergency_clearance
            and (ego_speed > settings.standstill_speed
                 or candidate.lead_speed < -settings.static_speed_threshold)):
        return True
    return candidate.ttc >= 0.0 and candidate.ttc <= settings.emergency_ttc


def follow_speed(candidate, ego_speed, cruising, settings):
    """Continuous gap feedback for an in-route forward-moving lead."""
    if candidate.gap is None or candidate.lead_speed is None:
        return 0.0
    desired_gap = settings.min_gap + settings.time_headway * ego_speed
    raw = max(0.0, candidate.lead_speed) + settings.gap_gain * (
        candidate.gap - desired_gap)
    return min(float(cruising), max(0.0, raw))


def approach_speed(stop_distance, ego_speed, cruising, settings):
    """Demand no faster than a provisional stop-distance envelope."""
    usable = max(0.0, stop_distance - ego_speed * settings.reaction_time)
    cap = math.sqrt(2.0 * settings.follow_deceleration * usable)
    return min(float(cruising), cap)


def stop_is_infeasible(stop_distance, ego_speed, settings):
    needed = (ego_speed * settings.reaction_time +
              ego_speed * ego_speed / (2.0 * settings.follow_deceleration))
    return ego_speed > settings.standstill_speed and stop_distance + 1e-6 < needed
