"""Select the obstacle that actually constrains this vehicle.

Two independent questions are answered separately:

1. Lane membership - which lane the target is in. Only a map-verified
   answer counts. `same_lane` is the captain's lateral-band fallback and is
   deliberately not treated as a lane decision; a wrong lane guess would
   silently discard a real obstacle.
2. Threat - how urgently that target constrains us. Unknown threat is never
   promoted to a stop. It is recorded in the reason text so a reviewer can
   see which values were missing.
"""

import math

from core.geometry import calculate_ttc, normalize_angle


# Longitudinal speed of the target along the ego vehicle's forward axis.
def _forward_speed(target, ego):
    return math.cos(ego.heading) * target.vx + math.sin(ego.heading) * target.vy


def target_in_lane(target, perception):
    """True only for a map-verified same-lane target.

    Without map verification the target is not claimed to be in lane. Callers
    keep it visible and require an explicit traffic-control decision.
    """
    return target.same_lane_valid is True and target.lane_id == perception.lane.lane_id


class Candidate(object):
    """A target plus the geometry the behaviour layer needs."""

    def __init__(self, target, clearance, ttc, in_lane):
        self.target = target
        self.clearance = clearance
        self.ttc = ttc
        self.in_lane = in_lane


def _clearance(target):
    """Bumper-to-bumper gap along the lane, or None when the target extent is unknown."""
    length = target.length
    if type(length) not in (int, float) or not math.isfinite(length) or length <= 0.0:
        return None
    return max(0.0, float(target.longitudinal_distance) - float(length) * 0.5)


def build_candidate(target, ego, perception):
    """Return a Candidate, or None when the target is not valid evidence of an obstacle."""
    if target.valid is not True:
        return None
    if not (math.isfinite(target.longitudinal_distance)
            and math.isfinite(target.lateral_distance)):
        return None
    # A target behind or beside the reference point cannot be a lead vehicle.
    if target.longitudinal_distance <= 0.0:
        return None
    ttc, _ = calculate_ttc(target.longitudinal_distance, ego.speed,
                           target.vx, target.vy, ego.heading)
    return Candidate(target, _clearance(target), ttc, target_in_lane(target, perception))


def select_lead(candidates):
    """Nearest in-lane candidate ahead, or None."""
    in_lane = []
    for candidate in candidates:
        if not candidate.in_lane:
            continue
        in_lane.append(candidate)
    if not in_lane:
        return None
    return min(in_lane, key=lambda item: item.target.longitudinal_distance)


def unverified_close_targets(candidates, settings):
    """Off-lane targets that are too close to ignore.

    These cannot be resolved into drive-or-stop by this member: steering
    around them is a planning decision, and the road is not proven clear.
    The caller treats them as a stop and states the limitation.
    """
    close = []
    for candidate in candidates:
        if candidate.in_lane:
            continue
        if candidate.clearance is None:
            continue
        if candidate.clearance <= settings.min_gap:
            close.append(candidate)
    return close


def heading_mismatch(target):
    """Angle between the target heading and the ego forward axis, in radians.

    Returns None when the captain did not publish a usable relative heading.
    Used only to annotate the reason text: when the heading is unknown the
    target is still treated as an obstacle, just a less predictable one.
    """
    relative = target.relative_heading
    if type(relative) not in (int, float) or not math.isfinite(relative):
        return None
    return abs(normalize_angle(float(relative)))
