"""Consume verified upcoming HDMap limits; no sign decoding or behaviour policy."""

import math

from core.validation import number


def upcoming_limits(perception, settings):
    """Return rear-axle distances at which the vehicle front must obey a limit.

    The producer has checked type, unit, scope and orientation. Previously
    passed limits remain the decision member's current-speed responsibility.
    Upcoming limits are retained even beyond the current planning horizon.
    """
    observations = getattr(perception, "speed_limit_observations", [])
    if not isinstance(observations, list) or len(observations) > 2000:
        raise ValueError("invalid or oversized speed-limit observations")
    limits = []
    for item in observations:
        if not isinstance(item, dict):
            raise ValueError("malformed speed-limit observation")
        if item.get("applicable") is not True or item.get("source") != "hdmap":
            continue
        distance, speed = item.get("along_distance"), item.get("speed_mps")
        active = item.get("active")
        if (perception.traffic_signs_valid is not True
                or not number(distance) or not number(speed) or speed <= 0
                or type(active) is not bool or active != (distance <= 0)):
            raise ValueError("unusable applicable speed-limit observation")
        if active:
            continue
        if settings.front_offset_m is None:
            raise ValueError("upcoming speed limit requires measured vehicle front offset")
        limits.append((max(0.0, distance - settings.front_offset_m), speed))
    return sorted(limits)


def speed_caps(samples, limits, deceleration):
    """Bound v^2 before every sign and retain its limit beyond that position.

    Keeping the lower cap through the horizon is conservative: this module
    does not infer a limit cancellation or authorize acceleration at a later
    sign. Exact sign positions are inserted into the sampling grid upstream.
    """
    # Ahead of a sign, every v^2 ceiling has the same slope -2*a. Behind
    # one, its ceiling is constant. Prefix/suffix minima avoid a point-by-sign
    # cross product at the bounded 2000-point input size.
    suffix = [float("inf")] * (len(limits) + 1)
    for index in range(len(limits) - 1, -1, -1):
        distance, speed = limits[index]
        suffix[index] = min(suffix[index + 1], speed * speed +
                            2.0 * deceleration * distance)
    cursor, passed, caps = 0, float("inf"), []
    for s, _ in samples:
        while cursor < len(limits) and limits[cursor][0] <= s:
            passed = min(passed, limits[cursor][1] ** 2)
            cursor += 1
        square = min(passed, suffix[cursor] - 2.0 * deceleration * s)
        caps.append(math.sqrt(max(0.0, square)))
    return caps
