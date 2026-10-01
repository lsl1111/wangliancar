"""Read-only helpers over the public Perception contract.

The decision member owns no sensing. Everything here normalizes what the
captain published and answers one question: is this value usable as fact?
An absent, malformed or stale source is treated as unknown, never as an
empty world and never as zero.
"""

from core.validation import current, number


def perception_usable(perception):
    """Ego frame is present, finite, unexpired and source-authorized.

    GPS gear is raw feedback, not the ControlOut N/D/R/P command enum.
    """
    return (
        current(perception)
        and perception.ego.valid is True
        and source_usable(perception, "gps")
        and all(number(value) for value in (
            perception.ego.x, perception.ego.y, perception.ego.heading,
            perception.ego.speed, perception.ego.vx, perception.ego.vy))
        and perception.ego.speed >= 0.0
        and type(perception.ego.gear) is int
    )


def source_usable(perception, name):
    """Whether a named source published by the captain is trustworthy.

    Only the explicit per-source gate counts. `Perception.valid` describes the
    ego frame, and a readable-but-empty target list must not be read as a
    clear road, so neither is used as a substitute here.
    """
    statuses = getattr(perception, "source_status", None)
    if not isinstance(statuses, dict):
        return False
    status = statuses.get(name)
    if not isinstance(status, dict):
        return False
    return status.get("usable") is True


def targets_usable(perception):
    """Parsed, source-authorized Sensor API observations; Ground Truth excluded."""
    if perception.targets_valid is not True or not isinstance(perception.targets, list):
        return False
    source = perception.target_source
    statuses = getattr(perception, "source_status", {})
    meta = statuses.get("targets", {}) if isinstance(statuses, dict) else {}
    return (isinstance(source, str) and source.startswith("sensor:")
            and source_usable(perception, "targets")
            and isinstance(meta, dict)
            and meta.get("sensor_read_ok") is not False)


def lane_usable(perception):
    return perception.lane.valid is True


def speed_limit(perception):
    """Lowest published speed limit in m/s, or None when none is trustworthy.

    Both fields stay at -1 while no reliable source exists, and a negative
    value must not be read as a limit of zero.
    """
    candidates = []
    for value in (perception.lane.speed_limit, perception.traffic.speed_limit):
        if type(value) in (int, float) and number(value) and value >= 0.0:
            candidates.append(float(value))
    if not candidates:
        return None
    return min(candidates)


def traffic_requires_stop(traffic):
    """Traffic state demands a stop, including an unresolved signal group.

    An observed but ambiguous signal group means the applicable lamp could not
    be identified; treating that as permission to proceed would be guessing.
    """
    if not isinstance(traffic, object):
        return False
    if not traffic.observed:
        return False
    if traffic.ambiguous:
        return True
    state = traffic.signal_state
    if not isinstance(state, str):
        return True
    return state != "GREEN"


def traffic_stop_distance(traffic, margin):
    """Path distance to stop at, or None when the stop line position is unknown."""
    distance = traffic.stop_line_distance
    if type(distance) not in (int, float) or not number(distance) or distance < 0.0:
        return None
    return max(0.0, float(distance) - margin)
