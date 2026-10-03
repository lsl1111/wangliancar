"""Shared applicable-signal gates; optional lamp absence is not a fault."""

from core.scene_requirements import SIGNAL_SCENES
from core.validation import number
from core.interfaces import DecisionMode


def traffic_required(perception):
    traffic = getattr(perception, "traffic", None)
    if (getattr(traffic, "required", False) is True
            or getattr(traffic, "observed", False) is True):
        return True
    if (getattr(traffic, "association_valid", False) is True
            and getattr(traffic, "signal_presence", "unknown") == "absent"):
        return False
    return getattr(perception, "scene_id", 0) in SIGNAL_SCENES


def traffic_usable(perception):
    traffic = getattr(perception, "traffic", None)
    statuses = getattr(perception, "source_status", {})
    status = statuses.get("traffic") if isinstance(statuses, dict) else None
    if status is not None and (not isinstance(status, dict)
                               or status.get("usable") is not True
                               or status.get("read_ok") is False
                               or status.get("quality") in
                               ("stale", "regressed", "unsynchronized", "invalid")):
        return False
    return (getattr(traffic, "valid", False) is True
            and getattr(traffic, "observed", False) is True
            and getattr(traffic, "ambiguous", True) is False
            and getattr(traffic, "signal_state", "UNKNOWN") in ("RED", "YELLOW", "GREEN"))


def can_approach_signal(perception):
    """An unresolved lamp may be approached only towards its known stop line.

    Legacy observed stop-line fields retain their previous contract. A new
    producer with explicit source metadata must also verify the association.
    """
    traffic = perception.traffic
    statuses = getattr(perception, 'source_status', {})
    status = statuses.get('traffic') if isinstance(statuses, dict) else None
    return (traffic.observed is True and number(traffic.stop_line_distance)
            and traffic.stop_line_distance >= 0
            and (status is None or (isinstance(status, dict)
                 and status.get('association_valid') is True)))


def bounded_signal_stop(perception, decision):
    """An unavailable lamp permits only a stop before its mapped boundary."""
    return (can_approach_signal(perception)
            and getattr(decision, 'valid', False) is True
            and decision.mode in (DecisionMode.STOP, DecisionMode.EMERGENCY_BRAKE,
                                  DecisionMode.KEEP_LANE, DecisionMode.FOLLOW)
            and number(decision.stop_distance)
            and 0 <= decision.stop_distance <= perception.traffic.stop_line_distance)
