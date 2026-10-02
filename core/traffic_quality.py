"""Shared applicable-signal gates; optional lamp absence is not a fault."""

from core.scene_requirements import SIGNAL_SCENES


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
