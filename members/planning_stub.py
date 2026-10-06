"""Fixed planning entry for the lane reference and guarded obstacles."""

import math

from members.planning.lane_planner import build_trajectory, PlannerSettings
from core.interfaces import DecisionTarget, Trajectory


_CONTROL_GEOMETRY = {}


def _settings_with_control_geometry():
    settings = PlannerSettings.from_environment()
    for name, value in _CONTROL_GEOMETRY.items():
        explicit = getattr(settings, name)
        if explicit is not None and not math.isclose(explicit, value, rel_tol=1e-9, abs_tol=1e-9):
            raise ValueError("planning/control vehicle geometry differs: " + name)
        setattr(settings, name, value)
    return settings


def configure_planning(config=None):
    """Captain supplies the same configured steering geometry as control.

    This is an explicit trial capability, not a claim of live calibration.
    Body extents continue to come from the shared NEVC_VEHICLE_* overrides.
    """
    global _CONTROL_GEOMETRY
    geometry = {}
    if config is not None and getattr(config, "control_calibrated", False):
        for name in ("wheelbase_m", "front_steer_max_rad"):
            value = getattr(config, "control_" + name, None)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError("configured control vehicle geometry is unavailable: " + name)
            geometry[name] = float(value)
    _CONTROL_GEOMETRY = geometry
    settings = _settings_with_control_geometry()
    settings.validate()
    return settings


def plan(perception, decision):
    try:
        settings = _settings_with_control_geometry()
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        output = Trajectory()
        if isinstance(decision, DecisionTarget):
            output.bind(decision)
        output.reason = str(exc)
        output.errors.append(str(exc))
        return output
    return build_trajectory(perception, decision, settings)
