"""Bounded, provisional decision parameters (Python 3.6, standard library)."""

import math
import os


ENV_PREFIX = "NEVC_DECISION_"
FIELDS = (
    "cruise_speed", "min_gap", "time_headway", "gap_gain", "resume_margin",
    "hold_distance",
    "follow_deceleration", "reaction_time", "front_offset_m",
    "emergency_clearance", "emergency_ttc", "static_speed_threshold",
    "standstill_speed", "blind_speed_tolerance", "traffic_stop_margin",
    "obstacle_stop_margin", "projection_tolerance_m", "route_ambiguity_m",
    "conflict_horizon_s", "recovery_frames", "release_frames",
)
INT_FIELDS = ("recovery_frames", "release_frames")
DEPRECATED = ("launch_ttc_cap", "stop_margin")


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _parse(name, raw, integer=False):
    value = str(raw).strip()
    if integer:
        if not value.isdigit():
            raise ValueError("decision override {0} must be a positive integer".format(name))
        return int(value)
    try:
        result = float(value)
    except ValueError:
        raise ValueError("decision override {0} is not numeric".format(name))
    if not math.isfinite(result):
        raise ValueError("decision override {0} is not finite".format(name))
    return result


def overrides_from_environment(environ=None):
    environ = os.environ if environ is None else environ
    for name in DEPRECATED:
        key = ENV_PREFIX + name.upper()
        if key in environ:
            raise ValueError("decision override {0} is retired".format(key))
    values = {}
    if "NEVC_VEHICLE_FRONT_OFFSET_M" in environ:
        values["front_offset_m"] = _parse("NEVC_VEHICLE_FRONT_OFFSET_M",
                                          environ["NEVC_VEHICLE_FRONT_OFFSET_M"])
    for name in FIELDS:
        key = ENV_PREFIX + name.upper()
        if key in environ:
            values[name] = _parse(key, environ[key], name in INT_FIELDS)
    return values


class DecisionSettings(object):
    def __init__(self, cruise_speed=30.0 / 3.6, min_gap=10.5,
                 time_headway=1.2, gap_gain=0.5, resume_margin=2.0,
                 hold_distance=0.05,
                 follow_deceleration=2.0, reaction_time=0.3,
                 front_offset_m=None, emergency_clearance=2.0,
                 emergency_ttc=2.0, static_speed_threshold=0.3,
                 standstill_speed=0.1, blind_speed_tolerance=0.5,
                 traffic_stop_margin=0.3, obstacle_stop_margin=0.5,
                 projection_tolerance_m=2.5, route_ambiguity_m=2.0,
                 conflict_horizon_s=3.0, recovery_frames=3,
                 release_frames=2):
        for name in FIELDS:
            setattr(self, name, locals()[name])

    def validate(self):
        for name in FIELDS:
            value = getattr(self, name)
            if name in INT_FIELDS:
                if type(value) is not int or value < 1:
                    raise ValueError("decision setting {0} must be a positive integer".format(name))
                continue
            if name == "front_offset_m" and value is None:
                continue
            if not _finite(value) or value < 0.0:
                raise ValueError("decision setting {0} must be finite and nonnegative".format(name))
        for name in ("cruise_speed", "time_headway", "gap_gain", "resume_margin", "follow_deceleration",
                     "emergency_ttc", "static_speed_threshold", "blind_speed_tolerance",
                     "projection_tolerance_m", "route_ambiguity_m", "conflict_horizon_s"):
            if getattr(self, name) <= 0.0:
                raise ValueError("decision setting {0} must be positive".format(name))
        if self.front_offset_m is not None and self.front_offset_m <= 0.0:
            raise ValueError("measured front offset must be positive")
        if self.standstill_speed >= self.blind_speed_tolerance:
            raise ValueError("standstill_speed must be below blind_speed_tolerance")
        if self.hold_distance >= self.resume_margin:
            raise ValueError("hold_distance must be below resume_margin")
        return self

    def replace(self, **overrides):
        values = dict((name, getattr(self, name)) for name in FIELDS)
        for name, value in overrides.items():
            if name not in values:
                raise ValueError("unknown decision setting: " + str(name))
            values[name] = value
        return DecisionSettings(**values).validate()

    @classmethod
    def from_environment(cls, environ=None, **overrides):
        values = overrides_from_environment(environ)
        values.update(overrides)
        return cls().replace(**values)
