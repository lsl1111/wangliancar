"""Decision tuning parameters. Python 3.6; standard library only.

Every value here is a provisional experiment setting, not a measured vehicle
capability or a competition threshold. See BASELINE.md.

Field tuning can override any parameter without a code change by setting an
environment variable. This keeps the shared `config/default.ini`, which the
captain owns, untouched:

    NEVC_DECISION_CRUISE_SPEED=11.1
    NEVC_DECISION_TIME_HEADWAY=1.5
"""

import math
import os


# Environment variable prefix for provisional tuning overrides.
ENV_PREFIX = "NEVC_DECISION_"

FIELDS = ("cruise_speed", "min_gap", "time_headway", "resume_margin",
          "emergency_clearance", "emergency_ttc", "launch_ttc_cap",
          "stop_margin", "obstacle_stop_margin", "traffic_stop_margin",
          "blind_speed_tolerance")


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _parse(name, raw):
    """Parse one override, rejecting rather than silently ignoring bad input."""
    text = str(raw).strip()
    if not text:
        raise ValueError("decision override {0} is empty".format(name))
    try:
        value = float(text)
    except ValueError:
        raise ValueError("decision override {0}={1!r} is not a number".format(name, raw))
    if not math.isfinite(value):
        raise ValueError("decision override {0}={1!r} is not finite".format(name, raw))
    return value


def overrides_from_environment(environ=None):
    """Return the parameter overrides present in the environment."""
    environ = os.environ if environ is None else environ
    values = {}
    for name in FIELDS:
        key = ENV_PREFIX + name.upper()
        if key in environ:
            values[name] = _parse(key, environ[key])
    return values


class DecisionSettings(object):
    def __init__(self, cruise_speed=8.0, min_gap=4.0, time_headway=1.2,
                 resume_margin=2.0, emergency_clearance=2.0, emergency_ttc=2.0,
                 launch_ttc_cap=6.0, stop_margin=3.0,
                 obstacle_stop_margin=3.0, traffic_stop_margin=3.0,
                 blind_speed_tolerance=0.5):
        # Desired speed used when no trustworthy speed limit is published.
        # Perception keeps lane/traffic speed limit at -1 until a real source
        # exists, so this value is the only speed demand the member owns.
        self.cruise_speed = cruise_speed
        # Clearance kept behind a lead vehicle at a standstill (m).
        self.min_gap = min_gap
        # Time gap kept behind a moving lead vehicle (s).
        self.time_headway = time_headway
        # Extra distance beyond the raw safe gap before following resumes.
        self.resume_margin = resume_margin
        # Below this clearance the lead vehicle counts as an emergency (m).
        self.emergency_clearance = emergency_clearance
        # Below this time-to-collision the lead vehicle counts as an
        # emergency (s), once the ego vehicle is actually moving.
        self.emergency_ttc = emergency_ttc
        # Time-to-collision is meaningless at a standstill, so the emergency
        # rule is disabled below the speed that makes the TTC reach this cap.
        self.launch_ttc_cap = launch_ttc_cap
        # Declared stop distance is measured from the GPS reference point, so
        # the occupant margin is subtracted here (provisional responsibility).
        self.stop_margin = stop_margin
        self.obstacle_stop_margin = obstacle_stop_margin
        self.traffic_stop_margin = traffic_stop_margin
        # While target observations are unusable, resume only below this speed.
        self.blind_speed_tolerance = blind_speed_tolerance

    def validate(self):
        for name in FIELDS:
            value = getattr(self, name)
            if not _number(value) or value < 0.0:
                raise ValueError("decision setting {0} must be finite and nonnegative".format(name))
        if self.cruise_speed <= 0.0:
            raise ValueError("cruise_speed must be positive")
        if self.emergency_ttc <= 0.0 or self.launch_ttc_cap <= 0.0:
            raise ValueError("time-to-collision limits must be positive")
        if self.resume_margin <= 0.0:
            raise ValueError("resume_margin must be positive")
        return self

    def replace(self, **overrides):
        """Return a validated copy with the named parameters replaced."""
        values = dict((name, getattr(self, name)) for name in FIELDS)
        for name, value in overrides.items():
            if name not in values:
                raise ValueError("unknown decision setting: " + str(name))
            values[name] = value
        return DecisionSettings(**values).validate()

    @classmethod
    def from_environment(cls, environ=None, **overrides):
        """Build settings from the environment, then apply explicit overrides."""
        values = overrides_from_environment(environ)
        values.update(overrides)
        return cls().replace(**values)
