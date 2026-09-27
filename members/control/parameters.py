"""Explicit vehicle calibration and bounded algorithm settings."""

import math


def _positive(name, value):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(name + " must be finite and positive")
    return float(value)


class VehicleCalibration(object):
    """All values must come from a vehicle-specific calibration run."""

    def __init__(self, wheelbase_m, front_steer_max_rad, steering_sign,
                 throttle_per_mps, brake_per_mps, hold_brake,
                 emergency_brake, max_throttle, max_brake):
        self.wheelbase_m = _positive("wheelbase_m", wheelbase_m)
        self.front_steer_max_rad = _positive("front_steer_max_rad", front_steer_max_rad)
        if self.front_steer_max_rad >= math.pi / 2:
            raise ValueError("front_steer_max_rad outside physical range")
        if steering_sign not in (-1, 1) or type(steering_sign) is not int:
            raise ValueError("steering_sign must be -1 or 1")
        self.steering_sign = steering_sign
        self.throttle_per_mps = _positive("throttle_per_mps", throttle_per_mps)
        self.brake_per_mps = _positive("brake_per_mps", brake_per_mps)
        self.hold_brake = _positive("hold_brake", hold_brake)
        self.emergency_brake = _positive("emergency_brake", emergency_brake)
        self.max_throttle = _positive("max_throttle", max_throttle)
        self.max_brake = _positive("max_brake", max_brake)
        if any(value > 1.0 for value in (self.hold_brake, self.emergency_brake,
                                         self.max_throttle, self.max_brake)):
            raise ValueError("calibrated pedal fraction must be <= 1")
        if self.hold_brake > self.max_brake or self.emergency_brake < self.max_brake:
            raise ValueError("brake ordering must be hold <= normal <= emergency")

    @classmethod
    def from_app_config(cls, config):
        if not getattr(config, "control_calibrated", False):
            return None
        names = ("wheelbase_m", "front_steer_max_rad", "steering_sign",
                 "throttle_per_mps", "brake_per_mps", "hold_brake",
                 "emergency_brake", "max_throttle", "max_brake")
        values = [getattr(config, "control_" + name, None) for name in names]
        if any(value is None for value in values):
            raise ValueError("control_calibrated requires all control_* vehicle parameters")
        return cls(*values)


class ControllerSettings(object):
    """Algorithm bounds; vehicle-specific pedal mapping lives in calibration."""

    def __init__(self):
        self.lookahead_base_m = 2.0
        self.lookahead_time_s = 0.7
        self.lookahead_max_m = 10.0
        self.max_segment_m = 12.0
        self.max_projection_error_m = 2.0
        self.max_heading_error_rad = math.pi / 2
        self.path_end_margin_m = 0.5
        self.preview_decel_mps2 = 2.0
        self.max_track_speed_mps = 8.0
        self.pi_kp = 1.0
        self.pi_ki = 0.25
        self.integral_limit = 3.0
        self.control_deadband_mps = 0.08
        self.hold_speed_mps = 0.2
        self.hold_release_speed_mps = 0.5
        self.hold_release_frames = 2
        self.steering_slew_per_s = 1.5
        self.throttle_slew_per_s = 1.0
        self.brake_slew_per_s = 1.5
        self.max_dt_s = 0.5
