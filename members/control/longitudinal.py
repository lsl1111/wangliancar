"""Bounded PI speed demand and mutually exclusive pedal mapping."""

import math


def pedal_request(speed, reference, integral, dt, settings, calibration):
    error = reference - speed
    if abs(error) <= settings.control_deadband_mps:
        error = 0.0
    proposal = max(-settings.integral_limit,
                   min(settings.integral_limit, integral + error * dt))
    demand = settings.pi_kp * error + settings.pi_ki * proposal
    if error < 0 and demand > 0:
        proposal, demand = 0.0, settings.pi_kp * error
    elif error > 0 and demand < 0:
        proposal, demand = 0.0, settings.pi_kp * error
    if not math.isfinite(demand):
        raise ValueError("nonfinite PI demand")
    throttle = min(calibration.max_throttle,
                   max(0.0, demand * calibration.throttle_per_mps))
    brake = min(calibration.max_brake,
                max(0.0, -demand * calibration.brake_per_mps))
    saturated = (throttle >= calibration.max_throttle and error > 0 or
                 brake >= calibration.max_brake and error < 0)
    if saturated:
        proposal = integral  # anti-windup: do not integrate farther into saturation
    return throttle, brake, proposal, error
