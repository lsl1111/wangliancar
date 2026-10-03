"""Bounded PI/profile-feedforward demand and mutually exclusive pedal mapping."""

import math


def pedal_request(speed, reference, integral, dt, settings, calibration,
                  acceleration=0.0):
    error = reference - speed
    # A fixed 0.08 m/s deadband would suppress every positive demand at a
    # smaller crawl speed, including parking corrections after a stop.
    deadband = min(settings.control_deadband_mps,
                   max(0.0, reference) * settings.crawl_deadband_ratio)
    if abs(error) <= deadband:
        error = 0.0
    proposal = max(-settings.integral_limit,
                   min(settings.integral_limit, integral + error * dt))
    feedforward = settings.acceleration_feedforward_s * acceleration
    # Preview may request propulsion only while below the speed reference.
    if error <= 0:
        feedforward = min(0.0, feedforward)
    # A falling profile can require braking before the current speed reference
    # becomes lower than GPS speed. Acceleration-era load integral must not
    # reverse that demand into propulsion. Preserve it for steady/rising
    # profiles and when the vehicle is substantially below a falling reference.
    reset_for_braking = (feedforward < -1e-6
                         and settings.pi_kp * error + feedforward <= 0.0
                         and proposal > 0.0)
    if reset_for_braking:
        proposal = 0.0
    demand = settings.pi_kp * error + settings.pi_ki * proposal + feedforward
    if error < 0 and demand > 0:
        proposal, demand = 0.0, settings.pi_kp * error + feedforward
    elif error > 0 and demand < 0:
        # Keep anticipatory braking for a falling profile even when measured
        # speed is slightly below today's reference.
        if feedforward >= 0:
            proposal, demand = 0.0, settings.pi_kp * error + feedforward
    if not math.isfinite(demand):
        raise ValueError("nonfinite PI demand")
    throttle = min(calibration.max_throttle,
                   max(0.0, demand * calibration.throttle_per_mps))
    brake = min(calibration.max_brake,
                max(0.0, -demand * calibration.brake_per_mps))
    saturated = (throttle >= calibration.max_throttle and error > 0 or
                 brake >= calibration.max_brake and error < 0)
    if saturated and not reset_for_braking:
        proposal = integral  # anti-windup: do not integrate farther into saturation
    return throttle, brake, proposal, error


def applied_integral(previous, proposed, error, requested_throttle,
                     requested_brake, throttle, brake, calibration):
    """Conditional integration after slew limits and pedal interlocking."""
    requested = (requested_throttle / calibration.throttle_per_mps -
                 requested_brake / calibration.brake_per_mps)
    applied = (throttle / calibration.throttle_per_mps -
               brake / calibration.brake_per_mps)
    if (requested - applied) * error > 1e-9:
        # Preserve an unwinding/reset proposal, only reject further windup.
        if (proposed - previous) * error > 0:
            return previous
    return proposed
