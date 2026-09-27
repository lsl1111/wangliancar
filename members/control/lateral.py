"""Pure Pursuit in the vehicle frame (x forward, y left)."""

import math

from core.geometry import world_to_ego


def geometry_request(path, progress, ego, settings):
    distance = min(settings.lookahead_max_m,
                   settings.lookahead_base_m + settings.lookahead_time_s * ego.speed)
    lookahead_s = min(path.length, progress + distance)
    target_x, target_y, unused_speed = path.sample(lookahead_s)
    forward, left = world_to_ego(ego.x, ego.y, ego.heading, target_x, target_y)
    chord2 = forward * forward + left * left
    if forward <= 0.1 or chord2 <= 0.04:
        raise ValueError("no forward lookahead point")
    curvature = 2.0 * left / chord2
    if not math.isfinite(curvature):
        raise ValueError("nonfinite curvature")
    return curvature, {
        "lookahead_x": target_x, "lookahead_y": target_y,
        "lookahead_s": lookahead_s, "lookahead_m": distance,
        "curvature_m_inv": curvature,
        "lookahead_forward_m": forward, "lookahead_left_m": left}


def steering_request(curvature, calibration):
    front_angle = math.atan(calibration.wheelbase_m * curvature)
    steering = calibration.steering_sign * front_angle / calibration.front_steer_max_rad
    if not all(math.isfinite(v) for v in (curvature, front_angle, steering)):
        raise ValueError("nonfinite steering request")
    return max(-1.0, min(1.0, steering)), front_angle
