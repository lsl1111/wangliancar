"""Pure Pursuit in the vehicle frame (x forward, y left)."""

import math

from core.geometry import world_to_ego


def geometry_request(path, progress, ego, settings):
    path_curvature = path.curvature_at(progress, settings.curve_window_m)
    base_distance = min(settings.lookahead_max_m,
                        settings.lookahead_base_m + settings.lookahead_time_s * ego.speed)
    distance = max(settings.lookahead_min_m,
                   base_distance / (1.0 + settings.curve_lookahead_gain_m *
                                    abs(path_curvature)))
    lookahead_s = min(path.length, progress + distance)
    target_x, target_y, unused_speed = path.sample(lookahead_s)
    # Predict rear-axle motion during a bounded actuator preview. This uses
    # measured GPS yaw rate, not a fabricated steering ratio/dynamic model.
    preview_x, preview_y, preview_heading = ego.x, ego.y, ego.heading
    yaw_rate = getattr(ego, "yaw_rate", None)
    preview_time = 0.0
    if (type(yaw_rate) in (int, float) and math.isfinite(yaw_rate)
            and ego.speed > 1e-6 and settings.steering_preview_s > 0):
        preview_time = min(settings.steering_preview_s,
                           0.35 * (lookahead_s - progress) / ego.speed)
        angle = yaw_rate * preview_time
        if abs(angle) < 1e-6:
            dx, dy = ego.speed * preview_time, 0.0
        else:
            radius = ego.speed / yaw_rate
            dx, dy = radius * math.sin(angle), radius * (1.0 - math.cos(angle))
        preview_x += dx * math.cos(ego.heading) - dy * math.sin(ego.heading)
        preview_y += dx * math.sin(ego.heading) + dy * math.cos(ego.heading)
        preview_heading += angle
    forward, left = world_to_ego(preview_x, preview_y, preview_heading, target_x, target_y)
    chord2 = forward * forward + left * left
    if forward <= 0.1 or chord2 <= 0.04:
        raise ValueError("no forward lookahead point")
    curvature = 2.0 * left / chord2
    if not math.isfinite(curvature):
        raise ValueError("nonfinite curvature")
    return curvature, {
        "lookahead_x": target_x, "lookahead_y": target_y,
        "lookahead_s": lookahead_s, "lookahead_m": distance,
        "path_curvature_m_inv": path_curvature,
        "curvature_m_inv": curvature,
        "steering_preview_s": preview_time,
        "preview_ego_x": preview_x, "preview_ego_y": preview_y,
        "lookahead_forward_m": forward, "lookahead_left_m": left}


def steering_request(curvature, calibration):
    front_angle = math.atan(calibration.wheelbase_m * curvature)
    steering = calibration.steering_sign * front_angle / calibration.front_steer_max_rad
    if not all(math.isfinite(v) for v in (curvature, front_angle, steering)):
        raise ValueError("nonfinite steering request")
    return max(-1.0, min(1.0, steering)), front_angle
