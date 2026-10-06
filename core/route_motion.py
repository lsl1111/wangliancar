"""Motion residuals relative to verified sampled road geometry.

For a road-aligned vehicle, a chord tangent is uncertain over the vehicle's
longitudinal footprint. Separate that sampled-road turn from actual sideways
slip. This is a route-following motion hypothesis, not lane-change prediction.
Callers must establish map membership, route association and measured width.
"""
import math

from core.geometry import normalize_angle, projection_within_polyline
from core.validation import number


def lateral_drift_bound(target, projection, velocity, width, window, margin, tolerance):
    """Shared support test and full drift for constant lateral target motion.

    A tolerance permits noise but never removes its footprint expansion.
    Beyond it, require bounded map membership, heading and remaining corridor.
    This tests the same local hypothesis for behavior and trajectory planning.
    """
    if not all(number(v) for v in (velocity, window, margin, tolerance)) or window <= 0 or min(margin, tolerance) < 0:
        raise ValueError('invalid target motion bound')
    if abs(velocity) <= 1e-9:
        return 0.0
    drift = abs(velocity)*window
    if not number(drift):
        raise ValueError('nonfinite obstacle lateral drift')
    if abs(velocity) <= tolerance:
        return drift
    if (not number(width) or width <= 0 or not number(target.heading)
            or not projection.get('_coverage_checked', False)):
        raise ValueError('crossing or oncoming obstacle motion unsupported')
    angle = target.heading-projection['heading']
    extent = .5*(target.length*abs(math.sin(angle))+target.width*abs(math.cos(angle)))
    clearance = width*.5-extent-margin
    if clearance <= 0 or projection['distance']+drift > clearance:
        raise ValueError('crossing or oncoming obstacle motion unsupported')
    return drift


def lateral_residual(points, projection, target, width):
    values = (target.heading, target.vx, target.vy, target.length, target.width, width)
    if (not all(number(v) for v in values) or min(target.length, target.width, width) <= 0
            or not projection_within_polyline(projection, len(points))):
        return None
    heading = projection['heading']
    angle = normalize_angle(target.heading - heading)
    lateral_extent = .5 * (target.length * abs(math.sin(angle))
                           + target.width * abs(math.cos(angle)))
    if projection['distance'] + lateral_extent >= width * .5:
        return None
    lengths = [math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:])]
    index = projection['index']
    if not 0 <= index < len(lengths):
        return None
    centre = sum(lengths[:index]) + min(1, max(0, projection['raw_ratio'])) * lengths[index]
    half_length = target.length * .5
    angles, along = [], 0.0
    for i, length in enumerate(lengths):
        if length > 1e-6 and along <= centre + half_length and along + length >= centre - half_length:
            a, b = points[i:i+2]
            angles.append(normalize_angle(math.atan2(b[1]-a[1], b[0]-a[0]) - heading))
        along += length
    if not angles or max(angles)-min(angles) >= math.pi / 2:
        return None
    low, high = min(angles), max(angles)
    # Body orientation must agree with the supported local road tangents.
    # A crossing car cannot claim a turn merely because a bend exists ahead.
    if not low-1e-6 <= angle <= high+1e-6:
        return None
    speed = math.hypot(target.vx, target.vy)
    if speed <= 1e-9:
        return 0.0
    velocity_heading = math.atan2(target.vy, target.vx)
    velocity_angle = normalize_angle(velocity_heading - heading)
    if abs(velocity_angle) >= math.pi / 2:
        return None
    outside = velocity_angle - max(low, min(high, velocity_angle))
    road_residual = speed * math.sin(outside)
    body_slip = speed * math.sin(normalize_angle(velocity_heading-target.heading))
    return road_residual if abs(road_residual) >= abs(body_slip) else body_slip
