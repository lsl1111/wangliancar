"""Small dependency-free geometry helpers."""

import math


# Shared default support boundary for forward trajectory tracking. This is
# independent of tolerances used to verify two HDMap samples at their join.
DEFAULT_FORWARD_HEADING_ERROR_RAD = math.pi / 4


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(angle):
    if not math.isfinite(angle):
        raise ValueError("nonfinite angle")
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def speed_2d(vx, vy):
    return math.hypot(vx, vy)


def opposes_direction(vx, vy, heading, speed, direction=1, standstill_mps=0.05):
    """Do not classify bounded standstill rollback as an opposite manoeuvre.

    The default matches the existing control gear-standstill criterion. A
    nonzero reported speed or vector speed above that criterion still requires
    directional agreement; this does not authorize driving against real motion.
    """
    signed = direction * (vx * math.cos(heading) + vy * math.sin(heading))
    return signed < -1e-6 and max(speed, math.hypot(vx, vy)) > standstill_mps


def project_polyline(points, x, y):
    """Nearest bounded segment projection; no extrapolation past map coverage."""
    best, along = None, 0.0
    for index in range(len(points) - 1):
        ax, ay = points[index][:2]
        bx, by = points[index + 1][:2]
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        if length < 1e-6:
            continue
        raw_ratio = ((x - ax) * dx + (y - ay) * dy) / (length * length)
        ratio = clamp(raw_ratio, 0.0, 1.0)
        px, py = ax + ratio * dx, ay + ratio * dy
        distance = math.hypot(x - px, y - py)
        if best is None or distance < best["distance"]:
            best = {"s": along + ratio * length, "distance": distance,
                    "index": index, "ratio": ratio, "raw_ratio": raw_ratio,
                    "point": (px, py), "heading": math.atan2(dy, dx)}
        along += length
    return best


def projection_within_polyline(projection, point_count, epsilon=1e-6):
    """Distinguish a bounded internal vertex from true coverage extrapolation.

    A clamped projection onto an internal vertex may have a raw ratio outside
    its single segment. Only the first/last segment can establish that the
    whole polyline's start/end was exceeded. Width, height, direction and
    ambiguity checks remain the caller's responsibility.
    """
    if (not isinstance(projection, dict) or type(point_count) is not int
            or point_count < 2 or type(epsilon) not in (int, float)
            or not math.isfinite(epsilon) or epsilon < 0):
        return False
    index, raw = projection.get('index'), projection.get('raw_ratio')
    if (type(index) is not int or not 0 <= index < point_count - 1
            or type(raw) not in (int, float) or not math.isfinite(raw)):
        return False
    return not ((index == 0 and raw < -epsilon)
                or (index == point_count - 2 and raw > 1 + epsilon))


def polyline_prefix(points, distance):
    """A bounded ahead path, including the exact end of the requested range."""
    result = [points[0][:2]] if points else []
    for a, b in zip(points, points[1:]):
        length = math.hypot(b[0]-a[0], b[1]-a[1])
        if length <= 1e-6:
            continue
        if distance <= length:
            ratio = max(0.0, distance / length)
            result.append((a[0]+ratio*(b[0]-a[0]), a[1]+ratio*(b[1]-a[1])))
            break
        result.append(b[:2])
        distance -= length
    return result


def swept_path_distance(points, start, end):
    """Minimum distance from a constant-velocity segment to a polyline.

    Check segment interiors as well as endpoints so a crossing between
    observations is retained. Callers expand by vehicle/target footprints.
    """
    def point_segment(p, a, b):
        dx, dy = b[0]-a[0], b[1]-a[1]
        square = dx*dx + dy*dy
        ratio = clamp(((p[0]-a[0])*dx+(p[1]-a[1])*dy)/square, 0, 1) if square > 1e-12 else 0
        return math.hypot(p[0]-a[0]-ratio*dx, p[1]-a[1]-ratio*dy)
    nearest = float('inf')
    ux, uy = end[0]-start[0], end[1]-start[1]
    for a, b in zip(points, points[1:]):
        vx, vy = b[0]-a[0], b[1]-a[1]
        determinant = ux*vy-uy*vx
        if abs(determinant) > 1e-12:
            dx, dy = a[0]-start[0], a[1]-start[1]
            first, second = (dx*vy-dy*vx)/determinant, (dx*uy-dy*ux)/determinant
            if 0 <= first <= 1 and 0 <= second <= 1:
                return 0.0
        nearest = min(nearest, point_segment(start,a,b), point_segment(end,a,b),
                      point_segment(a,start,end), point_segment(b,start,end))
    return nearest


def world_to_ego(ego_x, ego_y, heading, target_x, target_y):
    dx = target_x - ego_x
    dy = target_y - ego_y
    c = math.cos(heading)
    s = math.sin(heading)
    longitudinal = c * dx + s * dy
    lateral = -s * dx + c * dy
    return longitudinal, lateral


def calculate_ttc(longitudinal_distance, ego_speed, target_vx, target_vy, heading):
    target_forward_speed = math.cos(heading) * target_vx + math.sin(heading) * target_vy
    closing_speed = ego_speed - target_forward_speed
    if longitudinal_distance <= 0.0 or closing_speed <= 0.05:
        return -1.0, closing_speed
    return longitudinal_distance / closing_speed, closing_speed


def nearest_path_error(points, x, y, heading):
    """Return signed lateral error and heading error for a polyline."""
    if not points or len(points) < 2:
        return 0.0, 0.0
    best = None
    for index in range(len(points) - 1):
        x1, y1 = points[index][0], points[index][1]
        x2, y2 = points[index + 1][0], points[index + 1][1]
        vx, vy = x2 - x1, y2 - y1
        length2 = vx * vx + vy * vy
        if length2 < 1e-8:
            continue
        ratio = clamp(((x - x1) * vx + (y - y1) * vy) / length2, 0.0, 1.0)
        px, py = x1 + ratio * vx, y1 + ratio * vy
        dx, dy = x - px, y - py
        distance2 = dx * dx + dy * dy
        if best is None or distance2 < best[0]:
            cross = vx * (y - py) - vy * (x - px)
            signed = math.sqrt(distance2) if cross >= 0.0 else -math.sqrt(distance2)
            path_heading = math.atan2(vy, vx)
            best = (distance2, signed, normalize_angle(path_heading - heading))
    if best is None:
        return 0.0, 0.0
    return best[1], best[2]
