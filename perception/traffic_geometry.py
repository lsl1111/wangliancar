"""Bound traffic stop locations to verified route and full map line geometry."""
import math

from core.geometry import project_polyline, projection_within_polyline
from core.route_segments import verified_spans


def _points(raw):
    if not isinstance(raw, (list, tuple)) or not 2 <= len(raw) <= 20000:
        return None
    result = []
    for point in raw:
        if (not isinstance(point, (list, tuple)) or len(point) < 2
                or any(type(v) not in (int, float) or not math.isfinite(v)
                       for v in point[:3])):
            return None
        result.append(tuple(float(v) for v in point[:2]) +
                      (float(point[2]) if len(point) > 2 else 0.0,))
    return result


def signal_reference(lane):
    """Only the unchanged current prefix and verified selected successors."""
    current = _points(lane.center_line)
    identities = [lane.lane_id]
    if current is None or lane.forward_reference_valid is not True:
        return current, identities
    forward, ids = _points(lane.forward_reference), lane.forward_lane_ids
    if (forward is None or forward[:len(current)] != current
            or not isinstance(ids, (list, tuple)) or not ids or ids[0] != lane.lane_id
            or not verified_spans(len(current), len(forward), ids, lane.forward_lane_spans)):
        return current, identities
    return forward, list(ids)


def stop_line_position(points, light, width):
    """Return (verified s, unresolved lower bound), never a snapped endpoint.

    A point-only legacy record beyond the last segment can establish that an
    unresolved line is beyond coverage, but cannot establish a stop location.
    This lets an earlier verified line remain usable without accepting a
    farther green when the nearest line is unresolved.
    """
    if 'stop_line_boundary' not in light:
        x, y = float(light['stop_line_x']), float(light['stop_line_y'])
        if not all(math.isfinite(v) for v in (x, y)):
            raise ValueError('invalid stop-line point')
        projection = project_polyline(points, x, y)
        if projection is None:
            return None, None
        if (projection['distance'] <= max(2.0, width)
                and projection_within_polyline(projection, len(points))):
            return projection['s'], None
        if projection['index'] == len(points)-2 and projection['raw_ratio'] > 1:
            # Only a lower bound, not permission to extrapolate the road.
            a, b = points[-2:]
            length = math.hypot(b[0]-a[0], b[1]-a[1])
            lateral = abs((x-b[0])*(b[1]-a[1]) - (y-b[1])*(b[0]-a[0])) / length
            if lateral <= max(2.0, width):
                return None, projection['s']
        return None, None
    boundary = _points(light['stop_line_boundary'])
    if boundary is None:
        raise ValueError('invalid stop-line boundary')
    crossings, along = [], 0.0
    for a, b in zip(points, points[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(dx, dy)
        if length <= 1e-6:
            continue
        for c, d in zip(boundary, boundary[1:]):
            ux, uy = d[0]-c[0], d[1]-c[1]
            determinant = dx*uy-dy*ux
            if abs(determinant) <= 1e-9:
                continue
            cx, cy = c[0]-a[0], c[1]-a[1]
            route_ratio = (cx*uy-cy*ux)/determinant
            line_ratio = (cx*dy-cy*dx)/determinant
            if -1e-6 <= route_ratio <= 1+1e-6 and -1e-6 <= line_ratio <= 1+1e-6:
                route_z = a[2]+route_ratio*(b[2]-a[2])
                line_z = c[2]+line_ratio*(d[2]-c[2])
                if abs(route_z-line_z) <= 2.0:
                    progress = along+max(0.0, min(1.0, route_ratio))*length
                    if not any(abs(progress-old) <= .01 for old in crossings):
                        crossings.append(progress)
        along += length
    if len(crossings) == 1:
        return crossings[0], None
    # Missing intersection or multiple road positions is uncertainty, not absence.
    return None, None
