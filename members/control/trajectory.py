"""Spatial path sampling for the forward-only controller (metres, m/s)."""

import math

from core.geometry import normalize_angle


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class PreparedPath(object):
    def __init__(self, points, max_segment_m):
        self.points = []
        self.arc = []
        for point in points:
            x, y, speed = point.x, point.y, point.speed
            if not all(_finite(value) for value in (x, y, speed)) or speed < 0:
                raise ValueError("invalid path point")
            if self.points:
                previous = self.points[-1]
                distance = math.hypot(x - previous[0], y - previous[1])
                if distance <= 1e-6:
                    # A repeated position may carry a lower speed limit.
                    self.points[-1] = (previous[0], previous[1],
                                       min(previous[2], float(speed)))
                    continue
                if distance > max_segment_m:
                    raise ValueError("path segment gap exceeds bound")
                self.arc.append(self.arc[-1] + distance)
            else:
                self.arc.append(0.0)
            self.points.append((float(x), float(y), float(speed)))
        if len(self.points) < 2 or self.arc[-1] <= 1e-6:
            raise ValueError("path has no forward geometry")

    @property
    def length(self):
        return self.arc[-1]

    def project(self, x, y, heading, settings):
        candidates = []
        for index in range(len(self.points) - 1):
            ax, ay = self.points[index][:2]
            bx, by = self.points[index + 1][:2]
            dx, dy = bx - ax, by - ay
            segment = self.arc[index + 1] - self.arc[index]
            raw = ((x - ax) * dx + (y - ay) * dy) / (segment * segment)
            ratio = max(0.0, min(1.0, raw))
            px, py = ax + ratio * dx, ay + ratio * dy
            distance = math.hypot(x - px, y - py)
            candidates.append((distance, self.arc[index] + ratio * segment,
                               math.atan2(dy, dx), index, raw))
        candidates.sort(key=lambda item: item[0])
        best = candidates[0]
        if best[0] > settings.max_projection_error_m:
            raise ValueError("vehicle too far from planned path")
        if abs(normalize_angle(best[2] - heading)) > settings.max_heading_error_rad:
            raise ValueError("path direction opposes vehicle")
        if best[3] == len(self.points) - 2 and best[4] > 1.0:
            raise ValueError("vehicle beyond path end")
        for other in candidates[1:]:
            if (other[0] <= best[0] + 0.15 and
                    abs(other[1] - best[1]) > 5.0):
                raise ValueError("ambiguous crossing-path projection")
        return best[1], best[0], normalize_angle(best[2] - heading)

    def sample(self, s):
        s = max(0.0, min(self.length, s))
        for index in range(len(self.arc) - 1):
            if s <= self.arc[index + 1] or index == len(self.arc) - 2:
                segment = self.arc[index + 1] - self.arc[index]
                ratio = (s - self.arc[index]) / segment
                first, second = self.points[index], self.points[index + 1]
                return tuple(first[k] + ratio * (second[k] - first[k])
                             for k in range(3))
        return self.points[-1]

    def speed_reference(self, s, trajectory, settings):
        """Point speed with future slowdowns and known path end as upper bounds."""
        local = self.sample(s)[2]
        remaining = max(0.0, self.length - s - settings.path_end_margin_m)
        limit = min(local, settings.max_track_speed_mps,
                    math.sqrt(2.0 * settings.preview_decel_mps2 * remaining))
        if trajectory.stop_required:
            if not _finite(trajectory.stop_distance) or trajectory.stop_distance < 0:
                raise ValueError("stop distance missing")
            stop_remaining = max(0.0, trajectory.stop_distance - settings.path_end_margin_m)
            limit = min(limit, math.sqrt(2.0 * settings.preview_decel_mps2 * stop_remaining))
        else:
            limit = min(limit, trajectory.target_speed)
        for index, future_s in enumerate(self.arc):
            if future_s <= s:
                continue
            future_speed = self.points[index][2]
            distance = future_s - s
            limit = min(limit, math.sqrt(future_speed * future_speed +
                                         2.0 * settings.preview_decel_mps2 * distance))
        return max(0.0, limit), remaining
