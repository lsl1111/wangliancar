"""Spatial path sampling for the forward-only controller (metres, m/s)."""

import math
from bisect import bisect_left

from core.geometry import normalize_angle


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class PreparedPath(object):
    def __init__(self, points, max_segment_m, curve_window_m=3.0):
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
        self.curvatures = [self.curvature_at(s, curve_window_m)
                           for s in self.arc]

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
        index = max(0, min(len(self.arc) - 2, bisect_left(self.arc, s) - 1))
        segment = self.arc[index + 1] - self.arc[index]
        ratio = (s - self.arc[index]) / segment
        first, second = self.points[index], self.points[index + 1]
        return tuple(first[k] + ratio * (second[k] - first[k])
                     for k in range(3))

    def curvature_at(self, s, window_m=3.0):
        """Signed path curvature from three equally spaced arc samples."""
        left_s = max(0.0, s - window_m)
        right_s = min(self.length, s + window_m)
        if right_s - left_s <= 1e-6:
            return 0.0
        middle_s = s
        if middle_s - left_s <= 1e-6 or right_s - middle_s <= 1e-6:
            middle_s = (left_s + right_s) / 2.0
        ax, ay = self.sample(left_s)[:2]
        bx, by = self.sample(middle_s)[:2]
        cx, cy = self.sample(right_s)[:2]
        ab = math.hypot(bx - ax, by - ay)
        bc = math.hypot(cx - bx, cy - by)
        ac = math.hypot(cx - ax, cy - ay)
        if min(ab, bc, ac) <= 1e-6:
            return 0.0
        cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        return 2.0 * cross / (ab * bc * ac)

    def curve_speed_limit(self, s, settings):
        """Slow before geometric turns using the same braking envelope as points."""
        limit = settings.max_track_speed_mps
        for turn_s, curvature in zip(self.arc, self.curvatures):
            if turn_s < s - settings.turn_exit_margin_m or abs(curvature) <= 1e-6:
                continue
            # A crawl-speed floor must not override a lateral-acceleration cap.
            turn_speed = math.sqrt(settings.max_lateral_accel_mps2 / abs(curvature))
            distance = max(0.0, turn_s - s)
            limit = min(limit, math.sqrt(turn_speed * turn_speed +
                                         2.0 * settings.preview_decel_mps2 * distance))
        return limit

    def speed_reference(self, s, trajectory, settings):
        """Point speed with future slowdowns and known path end as upper bounds."""
        local = self.sample(s)[2]
        # A rolling profile starts at measured speed, not at cruise speed.
        # Preview the rising first segment throughout acceleration, otherwise
        # PI sees zero error on every replan and never reaches cruise speed.
        # An interior zero-speed constraint is never bypassed this way.
        if s <= 1e-6 and trajectory.target_speed > 0:
            preview_s = min(self.length, s + settings.launch_preview_m)
            preview_speed = self.sample(preview_s)[2]
            launch_limit = math.sqrt(local * local +
                                     2.0 * settings.launch_accel_mps2 *
                                     max(0.0, preview_s - s))
            local = max(local, min(preview_speed, launch_limit))
        remaining = max(0.0, self.length - s - settings.path_end_margin_m)
        limit = min(local, settings.max_track_speed_mps,
                    math.sqrt(2.0 * settings.preview_decel_mps2 * remaining))
        if trajectory.stop_required:
            if not _finite(trajectory.stop_distance) or trajectory.stop_distance < 0:
                raise ValueError("stop distance missing")
            stop_remaining = max(0.0, trajectory.stop_distance - settings.path_end_margin_m)
            limit = min(limit, math.sqrt(2.0 * settings.preview_decel_mps2 * stop_remaining))
        if not trajectory.stop_required or trajectory.target_speed > 0:
            limit = min(limit, trajectory.target_speed)
        for index, future_s in enumerate(self.arc):
            if future_s <= s:
                continue
            future_speed = self.points[index][2]
            distance = future_s - s
            limit = min(limit, math.sqrt(future_speed * future_speed +
                                         2.0 * settings.preview_decel_mps2 * distance))
        return max(0.0, limit), remaining

    def acceleration_reference(self, s, trajectory, settings, reference):
        """Spatial v^2 slope, bounded by the profile's acceleration limits.

        No SDK clock or trajectory time derivative is assumed. A measured-speed
        start point uses its first segment; a future constraint is previewed
        with a corresponding reduction in GPS-relative stopping distance.
        """
        distance = min(settings.acceleration_preview_m, self.length - s)
        if distance <= 1e-6 or reference <= settings.hold_speed_mps:
            return 0.0
        if s <= 1e-6:
            initial = min(self.points[0][2], reference)
            future = min(self.sample(s + distance)[2], reference)
        else:
            initial = reference
            # The stop distance is relative to the current ego position, so
            # subtract the virtual preview travel without mutating input.
            future, _ = self.speed_reference(s + distance, trajectory, settings)
            if trajectory.stop_required:
                stop = max(0.0, trajectory.stop_distance - distance -
                           settings.path_end_margin_m)
                future = min(future, math.sqrt(2.0 * settings.preview_decel_mps2 * stop))
        future = min(future, self.curve_speed_limit(s + distance, settings))
        acceleration = (future * future - initial * initial) / (2.0 * distance)
        return max(-settings.preview_decel_mps2,
                   min(settings.launch_accel_mps2, acceleration))
