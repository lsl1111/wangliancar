"""Forward Pure Pursuit + PI/profile-feedforward control with explicit arming.

The engine computes spatial diagnostics before calibration, but never marks a
pedal/steering command valid until vehicle-specific mapping is supplied.
"""

import math
import time

from core.interfaces import ControlOut, Trajectory
from core.validation import current, validate_output
from members.control.lateral import geometry_request, steering_request
from members.control.longitudinal import pedal_request, applied_integral
from members.control.parameters import ControllerSettings
from members.control.trajectory import PreparedPath


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _approach(previous, desired, amount):
    return max(previous - amount, min(previous + amount, desired))


def _stationary_stop(trajectory):
    if not trajectory.stop_required or trajectory.target_speed != 0:
        return False
    origin = trajectory.points[0]
    return all(point.speed == 0 and
               math.hypot(point.x - origin.x, point.y - origin.y) <= 1e-6
               for point in trajectory.points)


class ControlEngine(object):
    def __init__(self, calibration=None, settings=None, clock=None):
        self.calibration = calibration
        self.settings = (settings or ControllerSettings()).validate()
        self.clock = clock or time.monotonic
        self.reset()

    def reset(self):
        self.case = None
        self.last_frame = None
        self.last_time = None
        self.last_output = None
        self.integral = 0.0
        self.state = "INIT"
        self.release_frames = 0

    def _invalid(self, output, reason):
        self.integral = 0.0
        self.release_frames = 0
        self.last_output = None
        self.state = "FAULT_STOP"
        output.throttle = output.brake = output.steering = 0.0
        output.source = "control:FAULT_STOP"
        output.errors.append(reason)
        output.diagnostics["state"] = self.state
        return output

    def compute(self, perception, trajectory):
        output = ControlOut()
        output.source = "control:INIT"
        output.diagnostics = {}
        now = self.clock()
        try:
            output.bind(trajectory)
            case = (perception.case_id, perception.case_name)
            if self.case != case:
                self.reset()
                self.case = case
            if (type(perception.frame_id) is not int or
                    (self.last_frame is not None and
                     perception.frame_id < self.last_frame)):
                return self._invalid(output, "non-increasing perception frame")
            dt = 0.0 if self.last_time is None else now - self.last_time
            if self.last_time is not None and (dt < 0 or dt > self.settings.max_dt_s or
                    (dt == 0 and perception.frame_id != self.last_frame)):
                self.last_frame, self.last_time = perception.frame_id, now
                return self._invalid(output, "control loop dt outside bound")
            if not current(perception) or not perception.ego.valid:
                raise ValueError("ego perception invalid or expired")
            gps_status = perception.source_status.get("gps", {})
            if not isinstance(gps_status, dict) or gps_status.get("usable") is not True:
                raise ValueError("GPS source unusable")
            ego = perception.ego
            if (type(ego.frame_id) is not int or ego.frame_id != perception.frame_id or
                    type(ego.age_ms) is not int or ego.age_ms < 0 or
                    not all(_finite(getattr(ego, name)) for name in
                            ("x", "y", "heading", "speed")) or ego.speed < 0):
                raise ValueError("ego pose or speed invalid")
            if type(ego.gear) is not int:
                raise ValueError("invalid GPS gear position")
            validate_output(trajectory, Trajectory, perception)
            if perception.frame_id == self.last_frame:
                # Runtime must not resend a normal command on a repeated GPS
                # frame. Keep PI/slew/HOLD memory until a genuinely new frame;
                # expiry and stalling remain checked above and by supervisor.
                output.source = "control:DUPLICATE"
                output.errors.append("duplicate perception frame; no new command")
                output.diagnostics["state"] = self.state
                return output
            self.last_frame, self.last_time = perception.frame_id, now
            # A raw gearbox position cannot be used as an N/D/R/P command.
            # Brake-only output uses Neutral; forward tracking requests Drive.
            output.gear = 0
            output.diagnostics["dt_s"] = dt
            if trajectory.emergency_stop:
                self.integral = 0.0
                self.release_frames = 0
                self.state = "EMERGENCY"
                output.diagnostics["state"] = self.state
                if self.calibration is None:
                    return self._invalid(output, "vehicle calibration required")
                output.brake = self.calibration.emergency_brake
                output.source = "control:EMERGENCY"
                output.valid = True
                output.clamp()
                self.last_output = output
                return output

            if (not all(_finite(getattr(ego, name)) for name in ("vx", "vy"))
                    or ego.vx * math.cos(ego.heading) + ego.vy * math.sin(ego.heading) < -1e-6):
                raise ValueError("reverse or invalid velocity is unsupported for forward tracking")
            # Keep observed gearbox positions intact; the forward trajectory
            # supplies the intent, and signed velocity rejects reverse motion.
            output.gear = 1
            output.diagnostics["observed_gear"] = ego.gear
            output.diagnostics["requested_gear"] = output.gear
            if _stationary_stop(trajectory):
                origin = trajectory.points[0]
                if math.hypot(ego.x - origin.x, ego.y - origin.y) > self.settings.max_projection_error_m:
                    raise ValueError("stationary stop is too far from vehicle")
                if self.calibration is None:
                    return self._invalid(output, "vehicle calibration required")
                self.integral = 0.0
                self.release_frames = 0
                self.state = ("HOLD" if ego.speed <= self.settings.hold_speed_mps
                              else "STOPPING")
                output.brake = (self.calibration.hold_brake if self.state == "HOLD"
                                else self.calibration.max_brake)
                output.diagnostics.update({
                    "reference_speed_mps": 0.0,
                    "speed_error_mps": -ego.speed,
                    "state": self.state})
                output.source = "control:" + self.state
                output.valid = True
                output.clamp()
                self.last_output = output
                return output

            path = PreparedPath(trajectory.points, self.settings.max_segment_m,
                                self.settings.curve_window_m)
            progress, cross_track, heading_error = path.project(
                ego.x, ego.y, ego.heading, self.settings)
            reference, remaining = path.speed_reference(progress, trajectory,
                                                        self.settings)
            curve_limit = path.curve_speed_limit(progress, self.settings)
            reference = min(reference, curve_limit)
            if trajectory.stop_required and trajectory.target_speed == 0:
                reference = min(reference, ego.speed)
                self.integral = min(0.0, self.integral)
            output.diagnostics.update({
                "path_progress_m": progress, "path_remaining_m": remaining,
                "cross_track_abs_m": cross_track, "heading_error_rad": heading_error,
                "curve_speed_limit_mps": curve_limit,
                "reference_speed_mps": reference,
                "speed_error_mps": reference - ego.speed})
            hold_request = (reference <= self.settings.hold_speed_mps and
                            ego.speed <= self.settings.hold_speed_mps)
            # At a finite path end there may be no valid forward carrot left,
            # but braking must remain executable while the vehicle is moving.
            if reference <= self.settings.hold_speed_mps and not hold_request:
                if self.calibration is None:
                    return self._invalid(output, "vehicle calibration required")
                self.integral, self.release_frames = 0.0, 0
                self.state = "STOPPING"
                output.brake = self.calibration.max_brake
                output.steering = (self.last_output.steering if self.last_output else 0.0)
                output.diagnostics["state"] = self.state
                output.source, output.valid = "control:STOPPING", True
                output.clamp()
                self.last_output = output
                return output
            if hold_request or self.state == "HOLD":
                if (not hold_request and trajectory.target_speed > 0 and
                        reference >= self.settings.hold_release_speed_mps):
                    self.release_frames += 1
                else:
                    self.release_frames = 0
                if hold_request or self.release_frames < self.settings.hold_release_frames:
                    self.integral = 0.0
                    self.state = "HOLD"
                    output.diagnostics["state"] = self.state
                    if self.calibration is None:
                        return self._invalid(output, "vehicle calibration required")
                    output.brake = self.calibration.hold_brake
                    output.source = "control:HOLD"
                    output.valid = True
                    output.clamp()
                    self.last_output = output
                    return output
                self.release_frames = 0

            curvature, geometry = geometry_request(path, progress, ego,
                                                    self.settings)
            output.diagnostics.update(geometry)
            if self.calibration is None:
                return self._invalid(output, "vehicle calibration required")
            steering, front_angle = steering_request(curvature, self.calibration)
            output.diagnostics["front_angle_rad"] = front_angle
            acceleration = path.acceleration_reference(
                progress, trajectory, self.settings, reference)
            if trajectory.stop_required and trajectory.target_speed == 0:
                acceleration = min(0.0, acceleration)
            throttle, brake, next_integral, speed_error = pedal_request(
                ego.speed, reference, self.integral, dt,
                self.settings, self.calibration, acceleration)
            if trajectory.stop_required and trajectory.target_speed == 0:
                throttle = 0.0
            requested_throttle, requested_brake = throttle, brake
            if self.last_output is not None and dt > 0:
                steering = _approach(self.last_output.steering, steering,
                                     self.settings.steering_slew_per_s * dt)
                if throttle > self.last_output.throttle:
                    throttle = min(throttle, self.last_output.throttle +
                                   self.settings.throttle_slew_per_s * dt)
                if brake < self.last_output.brake:
                    brake = max(brake, self.last_output.brake -
                                self.settings.brake_slew_per_s * dt)
            if brake > 0:
                throttle = 0.0
            if throttle > 0:
                brake = 0.0
            next_integral = applied_integral(
                self.integral, next_integral, speed_error,
                requested_throttle, requested_brake, throttle, brake,
                self.calibration)
            output.throttle, output.brake, output.steering = throttle, brake, steering
            output.diagnostics["speed_error_mps"] = speed_error
            output.diagnostics["integral_m"] = next_integral
            output.diagnostics["reference_acceleration_mps2"] = acceleration
            self.integral = next_integral
            self.state = ("STOPPING" if trajectory.stop_required and
                          trajectory.target_speed == 0 else "TRACK")
            output.diagnostics["state"] = self.state
            output.source = "control:" + self.state
            output.valid = True
            output.clamp()
            self.last_output = output
            return output
        except (AttributeError, TypeError, ValueError, OverflowError, IndexError) as exc:
            return self._invalid(output, str(exc))
