"""Deterministic synthetic closed-loop benchmark, not a SimOne vehicle model.

Run with the project's Python 3.6: python -m tests.control_benchmark.
The bicycle model includes steering/pedal lag and drag. Values below are
test-only; results do not certify physical stopping distance or calibration.
"""

import json
import math
import time

from core.interfaces import Perception, Trajectory, TrajectoryPoint
from members.control.controller import ControlEngine
from members.control.parameters import ControllerSettings, VehicleCalibration


class Plant(object):
    def __init__(self, x=0.0, y=0.0, heading=0.0, speed=0.0, lag=0.2):
        self.x, self.y, self.heading, self.speed = x, y, heading, speed
        self.steering = self.throttle = self.brake = 0.0
        self.yaw_rate = 0.0
        self.lag = lag

    def step(self, output, dt, vehicle):
        blend = 1.0 - math.exp(-dt / self.lag)
        for field in ("steering", "throttle", "brake"):
            value = getattr(self, field)
            setattr(self, field, value + blend * (getattr(output, field) - value))
        acceleration = 3.0 * self.throttle - 6.0 * self.brake - 0.08 * self.speed
        next_speed = max(0.0, self.speed + acceleration * dt)
        speed = (self.speed + next_speed) * 0.5
        angle = self.steering * vehicle.front_steer_max_rad / vehicle.steering_sign
        yaw = speed / vehicle.wheelbase_m * math.tan(angle)
        self.yaw_rate = yaw
        mid_heading = self.heading + yaw * dt * 0.5
        self.x += speed * math.cos(mid_heading) * dt
        self.y += speed * math.sin(mid_heading) * dt
        self.heading += yaw * dt
        self.speed = next_speed


def vehicle():
    return VehicleCalibration(2.7, 0.5, 1, 0.3, 0.4, 0.2, 0.9, 0.4, 0.6)


def frame_input(plant, points, frame, target_speed, stop_distance=-1.0):
    p = Perception()
    p.valid, p.frame_id, p.timestamp = True, frame, frame * 50
    p.valid_until = time.monotonic() + 20.0
    p.case_id = p.case_name = "synthetic-benchmark"
    p.ego.valid, p.ego.frame_id, p.ego.age_ms = True, frame, 0
    p.ego.gear = 1
    p.ego.x, p.ego.y = plant.x, plant.y
    p.ego.heading, p.ego.speed = plant.heading, plant.speed
    p.ego.yaw_rate = plant.yaw_rate
    p.source_status = {"gps": {"usable": True}}
    t = Trajectory().bind(p)
    t.valid, t.target_speed = True, target_speed
    t.stop_required, t.stop_distance = stop_distance >= 0, stop_distance
    t.points = [TrajectoryPoint(x, y, speed, 0.0, i * 0.1)
                for i, (x, y, speed) in enumerate(points)]
    return p, t


def simulate(name, points, initial, target, duration=12.0, lag=0.2,
             duplicate=False, stop_x=None, settings=None):
    plant = Plant(*initial, lag=lag)
    calibration = vehicle()
    now = [0.0]
    engine = ControlEngine(calibration, settings, clock=lambda: now[0])
    cross_track, speed_errors, failures = [], [], []
    dt = 0.05
    for frame in range(1, int(duration / dt) + 1):
        stop_distance = max(0.0, stop_x - plant.x) if stop_x is not None else -1.0
        p, t = frame_input(plant, points, frame, target, stop_distance)
        output = engine.compute(p, t)
        if not output.valid:
            failures.append(output.errors)
            break
        cross_track.append(output.diagnostics.get("cross_track_abs_m", 0.0))
        speed_errors.append(abs(plant.speed - target) if target > 0 else plant.speed)
        plant.step(output, dt, calibration)
        if duplicate:
            now[0] += dt * 0.5
            engine.compute(p, t)  # Runtime skips normal sending on repeated GPS.
            now[0] += dt * 0.5
        else:
            now[0] += dt
    return {"scenario": name, "max_cross_track_m": max(cross_track or [0.0]),
            "tail_mean_speed_error_mps": sum(speed_errors[-40:]) / max(1, len(speed_errors[-40:])),
            "final_x_m": plant.x, "final_y_m": plant.y,
            "final_speed_mps": plant.speed, "failures": failures}


def benchmark():
    straight = [(float(i), 0.0, 3.0) for i in range(151)]
    results = []
    for lag in (0.1, 0.3):
        results.append(simulate("straight_lag_" + str(lag), straight,
                                (0.0, 0.8, 0.08, 0.0), 3.0, lag=lag))
    results.append(simulate("duplicate_frames", straight, (0.0, 0.0, 0.0, 0.0),
                            3.0, duration=20.0, duplicate=True))
    for radius in (10.0, 25.0):
        arc = [(radius * math.cos(i * 0.025), radius * math.sin(i * 0.025), 3.0)
               for i in range(121)]
        results.append(simulate("curve_radius_" + str(radius), arc,
                                (radius, 0.0, math.pi / 2.0, 3.0), 3.0,
                                duration=7.0, lag=0.3))
    shift = []
    for i in range(101):
        u = min(1.0, i / 30.0)
        shift.append((float(i), 3.5 * (6*u**5 - 15*u**4 + 10*u**3), 3.0))
    results.append(simulate("lateral_shift", shift, (0.0, 0.0, 0.0, 3.0),
                            3.0, lag=0.3))
    stop = [(i * 0.25, 0.0, min(4.0, math.sqrt(max(0.0, 2.0 * (20.0-i*0.25)))))
            for i in range(81)]
    results.append(simulate("stop_profile", stop, (0.0, 0.0, 0.0, 4.0),
                            0.0, duration=15.0, stop_x=20.0, lag=0.3))
    return results


if __name__ == "__main__":
    print(json.dumps(benchmark(), indent=2, sort_keys=True))
