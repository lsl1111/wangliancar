"""Closed-loop handoff through the real fixed decision/planning/control entries.

Only inputs, vehicle dimensions, clock and vehicle plant are synthetic.
No algorithm output is mocked or manufactured to bypass upstream checks.
This is an offline chain check, not Sensor API or SimOne acceptance.
"""

import math
import os
import unittest
from unittest.mock import patch

from core.interfaces import ControlOut, DecisionTarget, Trajectory
from core.validation import validate_output
from members import control_stub, decision_stub
from members.control.controller import ControlEngine
from members.decision.settings import DecisionSettings
from members.planning_stub import plan
from tests.control_benchmark import vehicle
from tests.test_control_maneuvers import BidirectionalPlant
from tests.test_decision import add_target, perception


class RealPipelineControlTests(unittest.TestCase):
    def setUp(self):
        self.now = [0.0]
        self.calibration = vehicle()
        self.settings = DecisionSettings(front_offset_m=3.5)
        self.environment = patch.dict(os.environ, {
            "NEVC_VEHICLE_FRONT_OFFSET_M": "3.5",
            "NEVC_VEHICLE_HALF_WIDTH_M": "0.9",
            "NEVC_PLANNING_MOTION_TOLERANCE_MPS": "0.05",
        })
        self.environment.start()
        self.addCleanup(self.environment.stop)
        self.engine_patch = patch.object(control_stub, "_engine",
            ControlEngine(self.calibration, clock=lambda: self.now[0]))
        self.engine_patch.start()
        self.addCleanup(self.engine_patch.stop)
        self.decision_patch = patch.object(decision_stub, "_ENGINE",
            decision_stub.DecisionEngine(self.settings))
        self.decision_patch.start()
        self.addCleanup(self.decision_patch.stop)

    def sample(self, plant, frame, curved=False):
        p = perception(speed=plant.speed, frame_id=frame, ttl=30)
        p.frame_id = p.ego.frame_id = frame
        p.timestamp = p.ego.timestamp = frame*50
        p.ego.age_ms, p.targets_age_ms = 0, 0
        p.case_id = p.case_name = "synthetic-real-pipeline"
        p.task_id, p.scene_id = "synthetic-real-pipeline", 25
        p.ego.x, p.ego.y, p.ego.heading = plant.x, plant.y, plant.heading
        p.ego.vx = plant.speed*math.cos(plant.heading)
        p.ego.vy = plant.speed*math.sin(plant.heading)
        p.ego.yaw_rate = plant.yaw_rate
        p.targets_frame, p.targets_timestamp = frame, p.timestamp
        if curved:
            p.lane.center_line = [(40*math.cos(i*0.01), 40*math.sin(i*0.01))
                                  for i in range(401)]
        for status in p.source_status.values():
            status.update({"frame_id": frame, "timestamp": p.timestamp,
                           "age_ms": 0, "usable": True})
        p.source_status["targets"]["sensor_read_ok"] = True
        return p

    def step(self, plant, p):
        decision = decision_stub.decide(p)
        self.assertTrue(decision.valid, decision.reason)
        validate_output(decision, DecisionTarget, p)
        trajectory = plan(p, decision)
        self.assertTrue(trajectory.valid, (p.frame_id, decision.reason, trajectory.reason))
        validate_output(trajectory, Trajectory, decision)
        output = control_stub.compute_control(p, trajectory)
        self.assertTrue(output.valid, (p.frame_id, decision.reason, trajectory.reason, output.errors))
        validate_output(output, ControlOut, trajectory)
        self.assertFalse(output.throttle > 0 and output.brake > 0)
        plant.step(output, 0.05, self.calibration)
        self.now[0] += 0.05
        return decision, trajectory, output

    def test_static_lead_launches_then_stops_before_conservative_boundary(self):
        plant = BidirectionalPlant()
        moved = held = False
        for frame in range(1, 901):
            p = self.sample(plant, frame)
            add_target(p, longitudinal=60-plant.x, speed=0)
            _, _, output = self.step(plant, p)
            moved = moved or output.throttle > 0
            held = held or output.diagnostics["state"] == "HOLD"
        self.assertTrue(moved)
        self.assertTrue(held)
        self.assertGreater(plant.x, 40)
        self.assertLess(plant.x, 50.5)
        self.assertLess(plant.speed, 0.05)

    def test_real_follow_stops_and_restarts_on_straight_and_curved_routes(self):
        for curved in (False, True):
            with self.subTest(curved=curved):
                self.now[0] = 0
                control_stub._engine.reset()
                decision_stub.reset_decision(self.settings)
                self.follow(curved)

    def follow(self, curved):
        plant = (BidirectionalPlant(x=40, heading=math.pi/2)
                 if curved else BidirectionalPlant())
        lead_s = 30.0
        held = restarted = False
        minimum_gap, max_lateral = float("inf"), 0.0
        for frame in range(1, 901):
            seconds = self.now[0]
            if seconds < 8:
                lead_speed = min(3, 0.75*seconds)
            elif seconds < 12:
                lead_speed = max(0, 3-0.75*(seconds-8))
            elif seconds < 23:
                lead_speed = 0
            else:
                lead_speed = min(3, 0.75*(seconds-23))
            p = self.sample(plant, frame, curved)
            progress = 40*math.atan2(plant.y, plant.x) if curved else plant.x
            target = add_target(p, longitudinal=lead_s-progress, speed=lead_speed)
            if curved:
                yaw = lead_s/40+math.pi/2
                target.x, target.y = 40*math.cos(lead_s/40), 40*math.sin(lead_s/40)
                target.vx, target.vy = lead_speed*math.cos(yaw), lead_speed*math.sin(yaw)
                target.heading = yaw
            _, _, output = self.step(plant, p)
            max_lateral = max(max_lateral, output.diagnostics.get("cross_track_abs_m", 0))
            minimum_gap = min(minimum_gap, lead_s-progress)
            if 18 <= seconds < 23:
                held = held or output.diagnostics["state"] == "HOLD"
            if seconds >= 23 and held and output.throttle > 0:
                restarted = True
            lead_s += 0.05*lead_speed
        self.assertTrue(held)
        self.assertTrue(restarted)
        self.assertGreater(progress, 60)
        self.assertGreater(minimum_gap, 10)
        self.assertLess(max_lateral, 0.3)
        self.assertAlmostEqual(3, plant.speed, delta=0.25)

    def test_red_stop_and_green_restart_use_real_public_outputs(self):
        plant = BidirectionalPlant()
        held_position = restart_time = None
        for frame in range(1, 901):
            p = self.sample(plant, frame)
            p.scene_id = 10
            p.traffic.observed = p.traffic.valid = True
            p.traffic.signal_state = "RED" if self.now[0] < 23 else "GREEN"
            p.traffic.stop_line_distance = max(0, 40-plant.x)
            _, _, output = self.step(plant, p)
            if 20 <= self.now[0] < 23:
                self.assertLess(plant.speed, 0.05)
                held_position = plant.x
            if self.now[0] >= 23 and output.throttle > 0 and restart_time is None:
                restart_time = self.now[0]
        self.assertIsNotNone(held_position)
        self.assertLess(held_position, 33.5)
        self.assertIsNotNone(restart_time)
        self.assertLess(restart_time-23, 5)
        self.assertGreater(plant.x, 80)


if __name__ == "__main__":
    unittest.main()
