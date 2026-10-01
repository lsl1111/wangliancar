"""Decision, path stop and controller handoff with synthetic vehicle dimensions.

These dimensions and pedal mapping are test fixtures, not SimOne calibration.
"""

import unittest

from members.control.controller import ControlEngine
from members.decision.engine import DecisionEngine
from members.decision.settings import DecisionSettings
from members.planning.lane_planner import PlannerSettings, build_trajectory
from tests.test_control import calibration
from tests.test_decision import add_target, perception


class ObstaclePipelineTests(unittest.TestCase):
    def run_case(self, ego_x, ego_speed, target_x=30.0):
        p = perception(speed=ego_speed, frame_id=int(ego_x) + 20)
        p.ego.x = ego_x
        p.ego.frame_id, p.ego.age_ms = p.frame_id, 0
        add_target(p, longitudinal=target_x - ego_x, speed=0.0, length=4.0)
        decision = DecisionEngine(DecisionSettings(front_offset_m=3.5)).run(p)
        trajectory = build_trajectory(
            p, decision, PlannerSettings(front_offset_m=3.5, half_width_m=0.9))
        control = ControlEngine(calibration()).compute(p, trajectory)
        return decision, trajectory, control

    def test_distant_stopped_lead_allows_launch_with_a_stop_boundary(self):
        decision, trajectory, control = self.run_case(0.0, 0.0)
        self.assertEqual("FOLLOW", decision.mode)
        self.assertGreater(decision.target_speed, 0.0)
        self.assertTrue(trajectory.valid, trajectory.reason)
        self.assertTrue(trajectory.stop_required)
        self.assertGreater(trajectory.target_speed, 0.0)
        self.assertEqual(0.0, trajectory.points[-1].speed)
        self.assertTrue(control.valid, control.errors)
        self.assertGreater(control.throttle, 0.0)
        self.assertEqual(0.0, control.brake)

    def test_infeasible_approach_requests_braking(self):
        decision, trajectory, control = self.run_case(16.0, 5.0)
        self.assertEqual("EMERGENCY_BRAKE", decision.mode)
        self.assertTrue(trajectory.emergency_stop)
        self.assertTrue(control.valid, control.errors)
        self.assertEqual(0.0, control.throttle)
        self.assertGreater(control.brake, 0.0)


if __name__ == "__main__":
    unittest.main()
