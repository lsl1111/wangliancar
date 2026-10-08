"""Public decide output versus separate unconnected D01 proposal diagnostics."""

import unittest
from unittest.mock import patch

from core.interfaces import DecisionTarget
from members.decision.engine import DecisionEngine
from members.decision.settings import DecisionSettings
from members.decision_stub import decide, reset_decision, decision_behavior_info
from tests.test_decision import perception, add_target, add_red_light


class BehaviorProposalTests(unittest.TestCase):
    def setUp(self):
        self.settings = DecisionSettings(front_offset_m=3.5, half_width_m=0.9)
        reset_decision(self.settings)
        self.addCleanup(reset_decision)

    def test_blocked_road_real_decide_produces_inactive_hazard_request(self):
        p = perception(speed=0.0)
        p.scene_id = 11
        add_target(p, longitudinal=30.0, speed=0.0)
        output = decide(p)
        self.assertTrue(output.valid, output.reason)
        self.assertEqual(set(vars(DecisionTarget())), set(vars(output)))
        requests = decision_behavior_info()["requests"]
        self.assertEqual(1, len(requests))
        self.assertEqual("BLOCKED_ROAD", requests[0]["maneuver"])
        self.assertTrue(requests[0]["light_intent"]["hazard_signal"])
        self.assertFalse(requests[0]["dispatch_allowed"])
        self.assertFalse(hasattr(output, "hazard_signal"))

    def test_normal_red_and_stopped_following_are_not_unresolved_blockage(self):
        for scene in (10, 14):
            reset_decision(self.settings)
            p = perception(speed=0.0)
            p.scene_id = scene
            if scene == 10:
                add_red_light(p, 30.0)
            else:
                add_target(p, longitudinal=30.0, speed=0.0)
            self.assertTrue(decide(p).valid)
            self.assertFalse(any(r["maneuver"] == "BLOCKED_ROAD"
                                 for r in decision_behavior_info()["requests"]))

    def test_scene_twenty_requests_control_dwell_but_does_not_claim_it_completed(self):
        p = perception(speed=0.0, frame_id=1)
        p.scene_id = 20
        add_red_light(p, 30.0)
        p.traffic.signal_id = 42
        first_output = decide(p)
        proposal = decision_behavior_info()["requests"][0]
        self.assertEqual("SIGNAL_STOP", proposal["maneuver"])
        self.assertGreater(proposal["minimum_standstill_duration_s"], 5.0)
        self.assertFalse(proposal["dispatch_allowed"])
        p.frame_id = 2
        p.traffic.signal_state = "GREEN"
        decide(p)
        continued = decision_behavior_info()["requests"][0]
        self.assertEqual(proposal["intent_id"], continued["intent_id"])
        self.assertNotEqual("COMPLETED", continued["status"])
        self.assertFalse(hasattr(first_output, "hold_duration_s"))

    def test_invalid_new_context_does_not_reset_pending_proposal(self):
        engine = DecisionEngine(self.settings)
        p = perception(speed=0.0, frame_id=1)
        p.scene_id = 20
        add_red_light(p, 30.0)
        p.traffic.signal_id = 42
        engine.run(p)
        initial = engine.behavior_diagnostics()["requests"][0]["intent_id"]
        p.ego.valid = False
        p.case_id = "untrusted-new-case"
        engine.run(p)
        self.assertEqual(initial, engine.behavior_diagnostics()["requests"][0]["intent_id"])

    def test_diagnostic_copy_and_failure_cannot_alter_normal_decision(self):
        engine = DecisionEngine(self.settings)
        p = perception(speed=0.0)
        baseline = engine.run(p)
        diagnostic = engine.behavior_diagnostics()
        diagnostic["dependencies"].clear()
        self.assertTrue(engine.behavior_diagnostics()["dependencies"])
        with patch.object(engine._behavior, "observe_legacy", side_effect=ValueError("synthetic failure")):
            actual = engine.run(p)
        self.assertTrue(actual.valid, actual.reason)
        self.assertEqual((baseline.mode, baseline.target_speed, baseline.stop_distance),
                         (actual.mode, actual.target_speed, actual.stop_distance))
        self.assertIn("BEHAVIOR_DIAGNOSTIC_ERROR", engine.behavior_diagnostics()["reason_code"])


if __name__ == "__main__":
    unittest.main()
