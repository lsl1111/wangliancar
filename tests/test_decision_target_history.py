"""D02 real decision entry behavior with synthetic continuous Sensor API frames."""

import math
import unittest

from core.interfaces import DecisionMode
from members.decision.engine import DecisionEngine
from members.decision.settings import DecisionSettings
from tests.test_decision import perception, add_target


class TargetHistoryTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.settings = DecisionSettings(front_offset_m=3.5, half_width_m=0.9)
        self.engine = DecisionEngine(self.settings, clock=lambda: self.now)

    def observe(self, index, time_s, targets=(), scene=4, source="sensor:test", speed=2.0):
        self.now = time_s
        p = perception(speed=speed, frame_id=index)
        p.scene_id, p.target_source = scene, source
        for identifier, distance, velocity, lateral, lane_id in targets:
            t = add_target(p, longitudinal=distance, speed=velocity, lane_id=lane_id)
            t.id, t.y, t.lateral_distance = identifier, lateral, lateral
        return p, self.engine.run(p)

    def test_previous_safe_lead_cap_merges_with_revealed_second_lead(self):
        _, first = self.observe(1, 0.0, ((1, 20.0, 3.0, 0.0, "lane-1"),))
        p, changed = self.observe(2, 0.1, ((2, 50.0, 8.0, 0.0, "lane-1"),))
        self.assertEqual(DecisionMode.FOLLOW, changed.mode)
        self.assertLessEqual(changed.target_speed, first.target_speed)
        self.assertGreater(changed.target_speed, 0.0)
        self.assertIn("HANDOFF_PREVIOUS_LEAD_CAP", changed.reason)
        self.assertEqual([2], [t.id for t in p.targets])
        self.assertFalse(self.engine._blind_stop)

    def test_safe_disappearance_has_bounded_memory_and_no_stop_latch(self):
        self.observe(1, 0.0, ((1, 20.0, 3.0, 0.0, "lane-1"),))
        p, coast = self.observe(2, 0.1)
        self.assertEqual([], p.targets)
        self.assertGreater(coast.target_speed, 0.0)
        self.assertNotIn(coast.mode, (DecisionMode.STOP, DecisionMode.EMERGENCY_BRAKE))
        _, cleared = self.observe(3, 0.5)
        self.assertEqual(DecisionMode.KEEP_LANE, cleared.mode)
        self.assertFalse(self.engine._blind_stop)

    def test_source_switch_cannot_reuse_old_ids_or_caps(self):
        self.observe(1, 0.0, ((1, 20.0, 3.0, 0.0, "lane-1"),))
        _, result = self.observe(2, 0.1, source="sensor:new")
        self.assertEqual(DecisionMode.KEEP_LANE, result.mode)
        self.assertEqual(0, self.engine.behavior_diagnostics()["target_history"]["track_count"])
        self.assertEqual("SOURCE_CHANGED", self.engine._history.reset_reason)

    def test_followed_car_cut_out_and_returned_stopped_keeps_follow_gap(self):
        self.observe(1, 0.0, ((1, 40.0, 4.0, 0.0, "lane-1"),))
        self.observe(2, 0.1, ((1, 40.4, 4.0, 2.5, "lane-2"),))
        _, stopped = self.observe(3, 0.2, ((1, 40.4, 0.0, 0.0, "lane-1"),))
        self.assertEqual(self.settings.min_gap, stopped.obstacle_clearances_m["1"])
        self.assertGreater(stopped.stop_distance, 0.0)

    def test_id_reuse_with_impossible_position_jump_resets_follow_identity(self):
        self.observe(1, 0.0, ((1, 40.0, 4.0, 0.0, "lane-1"),))
        _, different = self.observe(2, 0.1, ((1, 90.0, 0.0, 0.0, "lane-1"),))
        self.assertEqual(self.settings.obstacle_stop_margin, different.obstacle_clearances_m["1"])
        track = self.engine._history.tracks[1]
        self.assertEqual("ID_DISCONTINUITY", track.quality)
        self.assertFalse(track.was_followed)

    def test_new_near_intrusion_overrides_safe_handoff_cap(self):
        self.observe(1, 0.0, ((1, 30.0, 4.0, 0.0, "lane-1"),))
        _, risk = self.observe(2, 0.1, ((2, 5.0, 0.0, 0.0, "lane-1"),), speed=4.0)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, risk.mode)
        self.assertEqual(0.0, risk.target_speed)

    def test_danger_disappearance_uses_hazard_guard_not_safe_lead_coast(self):
        self.observe(1, 0.0, ((1, 8.0, 0.0, 0.0, "lane-1"),), speed=0.0)
        _, lost = self.observe(2, 0.1, speed=0.0)
        self.assertEqual(DecisionMode.STOP, lost.mode)
        self.assertNotIn("HANDOFF_PREVIOUS_LEAD_CAP", lost.reason)

    def test_repeated_gps_pause_gap_and_budget_are_bounded(self):
        settings = self.settings.replace(history_max_tracks=4, history_samples=2)
        engine = DecisionEngine(settings, clock=lambda: self.now)
        for index in range(1, 5):
            self.now = index * 0.1
            p = perception(speed=0.0, frame_id=index)
            for identifier in range(10):
                t = add_target(p, longitudinal=30 + identifier, speed=3.0)
                t.id = identifier
            engine.run(p)
            engine.run(p)
        snapshot = engine.behavior_diagnostics()["target_history"]
        self.assertLessEqual(snapshot["track_count"], 4)
        self.assertTrue(all(t["sample_count"] <= 2 for t in snapshot["tracks"]))
        self.now = 2.0
        p = perception(speed=0.0, frame_id=5)
        engine.run(p)
        self.assertEqual(0, engine._history.snapshot()["track_count"])

    def test_real_planning_and_control_consume_safe_lead_handoff(self):
        from members.planning.lane_planner import PlannerSettings, build_trajectory
        from members.control.controller import ControlEngine
        from tests.test_control import calibration
        control = ControlEngine(calibration(), clock=lambda: self.now)
        for index, time_s, targets in ((1, 0.0, ((1,20.0,3.0,0.0,"lane-1"),)),
                                       (2, 0.1, ((2,50.0,8.0,0.0,"lane-1"),))):
            p, decision = self.observe(index, time_s, targets)
            p.ego.frame_id, p.ego.age_ms = p.frame_id, 0
            trajectory = build_trajectory(p, decision, PlannerSettings(front_offset_m=3.5, half_width_m=0.9))
            self.assertTrue(trajectory.valid, trajectory.reason)
            self.assertLessEqual(trajectory.target_speed, decision.target_speed)
            output = control.compute(p, trajectory)
            self.assertTrue(output.valid, output.errors)
            self.assertFalse(output.throttle > 0 and output.brake > 0)
        self.assertEqual([2], [target.id for target in p.targets])

    def test_rotation_and_translation_preserve_handoff_speed(self):
        results = []
        for angle, shift in ((0.0, (0.0, 0.0)), (1.1, (100.0, -30.0))):
            engine = DecisionEngine(self.settings, clock=lambda: self.now)
            for index, target_present in ((1, True), (2, False)):
                self.now = 0.1 * (index - 1)
                p = perception(speed=2.0, frame_id=index)
                p.ego.x, p.ego.y = shift
                p.ego.heading = angle
                p.ego.vx, p.ego.vy = 2 * math.cos(angle), 2 * math.sin(angle)
                p.lane.center_line = [(shift[0] + x * math.cos(angle),
                                       shift[1] + x * math.sin(angle)) for x in range(401)]
                if target_present:
                    t = add_target(p, longitudinal=20.0, speed=3.0)
                    t.x, t.y = shift[0] + 20 * math.cos(angle), shift[1] + 20 * math.sin(angle)
                    t.vx, t.vy, t.heading = 3 * math.cos(angle), 3 * math.sin(angle), angle
                decision = engine.run(p)
            results.append(decision.target_speed)
        self.assertAlmostEqual(results[0], results[1], places=6)


if __name__ == "__main__":
    unittest.main()
