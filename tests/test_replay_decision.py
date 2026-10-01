"""Offline tests for the decision snapshot replay tool.

The reconstruction path is the risky part: a snapshot is JSON, and the replay
tool guesses which nested contract object each section belongs to. A silent
mistake there would produce a confident but wrong behaviour report.
"""

import json
import os
import shutil
import sys
import tempfile
import unittest

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from members.decision.engine import DecisionEngine  # noqa: E402
from members.decision.settings import DecisionSettings  # noqa: E402
from scripts import replay_decision  # noqa: E402


def snapshot(frame=42, x=10.0, targets=None, lane_valid=True, targets_valid=True):
    """A captain-shaped snapshot dict, as perception_to_dict would emit.

    `valid_until` is a process-local monotonic deadline, so a saved snapshot is
    always already expired; 0.0 reproduces that faithfully.
    """
    return {
        "frame_id": frame, "timestamp": 9000, "valid_until": 0.0,
        "valid": True, "errors": [], "scene_id": 6, "case_name": "06.test",
        "ego": {"frame_id": frame, "timestamp": 9000, "x": x, "y": 0.0, "z": 0.0,
                "heading": 0.0, "vx": 6.0, "vy": 0.0, "speed": 6.0, "gear": 1,
                "age_ms": 0, "valid": True, "wheel_speeds": [1.0, 1.0],
                "sdk_data": {}, "extra_states": []},
        "lane": {"lane_id": "road1", "valid": lane_valid, "lane_width": 3.5,
                 "center_line": [[0.0, 0.0, 0.0], [200.0, 0.0, 0.0]],
                 "speed_limit": -1.0, "left_boundary": [], "right_boundary": []},
        "traffic": {"signal_state": "UNKNOWN", "observed": False, "valid": False,
                    "stop_line_distance": -1.0, "speed_limit": -1.0},
        "targets": targets if targets is not None else [],
        "targets_valid": targets_valid,
        "source_status": {"targets": {"read_ok": targets_valid,
                                      "usable": targets_valid,
                                      "quality": "ok" if targets_valid else "unavailable",
                                      "age_ms": 0}},
    }


def target_entry(x=30.0, same_lane_valid=True):
    return {"id": 1, "type": 0, "x": x, "y": 0.0, "z": 0.0, "vx": 5.0, "vy": 0.0,
            "length": 4.0, "width": 1.8, "height": 1.5, "probability": 1.0,
            "longitudinal_distance": x - 10.0, "lateral_distance": 0.0,
            "distance": x - 10.0, "relative_speed": -1.0, "valid": True,
            "same_lane": same_lane_valid, "same_lane_valid": same_lane_valid,
            "lane_id": "road1" if same_lane_valid else "", "sdk_data": {}}


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix="decision_replay_")

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def write(self, name, payload):
        path = os.path.join(self.dir, name)
        with open(path, "w", encoding="utf-8") as stream:
            json.dump(payload, stream)
        return path

    def test_snapshot_reconstruction_preserves_nested_types(self):
        path = self.write("frame.json", snapshot(targets=[target_entry()]))
        p = replay_decision.load_perception(path)
        self.assertEqual(42, p.frame_id)
        self.assertTrue(p.ego.valid)
        self.assertEqual(6.0, p.ego.speed)
        self.assertEqual([1.0, 1.0], p.ego.wheel_speeds)
        self.assertEqual("road1", p.lane.lane_id)
        self.assertEqual(200.0, p.lane.center_line[-1][0])
        self.assertEqual(1, len(p.targets))
        self.assertEqual(30.0, p.targets[0].x)
        self.assertTrue(p.targets[0].same_lane_valid)
        self.assertTrue(p.targets_valid)

    def test_replay_reports_the_freshness_guard_by_default(self):
        first = self.write("a.json", snapshot(frame=1))
        second = self.write("b.json", snapshot(frame=2, targets=[target_entry()]))
        report = replay_decision.replay([first, second], DecisionSettings())
        self.assertEqual(2, report["counts"]["frames"])
        self.assertEqual(2, report["modes"]["EMERGENCY_BRAKE"])
        self.assertEqual(2, report["counts"]["lane_valid"])
        self.assertEqual(1, report["counts"]["targets"])
        self.assertEqual(1, report["counts"]["map_verified"])

    def test_replay_reaches_the_behaviour_tree_when_asked_to_revive(self):
        first = self.write("a.json", snapshot(frame=1))
        second = self.write("b.json", snapshot(frame=2, targets=[target_entry()]))
        report = replay_decision.replay([first, second], DecisionSettings(),
                                       revive_ttl=5.0)
        self.assertEqual(1, report["modes"]["KEEP_LANE"])
        self.assertEqual(1, report["modes"]["FOLLOW"])
        self.assertEqual(2, report["counts"]["frames"])

    def test_engine_state_continues_across_replayed_frames(self):
        # The first blind frame is a grace frame; the second latches a blind
        # stop, and because the ego frame reports 6 m/s it must request
        # braking. Latching only happens if one engine sees both frames in
        # order: a per-frame engine would report KEEP_LANE twice.
        paths = [self.write("blind_{0}.json".format(i),
                            snapshot(frame=i + 1, targets_valid=False))
                 for i in range(2)]
        report = replay_decision.replay(paths, DecisionSettings(), revive_ttl=5.0)
        self.assertEqual(1, report["modes"]["KEEP_LANE"])
        self.assertEqual(1, report["modes"]["EMERGENCY_BRAKE"])
        self.assertIn("blind stop", report["frames"][1]["reason"])

    def test_repeated_gps_frames_are_skipped_by_default(self):
        same = dict(snapshot(frame=7))
        paths = [self.write("one.json", same), self.write("two.json", dict(same))]
        skipped = replay_decision.replay(paths, DecisionSettings())
        self.assertEqual(1, skipped["counts"]["frames"])

        kept = replay_decision.replay(paths, DecisionSettings(), stop_on_repeat=False)
        self.assertEqual(2, kept["counts"]["frames"])

    def test_bad_input_is_reported_not_silently_accepted(self):
        not_a_snapshot = self.write("bad.json", {"hello": "world"})
        with self.assertRaises(ValueError):
            replay_decision.load_perception(not_a_snapshot)
        with self.assertRaises(ValueError):
            replay_decision.snapshot_files(os.path.join(self.dir, "missing"))

    def test_directory_discovery_requires_snapshots(self):
        self.write("frame.json", snapshot())
        found = replay_decision.snapshot_files(self.dir)
        self.assertEqual(1, len(found))
        empty = tempfile.mkdtemp(prefix="decision_empty_")
        try:
            with self.assertRaises(ValueError):
                replay_decision.snapshot_files(empty)
        finally:
            shutil.rmtree(empty, ignore_errors=True)

    def test_null_float_is_restored_as_non_finite(self):
        # perception_to_dict writes non-finite floats as null. Reading null
        # back as 0.0 would fabricate a plausible number out of a bad one.
        payload = snapshot()
        payload["ego"]["speed"] = None
        path = self.write("nan.json", payload)
        p = replay_decision.load_perception(path)
        self.assertFalse(p.ego.speed == 0.0)

    def test_report_counters_are_complete_for_every_frame(self):
        self.write("a.json", snapshot(frame=1))
        self.write("b.json", snapshot(frame=2, targets=[target_entry()]))
        report = replay_decision.replay(replay_decision.snapshot_files(self.dir),
                                        DecisionSettings(), revive_ttl=5.0)
        self.assertEqual(2, report["counts"]["frames"])
        self.assertEqual(sum(report["modes"].values()), 2)
        self.assertEqual(sum(report["reasons"].values()), 2)
        for item in report["frames"]:
            for key in ("file", "frame_id", "signal", "target_speed",
                        "stop_distance", "reason", "decision_valid"):
                self.assertIn(key, item)
        self.assertEqual(1, report["counts"]["map_verified"])


if __name__ == "__main__":
    unittest.main()
