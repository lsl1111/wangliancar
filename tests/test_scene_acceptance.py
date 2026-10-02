"""Acceptance checks distinguish clean data from missing or substituted sources."""

import copy
import json
import os
import tempfile
import unittest
from unittest.mock import patch

from scripts.accept_scene import analyze_frames, collect


def frame(number):
    return {
        "frame_id": number, "scene_id": 6, "case_id": "test-06",
        "case_name": "06.车道居中控制", "valid": True,
        "ego": {"valid": True, "frame_id": number, "x": float(number),
                "y": 0.0, "heading": 0.0, "speed": 1.0, "vx": 1.0,
                "vy": 0.0, "acceleration": 0.0},
        "source_status": {"gps": {"usable": True},
                          "targets": {"usable": True}},
        "targets_valid": True, "targets": [],
        "target_source": "sensor:perfectPerception1",
        "sensor_configurations_valid": True,
        "sensor_configurations": [{"id": "perfectPerception1"}],
        "lane": {"valid": True, "lane_id": "1_0_-1",
                 "center_line": [[0, 0, 0], [100, 0, 0]],
                 "left_boundary": [[0, 2, 0], [100, 2, 0]],
                 "right_boundary": [[0, -2, 0], [100, -2, 0]],
                 "lane_width": 4.0, "lane_width_valid": True,
                 "lateral_offset": 0.0, "heading_error": 0.0},
    }


class SceneAcceptanceTests(unittest.TestCase):
    def test_no_live_frames_is_pending_data_not_acceptance(self):
        report = analyze_frames([], 6, 2)
        self.assertEqual("NO_DATA", report["status"])

    def test_valid_empty_sensor_target_frames_pass_structure_only(self):
        report = analyze_frames([frame(1), frame(2)], 6, 2)
        self.assertEqual("STRUCTURAL_PASS", report["status"])
        self.assertEqual("PENDING", report["event_review"])
        self.assertEqual(0, report["counts"]["target_frames_with_objects"])

    def test_ground_truth_is_not_sensor_acceptance(self):
        values = [frame(1), frame(2)]
        for value in values:
            value["scene_id"] = 1
            value["target_source"] = "ground_truth"
        report = analyze_frames(values, 1, 2)
        self.assertEqual("FAIL_OR_INCOMPLETE", report["status"])
        self.assertEqual(0, report["counts"]["targets_ok"])
        self.assertTrue(any("ground-truth" in warning for warning in report["warnings"]))

    def test_scene_six_needs_no_target_sensor(self):
        values = [frame(1), frame(2)]
        for value in values:
            value["targets_valid"] = False
            value["source_status"]["targets"] = {"usable": False,
                                                   "sensor_presence": "not_configured"}
            value["target_source"] = "none"
            value["sensor_configurations"] = []
        report = analyze_frames(values, 6, 2)
        self.assertEqual("STRUCTURAL_PASS", report["status"])
        self.assertEqual(0, report["counts"]["targets_ok"])

    def test_wrong_scene_and_stalled_frame_fail(self):
        first = frame(1)
        second = copy.deepcopy(first)
        second["scene_id"] = 5
        report = analyze_frames([first, second], 6, 2)
        self.assertEqual("FAIL_OR_INCOMPLETE", report["status"])
        self.assertTrue(any("strictly increasing" in failure for failure in report["failures"]))

    def test_real_callback_id_can_be_verified_when_catalog_is_unavailable(self):
        values = [frame(1), frame(2)]
        for value in values:
            value["scene_id"] = 1
            value["sensor_configurations_valid"] = False
            value["sensor_configurations"] = []
            value["source_status"]["targets"].update(
                id_verified=True, sensor_read_ok=True,
                callback_sensor_ids=["perfectPerception1"])
        report = analyze_frames(values, 1, 2)
        self.assertEqual("STRUCTURAL_PASS", report["status"])
        self.assertEqual(0, report["counts"]["sensor_config_ok"])
        self.assertEqual(2, report["counts"]["target_id_verified"])
        self.assertTrue(any("configuration query remains unavailable" in warning
                            for warning in report["warnings"]))
        values[1]["source_status"]["targets"]["usable"] = False
        self.assertEqual("FAIL_OR_INCOMPLETE", analyze_frames(values, 1, 2)["status"])

    def test_default_id_probe_alone_is_not_sensor_id_verification(self):
        value = frame(1)
        value["scene_id"] = 1
        value["sensor_configurations_valid"] = False
        value["sensor_configurations"] = []
        value["source_status"]["targets"].update(id_verified=False, sensor_read_ok=True)
        self.assertEqual("FAIL_OR_INCOMPLETE", analyze_frames([value], 1, 1)["status"])

    def test_live_capture_skips_a_previous_run_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "latest_perception.json")
            with open(path, "w", encoding="utf-8") as stream:
                json.dump(frame(1), stream)
            with patch("scripts.accept_scene.time.time", return_value=100.0), \
                    patch("scripts.accept_scene.os.path.getmtime", return_value=99.0), \
                    patch("scripts.accept_scene.time.monotonic", side_effect=[0.0, 0.0, 2.0]), \
                    patch("scripts.accept_scene.time.sleep"):
                self.assertEqual([], collect(path, 1, 1.0))


if __name__ == "__main__":
    unittest.main()
