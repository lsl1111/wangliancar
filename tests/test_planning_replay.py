"""Offline recording checks; authored fixtures do not establish scene acceptance."""

import copy
import os
import unittest
from unittest.mock import patch

from core.interfaces import ControlOut, DecisionTarget, Perception, Target
from core.serialization import to_dict
from members.planning.lane_planner import PlannerSettings, build_trajectory
from members.planning.replay_bundle import replay_records


def manifest():
    return {
        "session_id": "synthetic-replay-session",
        "recording_kind": "continuous_runtime_loop",
        "data_kind": "synthetic_authored_regression_fixture",
        "capture_clock": "process_monotonic_seconds",
        "planner_settings": vars(PlannerSettings(
            front_offset_m=3.9187, half_width_m=0.9)),
        "source_quality_config": {
            "sensor_timeout_ms": 500, "max_sensor_frame_gap": 10},
    }


def record(index=0, frame_id=10, timestamp=10000, relative_time=0.0):
    capture = 1000.0 + relative_time
    perception = Perception()
    perception.frame_id = perception.ego.frame_id = frame_id
    perception.timestamp = perception.ego.timestamp = timestamp
    perception.valid_until = capture + 0.2
    perception.valid = perception.ego.valid = perception.lane.valid = True
    perception.scene_id = 15
    perception.case_id, perception.task_id = "synthetic-case", "synthetic-task"
    perception.ego.speed = perception.ego.vx = 2.0
    perception.ego.age_ms = 0
    perception.lane.lane_id = "synthetic-lane"
    perception.lane.center_line = [(0.0, 0.0), (200.0, 0.0)]
    perception.lane.lane_width_valid = True
    perception.lane.lane_width = 3.5
    perception.target_source = "sensor:synthetic-fusion"
    perception.targets_valid = True
    perception.targets_frame_id = frame_id
    perception.targets_timestamp = timestamp
    perception.targets_age_ms = 0
    perception.source_status = {
        "gps": {"read_ok": True, "usable": True, "quality": "ok",
                "frame_id": frame_id, "timestamp": timestamp, "age_ms": 0},
        "targets": {
            "sensor_id": "synthetic-fusion", "sensor_presence": "configured",
            "id_verified": True, "sensor_read_ok": True, "read_ok": True,
            "source": perception.target_source, "transport": "callback",
            "usable": True, "quality": "ok", "frame_id": frame_id,
            "timestamp": timestamp, "age_ms": 0},
    }
    decision = DecisionTarget().bind(perception)
    decision.valid, decision.target_speed = True, 5.0
    decision.target_lane_id = perception.lane.lane_id
    settings = PlannerSettings(front_offset_m=3.9187, half_width_m=0.9)
    with patch("core.validation.time.monotonic", return_value=capture):
        trajectory = build_trajectory(perception, decision, settings)
    if not trajectory.valid:
        raise AssertionError("fixture trajectory: " + trajectory.reason)
    control = ControlOut().bind(trajectory)
    control.valid, control.gear, control.throttle = True, 1, 0.1
    return {
        "session_id": "synthetic-replay-session",
        "record_index": index,
        "relative_time_s": relative_time,
        "capture_monotonic_s": capture,
        "perception": to_dict(perception),
        "decision": to_dict(decision),
        "trajectory": to_dict(trajectory),
        "control": to_dict(control),
        "safety": {"mode": "none", "reason": "fixture", "candidate": None},
        "send": {"attempted": False, "ok": False, "reason": "disabled"},
        "recording_errors": [],
    }


def issue_text(report):
    """Read reported findings without relying on a particular reason spelling."""
    return str(report).lower()


class PlanningReplayTests(unittest.TestCase):
    def test_recomputes_valid_input_with_explicit_capture_clock(self):
        item = record()
        # The replay must calculate from inputs rather than copy a recorded result.
        item["trajectory"]["points"][0]["speed"] = 77.0
        report = replay_records(manifest(), [item])
        summary, result = report["summary"], report["records"][0]
        self.assertEqual(1, summary["records"])
        self.assertEqual(1, summary["replayed"])
        self.assertEqual(1, summary["replay_valid"])
        self.assertEqual(1, summary["formal_sensor_frames"])
        self.assertTrue(result["formal_sensor_usable"])
        self.assertTrue(result["replay"]["valid"])
        self.assertEqual(2.0, result["replay"]["initial_speed"])
        self.assertGreater(result["replay"]["point_count"], 2)
        self.assertFalse(result["replay"]["emergency_stop"])
        self.assertEqual("not_evaluated", report["claims"]["real_world_acceptance"])
        self.assertEqual("not_evaluated", report["claims"]["score"])

    def test_expired_original_deadlines_cannot_be_revived(self):
        for component in ("perception", "decision"):
            with self.subTest(component=component):
                item = record()
                item[component]["valid_until"] = item["capture_monotonic_s"]
                if component == "perception":
                    # Keep the source contract intact to isolate original expiry.
                    item["decision"]["valid_until"] = item[component]["valid_until"]
                report = replay_records(manifest(), [item])
                self.assertIsNone(report["records"][0]["replay"])
                self.assertEqual(1, report["summary"]["expired_inputs"])
                self.assertIn("expir", issue_text(report["records"][0]["issues"]))

    def test_missing_original_capture_clock_cannot_be_replaced_by_fresh_ttl(self):
        item = record()
        del item["capture_monotonic_s"]
        report = replay_records(manifest(), [item])
        self.assertIsNone(report["records"][0]["replay"])
        self.assertEqual(1, report["summary"]["missing_capture_clock"])

    def test_source_mismatch_and_extended_deadline_are_not_repaired(self):
        for change in ("frame", "timestamp", "deadline"):
            with self.subTest(change=change):
                item = record()
                decision = item["decision"]
                if change == "frame":
                    decision["frame_id"] += 1
                elif change == "timestamp":
                    decision["timestamp"] += 1
                else:
                    decision["valid_until"] += 1.0
                before = copy.deepcopy(item)
                report = replay_records(manifest(), [item])
                result = report["records"][0]
                self.assertTrue(result["issues"])
                self.assertTrue(result["replay"] is None or
                                not result["replay"]["valid"])
                self.assertEqual(before, item)

    def test_gt_and_unverified_sensor_do_not_count_as_formal_sensor(self):
        for source, verified in (("ground_truth", False),
                                 ("sensor:synthetic-fusion", False)):
            with self.subTest(source=source):
                item = record()
                perception = item["perception"]
                perception["target_source"] = source
                perception["source_status"]["targets"].update(
                    source=source, id_verified=verified)
                report = replay_records(manifest(), [item])
                self.assertFalse(report["records"][0]["formal_sensor_usable"])
                self.assertEqual(0, report["summary"]["formal_sensor_frames"])
                self.assertEqual(int(source == "ground_truth"),
                                 report["summary"]["gt_frames"])

    def test_async_target_timestamp_is_preserved_and_not_treated_as_module_mismatch(self):
        item = record()
        item["perception"]["targets_timestamp"] += 1
        item["perception"]["source_status"]["targets"]["timestamp"] += 1
        before = copy.deepcopy(item)
        report = replay_records(manifest(), [item])
        self.assertTrue(report["records"][0]["formal_sensor_usable"])
        self.assertEqual(1, report["summary"]["formal_sensor_frames"])
        self.assertTrue(report["records"][0]["replay"]["valid"])
        self.assertEqual(before, item)

    def test_stale_or_failed_target_source_cannot_count_as_formal(self):
        for change in ("age", "read", "usable", "frame_gap"):
            with self.subTest(change=change):
                item = record()
                perception = item["perception"]
                status = perception["source_status"]["targets"]
                if change == "age":
                    perception["targets_age_ms"] = status["age_ms"] = 501
                elif change == "read":
                    status["sensor_read_ok"] = False
                elif change == "usable":
                    status["usable"] = False
                else:
                    perception["targets_frame_id"] = status["frame_id"] = 100
                report = replay_records(manifest(), [item])
                self.assertFalse(report["records"][0]["formal_sensor_usable"])
                self.assertEqual(0, report["summary"]["formal_sensor_frames"])

    def test_duplicate_and_regressed_gps_rows_remain_visible(self):
        rows = [record(), record(1, 10, 10000, 0.05),
                record(2, 9, 9999, 0.10)]
        before = copy.deepcopy(rows)
        report = replay_records(manifest(), rows)
        self.assertEqual(3, report["summary"]["records"])
        self.assertEqual(3, len(report["records"]))
        self.assertEqual(1, report["summary"]["gps_repeats"])
        self.assertEqual(1, report["summary"]["gps_regressions"])
        self.assertIn("repeat", str(report["records"][1]["gps_event"]).lower())
        self.assertIn("regress", str(report["records"][2]["gps_event"]).lower())
        self.assertEqual(before, rows)

    def test_contradictory_target_status_cannot_be_hidden_by_top_level_flags(self):
        changes = {"source": "ground_truth", "age_ms": 9999,
                   "frame_id": 9, "timestamp": 9999}
        for field, value in changes.items():
            with self.subTest(field=field):
                item = record()
                item["perception"]["source_status"]["targets"][field] = value
                report = replay_records(manifest(), [item])
                self.assertFalse(report["records"][0]["formal_sensor_usable"])
                self.assertEqual(0, report["summary"]["formal_sensor_frames"])
                self.assertTrue(report["records"][0]["source_issues"])

    def test_null_heading_is_not_filled_with_zero_to_accept_lateral_motion(self):
        item = record()
        target = Target()
        target.id, target.valid = 9001, True
        target.x, target.vx, target.vy = 40.0, 5.0, 0.01
        target.length, target.width = 4.0, 1.8
        target.heading = None
        target.same_lane_valid = target.same_lane = True
        target.lane_id = item["perception"]["lane"]["lane_id"]
        target.source = item["perception"]["target_source"]
        item["perception"]["targets"] = [to_dict(target)]
        report = replay_records(manifest(), [item])
        self.assertIsNone(item["perception"]["targets"][0]["heading"])
        self.assertIsNotNone(report["records"][0]["replay"])
        self.assertFalse(report["records"][0]["replay"]["valid"])

    def test_unknown_public_fields_are_rejected_instead_of_attached(self):
        for component, field in (("perception", "execution_feedback_proposal"),
                                 ("decision", "maneuver_goal_proposal")):
            with self.subTest(component=component):
                item = record()
                item[component][field] = {"status": "unapproved"}
                report = replay_records(manifest(), [item])
                self.assertIsNone(report["records"][0]["replay"])
                self.assertIn("schema", issue_text(report["records"][0]["issues"]))

    def test_missing_recorded_decision_fields_do_not_become_default_intents(self):
        for field in ("mode", "target_speed", "target_lane_id", "stop_distance"):
            with self.subTest(field=field):
                item = record()
                del item["decision"][field]
                report = replay_records(manifest(), [item])
                self.assertIsNone(report["records"][0]["replay"])
                self.assertIn("schema", issue_text(report["records"][0]["issues"]))

    def test_unexecuted_outputs_remain_missing_observations(self):
        item = record()
        for name in ("trajectory", "control", "safety", "send"):
            item[name] = None
        before = copy.deepcopy(item)
        report = replay_records(manifest(), [item])
        self.assertEqual(before, item)
        self.assertTrue(report["records"][0]["issues"])
        self.assertEqual("not_evaluated", report["claims"]["real_world_acceptance"])
        self.assertEqual("not_evaluated", report["claims"]["score"])

    def test_explicit_settings_do_not_use_the_readers_environment(self):
        with patch.dict(os.environ, {"NEVC_VEHICLE_FRONT_OFFSET_M": "invalid",
                                     "NEVC_PLANNING_DECELERATION_MPS2": "99"}):
            report = replay_records(manifest(), [record()])
        self.assertTrue(report["records"][0]["replay"]["valid"])
        self.assertEqual(1, report["summary"]["replay_valid"])

    def test_record_index_gap_and_uncovered_event_are_reported(self):
        rows = [record(), record(3, 11, 10050, 0.05)]
        events = [{"event_id": "synthetic-event", "event_kind": "stop_go",
                   "start_relative_time_s": 0.0, "end_relative_time_s": 0.2,
                   "expected_behavior": "stop and resume",
                   "observed_result": None, "score": None}]
        report = replay_records(manifest(), rows, events=events)
        self.assertEqual(2, report["summary"]["records"])
        self.assertEqual(2, len(report["records"]))
        self.assertIn("index", issue_text(report["records"][1]["issues"]))
        self.assertTrue(report["delivery"]["errors"])
        self.assertEqual("partial", report["events"][0]["coverage"])
        self.assertTrue(report["events"][0]["issues"])

    def test_all_input_containers_are_left_unchanged(self):
        header, rows = manifest(), [record()]
        events = []
        before = copy.deepcopy((header, rows, events))
        replay_records(header, rows, events=events)
        self.assertEqual(before, (header, rows, events))

    def test_required_target_quality_failure_blocks_replay_and_delivery(self):
        for failure in ("id", "read", "stale"):
            with self.subTest(failure=failure):
                item = record()
                perception = item["perception"]
                status = perception["source_status"]["targets"]
                if failure == "id":
                    status["id_verified"] = False
                elif failure == "read":
                    status["sensor_read_ok"] = False
                else:
                    status["age_ms"] = perception["targets_age_ms"] = 501
                report = replay_records(manifest(), [item], events=[])
                result = report["records"][0]
                self.assertTrue(result["targets_required"])
                self.assertFalse(result["formal_sensor_usable"])
                self.assertIsNone(result["replay"])
                self.assertEqual(0, report["summary"]["replayed"])
                self.assertEqual("incomplete", report["delivery"]["status"])
                self.assertIn("target", issue_text(result["issues"]))

    def test_missing_failed_stale_or_mismatched_gps_blocks_replay(self):
        for failure in ("missing", "read", "usable", "stale", "frame", "timestamp"):
            with self.subTest(failure=failure):
                item = record()
                perception = item["perception"]
                status = perception["source_status"]["gps"]
                if failure == "missing":
                    del perception["source_status"]["gps"]
                elif failure == "read":
                    status["read_ok"] = False
                elif failure == "usable":
                    status["usable"] = False
                elif failure == "stale":
                    status["age_ms"] = perception["ego"]["age_ms"] = 501
                elif failure == "frame":
                    status["frame_id"] += 1
                else:
                    status["timestamp"] += 1
                report = replay_records(manifest(), [item], events=[])
                result = report["records"][0]
                self.assertFalse(result["gps_source_usable"])
                self.assertTrue(result["gps_source_issues"])
                self.assertIsNone(result["replay"])
                self.assertEqual(0, report["summary"]["replayed"])
                self.assertEqual("incomplete", report["delivery"]["status"])

    def test_regressed_capture_clock_cannot_manufacture_a_longer_input_budget(self):
        rows = [record(), record(1, 11, 10050, 0.05)]
        # Keep the actual deadlines but forge an earlier capture time. Subtracting
        # it would manufacture hundreds of seconds of apparent remaining life.
        rows[1]["capture_monotonic_s"] = 500.0
        before = copy.deepcopy(rows)
        report = replay_records(manifest(), rows, events=[])
        self.assertEqual(2, report["summary"]["records"])
        self.assertEqual(1, report["summary"]["replayed"])
        self.assertIsNone(report["records"][1]["replay"])
        self.assertIn("capture_clock_regressed", report["records"][1]["issues"])
        self.assertEqual("incomplete", report["delivery"]["status"])
        self.assertEqual(before, rows)

    def test_missing_gear_or_recorded_context_cannot_be_defaulted(self):
        for field in ("gear", "case_id", "task_id", "scene_id"):
            with self.subTest(field=field):
                item = record()
                if field == "gear":
                    del item["perception"]["ego"][field]
                else:
                    del item["perception"][field]
                before = copy.deepcopy(item)
                report = replay_records(manifest(), [item], events=[])
                self.assertIsNone(report["records"][0]["replay"])
                self.assertIn("schema", issue_text(report["records"][0]["issues"]))
                self.assertEqual("incomplete", report["delivery"]["status"])
                self.assertEqual(before, item)

    def test_task_change_without_new_session_blocks_and_breaks_gps_continuity(self):
        rows = [record(), record(1, 1, 100, 0.05),
                record(2, 11, 10050, 0.10)]
        rows[1]["perception"]["task_id"] = "different-synthetic-task"
        report = replay_records(manifest(), rows, events=[])
        self.assertEqual(3, report["summary"]["records"])
        self.assertEqual(3, len(report["records"]))
        self.assertIsNone(report["records"][1]["replay"])
        self.assertIn("context", issue_text(report["records"][1]["issues"]))
        self.assertEqual("first", report["records"][1]["gps_event"])
        self.assertEqual("first", report["records"][2]["gps_event"])
        self.assertEqual(0, report["summary"]["gps_regressions"])
        self.assertEqual(0, report["summary"]["gps_repeats"])
        self.assertEqual("incomplete", report["delivery"]["status"])

    def test_recording_and_component_errors_and_reasons_remain_original_evidence(self):
        item = record()
        item["recording_errors"] = ["disk write failed", {"stage": "capture", "detail": "full"}]
        for name in ("perception", "decision", "trajectory", "control"):
            item[name]["errors"] = [name + ": original diagnostic error"]
        item["decision"]["reason"] = "original decision reason"
        item["trajectory"]["reason"] = "original trajectory rejection"
        item["trajectory"]["valid"] = False
        item["safety"] = {"mode": "fault_stop", "reason": "original safety cause",
                          "errors": ["original safety error"], "candidate": None}
        item["send"] = {"attempted": True, "ok": False, "reason": "original send rejection",
                        "adapter": {"error": "original adapter failure"}}
        before = copy.deepcopy(item)
        report = replay_records(manifest(), [item], events=[])
        recorded = report["records"][0]["recorded"]
        self.assertEqual(before["recording_errors"], recorded["recording_errors"])
        self.assertEqual(before["safety"], recorded["safety"])
        self.assertEqual(before["send"], recorded["send"])
        for name in ("perception", "decision", "trajectory", "control"):
            self.assertEqual(before[name]["errors"], recorded["stages"][name]["errors"])
            self.assertEqual(before[name].get("reason"), recorded["stages"][name]["reason"])
            self.assertEqual(before[name]["valid"], recorded["stages"][name]["valid"])
        self.assertIn("recording_errors_present", report["records"][0]["issues"])
        self.assertEqual("incomplete", report["delivery"]["status"])
        self.assertEqual(before, item)
        # The report also owns detached evidence, so editing it cannot rewrite
        # the supplied recording or hide the original failure.
        recorded["recording_errors"].append("report annotation")
        recorded["stages"]["decision"]["errors"].append("report annotation")
        self.assertEqual(before, item)


if __name__ == "__main__":
    unittest.main()
