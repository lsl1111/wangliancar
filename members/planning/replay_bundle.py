"""Offline component replay, never an SDK recorder or an online safety gate.

The bundle envelope is a proposed file format, not a new public interface.
Compare deadlines with the original capture clock before rebasing them. Keep
source quality, SDK timestamps, failures and repeated/regressed frames intact.
Python 3.6, standard library only; no decision/control/SDK execution.
"""

import argparse
import copy
import json
import math
import os
import sys
from unittest.mock import patch

from core.interfaces import DecisionTarget, Perception, Target
from core.scene_requirements import requires_targets
from core.validation import number
from members.planning.lane_planner import PlannerSettings
from members.planning_stub import plan


FRAME_FIELDS = ("frame_id", "timestamp", "valid_until", "valid")
STAGES = ("perception", "decision", "trajectory", "control")


def _integer(value):
    return type(value) is int and value >= 0


def _finite(value):
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError("nonfinite JSON number")
    if isinstance(value, dict):
        for item in value.values():
            _finite(item)
    elif isinstance(value, list):
        for item in value:
            _finite(item)


def _restore(obj, data, path):
    if not isinstance(data, dict):
        raise ValueError(path + ": expected object")
    for key, value in data.items():
        if key not in vars(obj):
            raise ValueError(path + ": unknown field " + key)
        previous = getattr(obj, key)
        if isinstance(obj, Perception) and key == "targets":
            if not isinstance(value, list):
                raise ValueError(path + ".targets: expected list")
            targets = []
            for index, item in enumerate(value):
                target = Target()
                _restore(target, item, path + ".targets[{0}]".format(index))
                required = ("id", "valid", "x", "y", "vx", "vy", "length", "width")
                if any(field not in item for field in required):
                    raise ValueError(path + ".targets: missing observed target fields")
                targets.append(target)
            setattr(obj, key, targets)
        elif hasattr(previous, "__dict__"):
            _restore(previous, value, path + "." + key)
        else:
            # null remains unknown, including damaged numeric inputs. Never
            # replace it with zero, a default heading or a validity flag.
            setattr(obj, key, copy.deepcopy(value))
    return obj


def _frame_contract(data, source, name):
    errors = []
    if not isinstance(data, dict):
        return [name + "_not_executed"]
    if any(field not in data for field in FRAME_FIELDS):
        return [name + "_contract_fields_missing"]
    if (not _integer(data["frame_id"]) or not _integer(data["timestamp"])
            or not number(data["valid_until"]) or type(data["valid"]) is not bool):
        errors.append(name + "_contract_invalid")
    if source is not None and isinstance(source, dict):
        if (data.get("frame_id") != source.get("frame_id")
                or data.get("timestamp") != source.get("timestamp")):
            errors.append(name + "_frame_mismatch")
        if (number(data.get("valid_until")) and number(source.get("valid_until"))
                and data["valid_until"] > source["valid_until"]):
            errors.append(name + "_deadline_extended")
    return errors


def _source_quality(perception, config):
    source = perception.target_source
    if source == "ground_truth":
        return False, ["diagnostic_ground_truth"]
    if not isinstance(source, str) or not source.startswith("sensor:") or not source[7:]:
        return False, ["formal_sensor_source_missing"]
    status = perception.source_status.get("targets", {})
    issues = []
    if not isinstance(status, dict):
        return False, ["target_source_status_invalid"]
    for field in ("id_verified", "sensor_read_ok", "read_ok", "usable"):
        if status.get(field) is not True:
            issues.append("target_" + field + "_unverified")
    if status.get("quality") != "ok" or perception.targets_valid is not True:
        issues.append("target_quality_unusable")
    if status.get("sensor_id") != source[7:]:
        issues.append("target_sensor_id_mismatch")
    if status.get("source") != source:
        issues.append("target_source_evidence_mismatch")
    for key, observed in (("frame_id", perception.targets_frame_id),
                          ("timestamp", perception.targets_timestamp),
                          ("age_ms", perception.targets_age_ms)):
        if not _integer(status.get(key)) or status[key] != observed:
            issues.append("target_" + key + "_evidence_mismatch")
    if not isinstance(config, dict):
        config = {}
    timeout, gap = config.get("sensor_timeout_ms"), config.get("max_sensor_frame_gap")
    if not number(timeout) or timeout <= 0 or not _integer(gap):
        issues.append("source_quality_thresholds_unknown")
    else:
        if (not _integer(perception.targets_age_ms)
                or perception.targets_age_ms >= timeout):
            issues.append("target_age_unusable")
        if (not _integer(perception.targets_frame_id)
                or abs(perception.targets_frame_id - perception.frame_id) > gap):
            issues.append("target_frame_unsynchronized")
    # SDK target timestamps may differ from GPS timestamps. Their verified
    # source/frame/age policy is the relevant gate, not exact timestamp equality.
    return not issues, issues


def _gps_quality(perception, config):
    status = perception.source_status.get("gps")
    if not isinstance(status, dict):
        return False, ["gps_source_status_missing"]
    issues = []
    if (status.get("read_ok") is not True or status.get("usable") is not True
            or status.get("quality") != "ok" or perception.ego.valid is not True
            or perception.valid is not True):
        issues.append("gps_quality_unusable")
    for key, observed in (("frame_id", perception.frame_id),
                          ("timestamp", perception.timestamp),
                          ("age_ms", perception.ego.age_ms)):
        if not _integer(status.get(key)) or status[key] != observed:
            issues.append("gps_" + key + "_evidence_mismatch")
    timeout = config.get("sensor_timeout_ms") if isinstance(config, dict) else None
    if not number(timeout) or timeout <= 0:
        issues.append("gps_age_threshold_unknown")
    elif not _integer(perception.ego.age_ms) or perception.ego.age_ms >= timeout:
        issues.append("gps_age_unusable")
    return not issues, issues


def _settings(manifest):
    raw = manifest.get("planner_settings")
    if not isinstance(raw, dict):
        raise ValueError("planner_settings_missing")
    expected = set(vars(PlannerSettings()))
    if set(raw) != expected:
        missing, extra = sorted(expected - set(raw)), sorted(set(raw) - expected)
        raise ValueError("planner_settings_schema_mismatch: missing={0}; extra={1}".format(missing, extra))
    settings = PlannerSettings(**copy.deepcopy(raw))
    settings.validate()
    return settings


def _event_review(events, reports):
    if events is None:
        return []
    if not isinstance(events, list):
        return [{"coverage": "invalid", "issues": ["events_must_be_list"]}]
    times = [item["relative_time_s"] for item in reports
             if number(item.get("relative_time_s"))]
    result = []
    for event in events:
        if not isinstance(event, dict):
            result.append({"coverage": "invalid", "issues": ["event_invalid"]})
            continue
        start, end = event.get("start_relative_time_s"), event.get("end_relative_time_s")
        row = {"event_id": event.get("event_id"), "coverage": "invalid", "issues": []}
        if (not isinstance(row["event_id"], str) or not row["event_id"]
                or not number(start) or not number(end) or start < 0 or end < start):
            row["issues"].append("event_interval_invalid")
        else:
            matching = [v for v in times if start <= v <= end]
            row["record_count"] = len(matching)
            if not matching:
                row["coverage"] = "no_records"
                row["issues"].append("event_has_no_records")
            elif min(times) > start or max(times) < end:
                row["coverage"] = "partial"
                row["issues"].append("event_interval_not_fully_recorded")
            else:
                row["coverage"] = "covered"
            # Interval coverage alone does not prove every loop was recorded.
            row["outcome"] = "not_evaluated"
        result.append(row)
    return result


def replay_records(manifest, records, events=None, max_records=10000):
    """Audit a stream and recompute only plan(p, recorded_decision).

    Source labels and shape checks are evidence checks, not independent proof
    that a sensor/case was real. This stateless component replay does not execute
    decision recovery, control, supervision, sending, vehicle dynamics or scoring.
    """
    if not isinstance(manifest, dict):
        raise ValueError("manifest must be an object")
    _finite(manifest)
    if not _integer(max_records) or max_records == 0:
        raise ValueError("max_records must be a positive integer")
    summary = dict((key, 0) for key in (
        "records", "replayed", "replay_valid", "formal_sensor_frames", "gt_frames",
        "expired_inputs", "missing_capture_clock", "gps_repeats", "gps_regressions"))
    report = {"format_version": 1, "recording_kind": manifest.get("recording_kind", "unknown"),
              "delivery": {"errors": [], "warnings": [], "missing_sections": []},
              "summary": summary, "records": [], "events": [],
              "claims": {"real_world_acceptance": "not_evaluated", "score": "not_evaluated",
                         "whole_pipeline_replay": "not_executed", "continuous_capture": "not_proven"}}
    settings = None
    try:
        settings = _settings(manifest)
    except (ValueError, TypeError) as exc:
        report["delivery"]["errors"].append(str(exc))
    session = manifest.get("session_id")
    if not isinstance(session, str) or not session:
        report["delivery"]["errors"].append("manifest_session_id_missing")
    if not manifest.get("code_commit"):
        report["delivery"]["warnings"].append("recorded_code_commit_unknown")
    if not manifest.get("sdk_timestamp_unit_verified"):
        report["delivery"]["warnings"].append("sdk_timestamp_unit_unknown_not_converted")
    if events is None:
        report["delivery"]["missing_sections"].append("events")
    previous_index, previous_time, previous_capture, previous_gps = None, None, None, None
    session_context = None
    for ordinal, original in enumerate(records):
        if ordinal >= max_records:
            report["delivery"]["errors"].append("record_limit_reached_remaining_records_not_checked")
            break
        summary["records"] += 1
        row = {"record_index": None, "relative_time_s": None, "issues": [],
               "source_issues": [], "gps_source_usable": False, "gps_source_issues": [],
               "formal_sensor_usable": False, "gps_event": "unknown", "replay": None}
        report["records"].append(row)
        if not isinstance(original, dict):
            row["issues"].append("record_must_be_object")
            continue
        record = copy.deepcopy(original)
        row["record_index"], row["relative_time_s"] = record.get("record_index"), record.get("relative_time_s")
        try:
            _finite(record)
        except ValueError:
            row["issues"].append("nonfinite_record")
            continue
        if record.get("_read_error"):
            row["issues"].append("jsonl_read_error")
            row["detail"] = record["_read_error"]
            continue
        row["recorded"] = {"recording_errors": copy.deepcopy(record.get("recording_errors")),
                           "safety": copy.deepcopy(record.get("safety")),
                           "send": copy.deepcopy(record.get("send")), "stages": {}}
        for name in STAGES:
            observed = record.get(name)
            row["recorded"]["stages"][name] = (
                dict((key, copy.deepcopy(observed.get(key))) for key in ("valid", "reason", "errors"))
                if isinstance(observed, dict) else None)
        if record.get("session_id") != session or not session:
            row["issues"].append("session_mismatch")
        index, relative, capture = row["record_index"], row["relative_time_s"], record.get("capture_monotonic_s")
        if not _integer(index):
            row["issues"].append("record_index_invalid")
        elif previous_index is not None and index != previous_index + 1:
            row["issues"].append("record_index_discontinuity")
        elif previous_index is None and index != 0:
            row["issues"].append("recording_prefix_missing")
        if _integer(index):
            previous_index = index
        if not number(relative) or relative < 0:
            row["issues"].append("relative_time_invalid")
        elif previous_time is not None and relative < previous_time:
            row["issues"].append("relative_time_regressed")
        if number(relative):
            previous_time = relative
        clock_known = (manifest.get("capture_clock") == "process_monotonic_seconds"
                       and number(capture) and capture >= 0)
        if not clock_known:
            summary["missing_capture_clock"] += 1
            row["issues"].append("capture_clock_missing_or_unverified")
        else:
            if previous_capture is not None and capture < previous_capture:
                row["issues"].append("capture_clock_regressed")
                clock_known = False
            else:
                previous_capture = capture
        if record.get("recording_errors"):
            row["issues"].append("recording_errors_present")
        inputs_ok = True
        parent = None
        for name in STAGES:
            value = record.get(name)
            problems = _frame_contract(value, parent, name)
            row["issues"].extend(problems)
            if name in ("perception", "decision") and problems:
                inputs_ok = False
            parent = value
        for name in ("safety", "send"):
            if not isinstance(record.get(name), dict):
                row["issues"].append(name + "_not_recorded")
        if not inputs_ok:
            continue
        p_data, d_data = record["perception"], record["decision"]
        try:
            required = ("ego", "lane", "targets", "source_status", "target_source", "targets_valid",
                        "scene_id", "case_id", "task_id",
                        "targets_frame_id", "targets_timestamp", "targets_age_ms")
            if any(key not in p_data for key in required):
                raise ValueError("perception: missing observed source fields")
            if not isinstance(p_data["ego"], dict) or any(key not in p_data["ego"] for key in
                    ("frame_id", "timestamp", "valid", "age_ms", "x", "y", "heading", "speed", "vx", "vy", "gear")):
                raise ValueError("ego: missing observed state fields")
            if not isinstance(p_data["source_status"], dict):
                raise ValueError("source_status: expected object")
            if any(key not in d_data for key in
                   ("mode", "target_speed", "target_lane_id", "stop_distance")):
                raise ValueError("decision: missing recorded behavior fields")
            p = _restore(Perception(), p_data, "perception")
            d = _restore(DecisionTarget(), d_data, "decision")
            if p.ego.frame_id != p.frame_id or p.ego.timestamp != p.timestamp:
                raise ValueError("ego frame does not match perception")
        except (ValueError, TypeError, AttributeError) as exc:
            row["issues"].append("input_schema_invalid")
            row["detail"] = str(exc)
            continue
        row["frame_id"], row["timestamp"], row["target_source"] = p.frame_id, p.timestamp, p.target_source
        context = (p.case_id, p.task_id, p.scene_id)
        row["context"] = {"case_id": p.case_id, "task_id": p.task_id, "scene_id": p.scene_id}
        context_ok = (isinstance(p.case_id, str) and bool(p.case_id)
                      and isinstance(p.task_id, str) and bool(p.task_id) and _integer(p.scene_id))
        if not context_ok:
            row["issues"].append("recording_context_unknown")
        elif session_context is None:
            session_context = context
        elif context != session_context:
            row["issues"].append("recording_context_changed_without_new_session")
            context_ok = False
        for field in ("case_id", "task_id", "scene_id"):
            expected = manifest.get(field)
            if expected is not None and row["context"][field] != expected:
                row["issues"].append("manifest_" + field + "_mismatch")
                context_ok = False
        row["gps_source_usable"], row["gps_source_issues"] = _gps_quality(
            p, manifest.get("source_quality_config"))
        if not row["gps_source_usable"]:
            row["issues"].append("required_gps_source_unusable")
        row["formal_sensor_usable"], row["source_issues"] = _source_quality(
            p, manifest.get("source_quality_config"))
        row["targets_required"] = requires_targets(p.scene_id)
        if row["targets_required"] and not row["formal_sensor_usable"]:
            row["issues"].append("required_target_source_unusable")
        summary["formal_sensor_frames"] += int(row["formal_sensor_usable"])
        summary["gt_frames"] += int(p.target_source == "ground_truth")
        if not context_ok:
            previous_gps = None
        signature = p.frame_id, p.timestamp
        row["gps_event"] = "first" if previous_gps is None else "advanced"
        if previous_gps is not None:
            if p.frame_id < previous_gps[0] or p.timestamp < previous_gps[1]:
                row["gps_event"] = "regression"
                summary["gps_regressions"] += 1
            elif p.frame_id == previous_gps[0]:
                row["gps_event"] = "repeat"
                summary["gps_repeats"] += 1
        previous_gps = signature if context_ok else None
        if not clock_known:
            continue
        remaining = min(p.valid_until, d.valid_until) - capture
        row["original_input_remaining_s"] = remaining
        if remaining <= 0:
            summary["expired_inputs"] += 1
            row["issues"].append("input_deadline_expired_at_capture")
            continue
        if (settings is None or not number(relative) or relative < 0
                or "relative_time_regressed" in row["issues"]
                or record.get("session_id") != session or not context_ok
                or not row["gps_source_usable"]
                or (row["targets_required"] and not row["formal_sensor_usable"])):
            row["issues"].append("replay_configuration_or_context_invalid")
            continue
        # Rebase the exact recorded remaining budget, not a fabricated fresh TTL.
        # Call only the real fixed planning entry; freeze its own settings factory
        # to the explicit captured configuration, independent of this shell's env.
        clock = 1000000.0 + relative
        p.valid_until = clock + (p.valid_until - capture)
        d.valid_until = clock + (d.valid_until - capture)
        with patch("core.validation.time.monotonic", return_value=clock), \
                patch.object(PlannerSettings, "from_environment", return_value=settings):
            trajectory = plan(p, d)
        summary["replayed"] += 1
        summary["replay_valid"] += int(trajectory.valid is True)
        row["replay"] = {"valid": trajectory.valid, "emergency_stop": trajectory.emergency_stop,
                         "reason": trajectory.reason, "point_count": len(trajectory.points),
                         "initial_speed": trajectory.points[0].speed if trajectory.points else None,
                         "stop_distance": trajectory.stop_distance,
                         "motion_direction": trajectory.motion_direction,
                         "precision_stop": trajectory.precision_stop,
                         "recorded_input_budget_s": remaining}
    if summary["records"] == 0:
        report["delivery"]["errors"].append("no_frame_records")
    for row in report["records"]:
        if row["issues"]:
            report["delivery"]["errors"].append({"record_index": row["record_index"], "issues": row["issues"]})
    report["events"] = _event_review(events, report["records"])
    for event in report["events"]:
        if event["issues"]:
            report["delivery"]["errors"].append({"event_id": event.get("event_id"), "issues": event["issues"]})
    report["delivery"]["status"] = "incomplete" if report["delivery"]["errors"] or report["delivery"]["missing_sections"] else "structural_only"
    return report


def _jsonl(path):
    with open(path, "r", encoding="utf-8-sig") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                yield {"_read_error": "blank line at {0}".format(line_number)}
                continue
            try:
                yield json.loads(line)
            except ValueError:
                yield {"_read_error": "invalid JSON at line {0}".format(line_number)}


def main(argv=None):
    parser = argparse.ArgumentParser(description="Audit recorded bundle and recompute planning offline; never send control.")
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--frames", required=True)
    parser.add_argument("--events")
    parser.add_argument("--report", required=True)
    parser.add_argument("--max-records", type=int, default=10000)
    args = parser.parse_args(argv)
    output = os.path.normcase(os.path.abspath(args.report))
    inputs = [args.manifest, args.frames] + ([args.events] if args.events else [])
    if output in [os.path.normcase(os.path.abspath(path)) for path in inputs]:
        parser.error("report must not overwrite a bundle input")
    try:
        with open(args.manifest, "r", encoding="utf-8-sig") as stream:
            manifest = json.load(stream)
        events = None
        if args.events:
            with open(args.events, "r", encoding="utf-8-sig") as stream:
                events = json.load(stream)
        report = replay_records(manifest, _jsonl(args.frames), events, args.max_records)
        directory = os.path.dirname(os.path.abspath(args.report))
        if not os.path.isdir(directory):
            os.makedirs(directory)
        with open(args.report, "w", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        print(json.dumps({"delivery": report["delivery"]["status"], "summary": report["summary"],
                          "claims": report["claims"]}, ensure_ascii=False))
        # Incomplete delivery is a useful report, but not a successful acceptance.
        return 1 if report["delivery"]["status"] == "incomplete" else 0
    except (OSError, ValueError, TypeError) as exc:
        print("Replay error: " + str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
