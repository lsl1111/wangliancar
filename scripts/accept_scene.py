"""Read-only acceptance of captain Perception snapshots (Python 3.6)."""

from __future__ import print_function

import argparse
import json
import math
import os
import sys
import time


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from core.scene_requirements import requires_targets

DEFAULT_SNAPSHOT = os.path.join(PROJECT_DIR, "runtime_data", "latest_perception.json")
DEFAULT_OUTPUT = os.path.join(PROJECT_DIR, "runtime_data", "scene_acceptance")

# These are data-channel requirements, not a claim that the scenario behaviour works.
SCENES = {
    1: ("前方车辆静止", "targets"), 2: ("前方车辆制动", "targets"),
    3: ("前方行人横穿", "targets"), 4: ("直道车道偏离抑制", "lane"),
    5: ("弯道车道偏离抑制", "lane"), 6: ("车道居中控制", "lane"),
    7: ("垂直泊车", "parking"), 8: ("平行泊车", "parking"),
    9: ("限速标志识别及响应", "speed_sign"),
    10: ("机动车信号灯识别及响应", "traffic_light"),
    11: ("系统无法处置的场景", "targets"),
    12: ("自动紧急避让", "targets"), 13: ("前方障碍物起步", "targets"),
    14: ("稳定跟车", "targets"), 15: ("弯道内跟车", "targets"),
    16: ("避让障碍物变道", "lane_change"),
    17: ("避让低速行驶车辆变道", "lane_change"),
    18: ("无信号灯路口车辆冲突通行", "intersection"),
    19: ("车道线识别及响应", "lane_mark"),
    20: ("停止线识别及响应", "stop_line"),
    21: ("左侧车辆通行起步", "targets"),
    22: ("上坡-下坡路跟车", "targets"),
    23: ("跟车时前车切出", "targets"),
    24: ("跟车时邻车道车辆切入", "targets"),
    25: ("停-走功能", "targets"),
    26: ("避让故障车辆变道", "lane_change"),
    27: ("避让事故车辆变道", "lane_change"),
    28: ("临近车道有车变道", "lane_change"),
    29: ("前方车道减少变道", "lane_change"),
    30: ("无信号灯路口非机动车冲突通行", "intersection"),
    31: ("路口车辆冲突通行", "intersection"),
    32: ("拥堵路口通行", "intersection"),
    33: ("群体行人通行", "vulnerable"),
    34: ("群体非机动车通行", "vulnerable"),
    35: ("行人和非机动车通行", "vulnerable"),
    36: ("行人折返通行", "vulnerable"),
    37: ("行人违章通行", "vulnerable"),
    38: ("非机动车违章通行", "vulnerable"),
    39: ("事故工况-对向冲突", "intersection"),
    40: ("事故工况-冲突对象突然出现", "targets"),
    41: ("连续赛道", "continuous"),
}


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _dict(value):
    return value if isinstance(value, dict) else {}


def analyze_frames(frames, scene_id, minimum=20):
    """Check published field integrity; events still need manual scene review."""
    if scene_id not in SCENES:
        raise ValueError("scene_id must be 1..41")
    name, capability = SCENES[scene_id]
    target_required = requires_targets(scene_id)
    failures, warnings = [], []
    counts = {"gps_ok": 0, "targets_ok": 0, "lane_ok": 0,
              "sensor_config_ok": 0, "target_id_verified": 0, "target_frames_with_objects": 0,
              "traffic_frames": 0, "sign_frames": 0, "route_frames": 0,
              "parking_frames": 0, "stop_line_frames": 0,
              "crosswalk_frames": 0, "speed_limit_frames": 0}
    seen_ids, seen_cases, seen_sources = set(), set(), set()
    last_frame = None
    for index, item in enumerate(frames):
        if not isinstance(item, dict):
            failures.append("snapshot[{0}] is not an object".format(index))
            continue
        frame_id = item.get("frame_id")
        if type(frame_id) is not int or frame_id < 0:
            failures.append("snapshot[{0}] invalid frame_id".format(index))
            continue
        if last_frame is not None and frame_id <= last_frame:
            failures.append("frame order is not strictly increasing at {0}".format(frame_id))
        last_frame = frame_id
        seen_ids.add(frame_id)
        actual_scene = item.get("scene_id")
        if actual_scene != scene_id:
            failures.append("frame {0}: expected scene {1}, got {2}".format(
                frame_id, scene_id, actual_scene))
        seen_cases.add((item.get("case_id"), item.get("case_name")))
        if not item.get("case_name"):
            failures.append("frame {0}: case name missing".format(frame_id))
        ego = _dict(item.get("ego"))
        status = _dict(item.get("source_status"))
        gps = _dict(status.get("gps"))
        targets_status = _dict(status.get("targets"))
        lane = _dict(item.get("lane"))
        targets = item.get("targets")
        source = item.get("target_source", "none")
        seen_sources.add(source)
        if (item.get("valid") is True and ego.get("valid") is True
                and ego.get("frame_id") == frame_id
                and all(_finite(ego.get(k)) for k in
                        ("x", "y", "heading", "speed", "vx", "vy", "acceleration"))
                and gps.get("usable") is True):
            counts["gps_ok"] += 1
        if (item.get("targets_valid") is True and isinstance(targets, list)
                and targets_status.get("usable") is True
                and source.startswith("sensor:")):
            counts["targets_ok"] += 1
        if isinstance(targets, list) and targets:
            counts["target_frames_with_objects"] += 1
            if any(not isinstance(t, dict) or t.get("valid") is not True or
                   not all(_finite(t.get(k)) for k in ("x", "y", "distance"))
                   for t in targets):
                failures.append("frame {0}: malformed target".format(frame_id))
        if (lane.get("valid") is True and bool(lane.get("lane_id"))
                and lane.get("lane_width_valid") is True
                and _finite(lane.get("lane_width")) and lane["lane_width"] > 0
                and all(isinstance(lane.get(k), list) and len(lane[k]) >= 2
                        for k in ("center_line", "left_boundary", "right_boundary"))
                and _finite(lane.get("lateral_offset"))
                and _finite(lane.get("heading_error"))):
            counts["lane_ok"] += 1
        configurations = item.get("sensor_configurations")
        if (item.get("sensor_configurations_valid") is True
                and isinstance(configurations, list)
                and any(isinstance(c, dict) and c.get("id") == source.split(":", 1)[1]
                        for c in configurations if source.startswith("sensor:"))):
            counts["sensor_config_ok"] += 1
            counts["target_id_verified"] += 1
        elif (source.startswith("sensor:") and targets_status.get("id_verified") is True
              and source.split(":", 1)[1] in targets_status.get("callback_sensor_ids", [])
              and targets_status.get("sensor_read_ok") is True):
            # The actual official callback is ID evidence, not a claim that the
            # scenario's static configuration query succeeded.
            counts["target_id_verified"] += 1
        if _dict(item.get("traffic")).get("valid") is True:
            counts["traffic_frames"] += 1
        if item.get("traffic_signs_valid") is True and item.get("traffic_signs"):
            counts["sign_frames"] += 1
        if item.get("route_valid") is True and item.get("route_points"):
            counts["route_frames"] += 1
        for key, minimum_points, counter in (("parking_spaces", 4, "parking_frames"),
                ("map_stop_lines", 2, "stop_line_frames"),
                ("map_crosswalks", 3, "crosswalk_frames")):
            records = item.get(key)
            if item.get(key + "_valid") is True and isinstance(records, list) and any(
                    isinstance(record, dict) and record.get("valid") is True
                    and record.get("source") == "hdmap"
                    and isinstance(record.get("boundary_knots"), list)
                    and len(record["boundary_knots"]) >= minimum_points
                    and all(isinstance(point, (list, tuple)) and len(point) == 3
                            and all(_finite(value) for value in point)
                            for point in record["boundary_knots"])
                    and (key == "parking_spaces" or lane.get("lane_id") in record.get("lane_ids", []))
                    for record in records):
                counts[counter] += 1
        speed_observations = item.get("speed_limit_observations", [])
        if isinstance(speed_observations, list) and any(isinstance(value, dict)
                and value.get("source") == "hdmap" and value.get("applicable") is True
                and _finite(value.get("speed_mps")) and value["speed_mps"] > 0
                for value in speed_observations):
            counts["speed_limit_frames"] += 1
    total = len(frames)
    if total < minimum:
        failures.append("only {0}/{1} distinct frames captured".format(total, minimum))
    if counts["gps_ok"] != total:
        failures.append("GPS usable in {0}/{1} frames".format(counts["gps_ok"], total))
    if target_required and counts["targets_ok"] != total:
        failures.append("configured sensor targets usable in {0}/{1} frames".format(
            counts["targets_ok"], total))
    if capability != "parking" and counts["lane_ok"] != total:
        failures.append("HD-map lane usable in {0}/{1} frames".format(counts["lane_ok"], total))
    if target_required and total and counts["target_id_verified"] == 0:
        failures.append("target sensor ID was not confirmed by configurations or official callbacks")
    if target_required and counts["target_id_verified"] and not counts["sensor_config_ok"]:
        warnings.append("sensor ID verified by official callback; configuration query remains unavailable")
    if len(seen_cases) > 1:
        failures.append("case identity changed during capture")
    required_observation = {"parking": ("parking_frames", "parking-space geometry"),
                            "speed_sign": ("speed_limit_frames", "applicable numeric speed limit"),
                            "stop_line": ("stop_line_frames", "lane-associated stop-line geometry")}
    if capability in required_observation:
        counter, description = required_observation[capability]
        if counts[counter] == 0:
            failures.append(description + " not observed through a valid API data source")
    if capability == "continuous":
        warnings.append("mixed scenario capabilities require event-by-event review")
    if capability == "traffic_light" and counts["traffic_frames"] == 0:
        warnings.append("no applicable signal/stop-line pair observed")
    if capability == "continuous" and counts["route_frames"] == 0:
        warnings.append("task waypoints not observed")
    if target_required and not counts["target_frames_with_objects"]:
        warnings.append("no dynamic target event observed")
    if "ground_truth" in seen_sources:
        warnings.append("ground-truth fallback observed; sensor-range compliance unverified")
    if total and not target_required and counts["targets_ok"] != total:
        warnings.append("target sensor optional for this scene; unavailable frames do not block acceptance")
    result_status = ("NO_DATA" if not total else
                     "STRUCTURAL_PASS" if total >= minimum and not failures else
                     "FAIL_OR_INCOMPLETE")
    return {"scene_id": scene_id, "scene_name": name, "capability": capability,
            "status": result_status,
            "event_review": "PENDING", "frames": total, "unique_frames": len(seen_ids),
            "case_identity": [list(x) for x in sorted(seen_cases, key=str)],
            "target_sources": sorted(seen_sources), "counts": counts,
            "failures": failures, "warnings": warnings}


def collect(snapshot_path, count, timeout_sec):
    frames, last_frame = [], None
    started_at = time.time()
    deadline = time.monotonic() + timeout_sec
    while len(frames) < count and time.monotonic() < deadline:
        try:
            if os.path.getmtime(snapshot_path) < started_at:
                time.sleep(0.05)
                continue  # A previous run's JSON is not a live data frame.
            # Atomic publisher replacement means each read sees a whole JSON file.
            with open(snapshot_path, "r", encoding="utf-8") as stream:
                item = json.load(stream)
            frame_id = item.get("frame_id") if isinstance(item, dict) else None
            if type(frame_id) is int and frame_id != last_frame:
                frames.append(item)
                last_frame = frame_id
        except (IOError, OSError, ValueError):
            pass
        time.sleep(0.05)
    return frames


def main(argv=None):
    parser = argparse.ArgumentParser(description="Read-only NEVC scene-data acceptance")
    parser.add_argument("--scene", type=int, required=True, choices=sorted(SCENES))
    parser.add_argument("--snapshot", default=DEFAULT_SNAPSHOT)
    parser.add_argument("--replay", help="existing JSONL capture instead of live latest_perception.json")
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--timeout-sec", type=float, default=30.0)
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    if args.frames < 1 or args.timeout_sec <= 0:
        parser.error("--frames and --timeout-sec must be positive")
    if args.replay:
        with open(args.replay, "r", encoding="utf-8") as stream:
            frames = [json.loads(line) for line in stream if line.strip()]
    else:
        frames = collect(args.snapshot, args.frames, args.timeout_sec)
    report = analyze_frames(frames, args.scene, args.frames)
    if not os.path.isdir(args.output_dir):
        os.makedirs(args.output_dir)
    stem = "scene_{0:02d}".format(args.scene)
    with open(os.path.join(args.output_dir, stem + "_frames.jsonl"), "w", encoding="utf-8") as stream:
        for frame in frames:
            stream.write(json.dumps(frame, ensure_ascii=False, allow_nan=False) + "\n")
    with open(os.path.join(args.output_dir, stem + "_report.json"), "w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, sort_keys=True)
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if report["status"] == "STRUCTURAL_PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
