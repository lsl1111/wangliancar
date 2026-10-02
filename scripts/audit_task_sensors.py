"""Read saved platform task configuration without publishing or changing it.

Only explicitly selected diagnostic fields leave the raw configuration, which
can contain credentials. This tool does not replace live Sensor API evidence.
"""

from __future__ import print_function

import argparse
import glob
import json
import os
import re


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _text(value):
    return value[:512] if isinstance(value, str) else None


def _summaries(value, path=""):
    records = []
    if isinstance(value, dict):
        if "generalSensors" in value:
            sensors = value["generalSensors"]
            custom = value.get("customization", {})
            custom = custom if isinstance(custom, dict) else {}
            records.append({
                "json_path": path or "/", "main_vehicle_id": value.get("mainVehicleId")
                    if type(value.get("mainVehicleId")) in (int, str) else None,
                # customization describes the controller, not the vehicle preset.
                "controller_name": _text(custom.get("name")),
                "start_script": _text(custom.get("startScriptPath")),
                "end_script": _text(custom.get("endScriptPath")),
                "sensor_list_valid": isinstance(sensors, list),
                "sensor_count": len(sensors) if isinstance(sensors, list) else None,
                "sensors": [{"id": _text(sensor.get("sensorId", sensor.get("id"))),
                             "type": sensor.get("sensorType", sensor.get("type"))
                             if type(sensor.get("sensorType", sensor.get("type")))
                             in (int, str) else None}
                            for sensor in sensors if isinstance(sensor, dict)]
                            if isinstance(sensors, list) else []})
        for key, child in value.items():
            records.extend(_summaries(child, path + "/" + key))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            records.extend(_summaries(child, path + "/" + str(index)))
    return records


def inspect_task(platform_root, task_id):
    if not isinstance(task_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,128}", task_id):
        raise ValueError("invalid task ID")
    files, errors = [], []
    for relative in ("Common/Foundation/configJsonFromWeb",
                     "Module/VehicleDynamic/configJsonFromWeb"):
        directory = os.path.join(platform_root, relative)
        for filename in sorted(glob.glob(os.path.join(directory, task_id + "_*WorkJsonFromWeb.txt"))):
            try:
                with open(filename, "rb") as stream:
                    raw = stream.read()
                try:
                    content = raw.decode("utf-8-sig")
                except UnicodeDecodeError:
                    content = raw.decode("gb18030")
                files.append({"file": os.path.abspath(filename),
                              "records": _summaries(json.loads(content))})
            except (OSError, ValueError, UnicodeError) as exc:
                errors.append({"file": os.path.abspath(filename), "error": type(exc).__name__})
    records = [record for item in files for record in item["records"]]
    counts = [record["sensor_count"] for record in records if record["sensor_list_valid"]]
    return {"task_id": task_id, "evidence": "saved_platform_task_config",
            "live_api_verified": False, "files": files, "errors": errors,
            "status": ("sensor_configuration_present" if any(counts) else
                       "empty_sensor_configuration" if counts else "unknown"),
            "note": "Saved configuration does not prove live publication or API freshness."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--platform-root", required=True)
    parser.add_argument("--task-id")
    parser.add_argument("--snapshot", default=os.path.join(PROJECT_DIR, "runtime_data", "latest_perception.json"))
    parser.add_argument("--output")
    args = parser.parse_args()
    task_id = args.task_id
    if task_id is None:
        with open(args.snapshot, encoding="utf-8") as stream:
            task_id = json.load(stream).get("task_id")
    result = inspect_task(args.platform_root, task_id)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2)
    print(json.dumps(result, ensure_ascii=True, indent=2))


if __name__ == "__main__":
    main()
