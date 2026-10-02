import json
import os
import tempfile
import unittest

from core.interfaces import Perception
from runtime import target_input_summary
from scripts.audit_task_sensors import inspect_task


class TaskSensorAuditTests(unittest.TestCase):
    def test_empty_actual_instance_and_script_are_reported_without_credentials(self):
        with tempfile.TemporaryDirectory() as directory:
            saved = os.path.join(directory, "Common", "Foundation", "configJsonFromWeb")
            os.makedirs(saved)
            filename = os.path.join(saved, "task1_CybertronBridgeIO_WorkJsonFromWeb.txt")
            data = {"data": [{"mainVehicleId": 0, "generalSensors": [],
                     "customization": {"name": "NEVC_car", "startScriptPath": "captain.bat",
                                       "password": "DO_NOT_EXPORT"},
                     "connection": {"token": "DO_NOT_EXPORT"}}]}
            with open(filename, "w", encoding="utf-8") as stream:
                json.dump(data, stream)
            report = inspect_task(directory, "task1")
            self.assertEqual("empty_sensor_configuration", report["status"])
            self.assertEqual("captain.bat", report["files"][0]["records"][0]["start_script"])
            self.assertEqual("NEVC_car", report["files"][0]["records"][0]["controller_name"])
            self.assertNotIn("main_vehicle_name", report["files"][0]["records"][0])
            self.assertNotIn("DO_NOT_EXPORT", json.dumps(report))
            self.assertFalse(report["live_api_verified"])

    def test_present_legacy_encoded_config_is_not_claimed_to_be_live_api_data(self):
        with tempfile.TemporaryDirectory() as directory:
            saved = os.path.join(directory, "Module", "VehicleDynamic", "configJsonFromWeb")
            os.makedirs(saved)
            filename = os.path.join(saved, "task1_Driver_WorkJsonFromWeb.txt")
            data = {"data": {"generalSensors": [{"sensorId": "camera2", "sensorType": 1,
                                                "password": "DO_NOT_EXPORT"}],
                             "customization": {"name": "自动驾驶"}}}
            with open(filename, "wb") as stream:
                stream.write(json.dumps(data, ensure_ascii=False).encode("gb18030"))
            report = inspect_task(directory, "task1")
            self.assertEqual("sensor_configuration_present", report["status"])
            record = report["files"][0]["records"][0]
            self.assertEqual("自动驾驶", record["controller_name"])
            self.assertEqual([{"id": "camera2", "type": 1}], record["sensors"])
            self.assertNotIn("DO_NOT_EXPORT", json.dumps(report))
            self.assertFalse(report["live_api_verified"])

    def test_missing_or_malformed_data_remains_unknown(self):
        with tempfile.TemporaryDirectory() as directory:
            self.assertEqual("unknown", inspect_task(directory, "task1")["status"])
            saved = os.path.join(directory, "Common", "Foundation", "configJsonFromWeb")
            os.makedirs(saved)
            with open(os.path.join(saved, "task1_Driver_WorkJsonFromWeb.txt"), "w") as stream:
                stream.write("invalid json")
            report = inspect_task(directory, "task1")
            self.assertEqual("unknown", report["status"])
            self.assertEqual("JSONDecodeError", report["errors"][0]["error"])

    def test_task_id_cannot_select_other_paths(self):
        for value in ("../other", "task*", "task/other", "task\n", "", None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                inspect_task("unused", value)

    def test_formal_empty_objects_and_diagnostic_truth_are_distinct(self):
        p = Perception()
        p.scene_id = 15
        p.targets_valid = True
        p.target_source = "sensor:camera2"
        p.source_status = {"targets": {"usable": True, "sensor_read_ok": True}}
        self.assertEqual("valid_empty", target_input_summary(p)["state"])
        p.targets = [object()]
        self.assertEqual("valid_objects", target_input_summary(p)["state"])
        p.target_source = "ground_truth"
        report = target_input_summary(p)
        self.assertEqual("unavailable", report["state"])
        self.assertTrue(report["diagnostic_only"])
        self.assertTrue(report["required"])
        p.target_source = "sensor:camera2"
        p.source_status["targets"]["sensor_read_ok"] = False
        self.assertEqual("unavailable", target_input_summary(p)["state"])
        p.scene_id = 6
        self.assertFalse(target_input_summary(p)["required"])
