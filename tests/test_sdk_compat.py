"""Known polling ABI regression checks without vendor DLLs or a running case."""

import ctypes
import struct
import unittest
from types import SimpleNamespace

from simone_platform.sdk_compat import polling_structs


def make_type(name, base, fields):
    return type(name, (base,), {"_pack_": 1, "_fields_": fields})


def legacy_sdk():
    header = make_type("Header", ctypes.Structure,
                       [("timestamp", ctypes.c_int64), ("frame", ctypes.c_int32),
                        ("version", ctypes.c_int32)])
    vector = make_type("Vec3", ctypes.Structure,
                       [("x", ctypes.c_float), ("y", ctypes.c_float), ("z", ctypes.c_float)])
    point = make_type("Waypoint", ctypes.Structure,
                      [("index", ctypes.c_int32), ("posX", ctypes.c_float),
                       ("posY", ctypes.c_float), ("heading", ctypes.c_float * 4)])
    waypoints = make_type("Waypoints", header,
                          [("wayPointsSize", ctypes.c_int32), ("wayPoints", point * 100)])
    detection = make_type("Detection", ctypes.Structure,
                          [("id", ctypes.c_int32), ("type", ctypes.c_int32),
                           ("posX", ctypes.c_float), ("remaining", ctypes.c_byte * 116)])
    detections = make_type("Detections", header,
                           [("objectSize", ctypes.c_int32), ("objects", detection * 256)])
    obstacle = make_type("Obstacle", ctypes.Structure,
                         [("id", ctypes.c_int32), ("viewId", ctypes.c_int32),
                          ("type", ctypes.c_int32), ("theta", ctypes.c_float),
                          ("posX", ctypes.c_float), ("remaining", ctypes.c_byte * 56)])
    obstacles = make_type("Obstacles", header,
                          [("obstacleSize", ctypes.c_int32), ("obstacle", obstacle * 255)])
    traffic_light = make_type("TrafficLight", ctypes.Structure,
                             [("index", ctypes.c_int32), ("opendriveLightId", ctypes.c_int32),
                              ("countDown", ctypes.c_int32), ("status", ctypes.c_int32)])
    return SimpleNamespace(SimOne_Data=header, SimOneData_Vec3f=vector,
                           SimOne_Data_WayPoints=waypoints,
                           SimOne_Data_SensorDetections_Entry=detection,
                           SimOne_Data_SensorDetections=detections,
                           SimOne_Data_Obstacle_Entry=obstacle,
                           SimOne_Data_Obstacle=obstacles,
                           SimOne_Data_TrafficLight=traffic_light,
                           ESimOne_TrafficLight_Status=ctypes.c_int32)


class SDKCompatTests(unittest.TestCase):
    def test_known_layouts_match_cpp_sizes_without_modifying_vendor_classes(self):
        original = legacy_sdk()
        view, repairs = polling_structs(original, b"3.0.0001")
        for name, expected in (("SimOne_Data_WayPoints", 2884),
                               ("SimOne_Data_SensorDetections_Entry", 228),
                               ("SimOne_Data_SensorDetections", 58388),
                               ("SimOne_Data_Obstacle_Entry", 644),
                               ("SimOne_Data_Obstacle", 164240),
                               ("SimOne_Data_TrafficLight", 13)):
            self.assertEqual(expected, ctypes.sizeof(getattr(view, name)))
            self.assertIn(name, repairs)
        self.assertEqual(2820, ctypes.sizeof(original.SimOne_Data_WayPoints))
        self.assertEqual(128, ctypes.sizeof(original.SimOne_Data_SensorDetections_Entry))
        self.assertEqual(76, ctypes.sizeof(original.SimOne_Data_Obstacle_Entry))
        self.assertEqual(16, ctypes.sizeof(original.SimOne_Data_TrafficLight))
        self.assertIs(original.SimOne_Data, view.SimOne_Data)

    def test_traffic_light_bool_id_countdown_and_status_offsets(self):
        sdk, _ = polling_structs(legacy_sdk(), "3.0.0001")
        kind = sdk.SimOne_Data_TrafficLight
        self.assertEqual(1, kind.opendriveLightId.offset)
        self.assertEqual(5, kind.countDown.offset)
        self.assertEqual(9, kind.status.offset)
        value = kind.from_buffer_copy(struct.pack("<?iii", True, 42, 10, 1))
        self.assertTrue(value.isMainVehicleNextTrafficLight)
        self.assertEqual((42, 10, 1), (value.opendriveLightId, value.countDown, value.status))

    def test_waypoint_id_count_and_second_point_have_correct_offsets(self):
        sdk, _ = polling_structs(legacy_sdk(), "3.0.0001")
        kind = sdk.SimOne_Data_WayPoints
        self.assertEqual(16, kind.mainVehicleId.offset)
        self.assertEqual(80, kind.wayPointsSize.offset)
        self.assertEqual(84, kind.wayPoints.offset)
        raw = bytearray(ctypes.sizeof(kind))
        raw[16:18] = b"0\0"
        struct.pack_into("<i", raw, 80, 2)
        struct.pack_into("<iff", raw, 84, 7, 12.0, 3.0)
        struct.pack_into("<iff", raw, 112, 8, 24.0, 4.0)
        value = kind.from_buffer_copy(raw)
        self.assertEqual(b"0", value.mainVehicleId)
        self.assertEqual(2, value.wayPointsSize)
        self.assertEqual(8, value.wayPoints[1].index)
        self.assertEqual(24.0, value.wayPoints[1].posX)

    def test_detection_corner_tail_and_array_stride_are_correct(self):
        sdk, _ = polling_structs(legacy_sdk(), "3.0.0001")
        kind = sdk.SimOne_Data_SensorDetections
        entry = sdk.SimOne_Data_SensorDetections_Entry
        self.assertEqual(128, entry.cornerPointSize.offset)
        self.assertEqual(132, entry.cornerPoints.offset)
        raw = bytearray(ctypes.sizeof(kind))
        struct.pack_into("<i", raw, 16, 2)
        struct.pack_into("<iif", raw, 20, 10, 6, 15.0)
        struct.pack_into("<i", raw, 20 + 128, 1)
        struct.pack_into("<fff", raw, 20 + 132, 1.0, 2.0, 3.0)
        struct.pack_into("<iif", raw, 20 + 228, 11, 6, 28.0)
        value = kind.from_buffer_copy(raw)
        self.assertEqual(1, value.objects[0].cornerPointSize)
        self.assertEqual(2.0, value.objects[0].cornerPoints[0].y)
        self.assertEqual(11, value.objects[1].id)
        self.assertEqual(28.0, value.objects[1].posX)

    def test_ground_truth_prediction_and_array_stride_are_correct(self):
        sdk, _ = polling_structs(legacy_sdk(), "3.0.0001")
        kind, entry = sdk.SimOne_Data_Obstacle, sdk.SimOne_Data_Obstacle_Entry
        self.assertEqual(76, entry.prediction.offset)
        self.assertEqual(568, ctypes.sizeof(sdk.Prediction))
        raw = bytearray(ctypes.sizeof(kind))
        struct.pack_into("<i", raw, 16, 2)
        struct.pack_into("<i", raw, 20, 101)
        struct.pack_into("<If", raw, 20 + 76, 1, 0.1)
        struct.pack_into("<fff", raw, 20 + 84, 5.0, 6.0, 7.0)
        struct.pack_into("<i", raw, 20 + 644, 102)
        struct.pack_into("<f", raw, 20 + 644 + 16, 42.0)
        value = kind.from_buffer_copy(raw)
        self.assertEqual(1, value.obstacle[0].prediction.trajectorySize)
        self.assertEqual(6.0, value.obstacle[0].prediction.trajectory[0].y)
        self.assertEqual(102, value.obstacle[1].id)
        self.assertEqual(42.0, value.obstacle[1].posX)

    def test_unknown_version_and_unknown_layout_fail_before_io(self):
        with self.assertRaises(RuntimeError):
            polling_structs(legacy_sdk(), "9.0.0")
        sdk = legacy_sdk()
        sdk.SimOne_Data_WayPoints = make_type("Bad", ctypes.Structure,
                                             [("bytes", ctypes.c_byte * 2821)])
        with self.assertRaises(RuntimeError):
            polling_structs(sdk, "3.0.0001")

    def test_partial_entry_array_mix_is_rejected(self):
        sdk = legacy_sdk()
        sdk.SimOne_Data_SensorDetections_Entry = make_type(
            "Partial", sdk.SimOne_Data_SensorDetections_Entry,
            [("extra", ctypes.c_byte * 100)])
        with self.assertRaises(RuntimeError):
            polling_structs(sdk, "3.0.0001")

    def test_already_current_structs_pass_through(self):
        view, _ = polling_structs(legacy_sdk(), "3.0.0001")
        again, repairs = polling_structs(view, "3.0.0001")
        self.assertIs(view, again)
        self.assertEqual([], repairs)


if __name__ == "__main__":
    unittest.main()
