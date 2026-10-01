"""The only module in this project allowed to call the official SimOne API."""

import importlib
import copy
import ctypes
import math
import os
import sys
import time

from core.interfaces import ControlOut
from simone_platform.sdk_compat import polling_structs


def _decode_sdk_text(value):
    if isinstance(value, bytes):
        raw = value.split(b"\0", 1)[0]
        for encoding in ("utf-8", "gb18030"):
            try:
                return raw.decode(encoding)
            except UnicodeDecodeError:
                pass
        return raw.decode("utf-8", errors="replace")
    return str(value)


def _sdk_int(value):
    """ctypes enum fields need .value on the bundled Python 3.6 SDK."""
    return int(getattr(value, "value", value))


def _map_string(value):
    return value.GetString() if hasattr(value, "GetString") else _decode_sdk_text(value)


def _native_dict(value):
    """Detach small SDK structs, including inherited headers and enum values.

    Fixed lane arrays have no count in this SDK: retain them without inventing
    a valid-point count. Never use this helper for image/point-cloud buffers.
    """
    if isinstance(value, bytes):
        return _decode_sdk_text(value)
    if isinstance(value, ctypes.Array) or isinstance(value, (tuple, list)):
        return [_native_dict(item) for item in value]
    if isinstance(value, ctypes.Structure):
        fields = []
        for cls in reversed(type(value).__mro__):
            fields.extend(cls.__dict__.get("_fields_", []))
        return {field[0]: _native_dict(getattr(value, field[0])) for field in fields}
    if hasattr(value, "value"):
        return _native_dict(value.value)
    if hasattr(value, "__dict__"):
        return {key: _native_dict(item) for key, item in vars(value).items()}
    return value


class SimOneAdapter(object):
    CASE_STOP = 1
    CASE_RUNNING = 2
    CASE_PAUSE = 3

    def __init__(self, config, logger):
        self.config = config
        self.logger = logger
        self.structs = None
        self.sensor_api = None
        self.pnc_api = None
        self.service_api = None
        self.hdmap = None
        self.connected = False
        self.map_loaded = False
        self.last_send_result = {}
        self._source_status = {}
        self._reference_snapshot = {"route_points": [], "route_valid": False,
                                    "route_waypoints": [],
                                    "sensor_configurations": [],
                                    "sensor_configurations_valid": False,
                                    "environment": {}, "environment_valid": False}
        self._reference_update = 0.0
        self._traffic_candidates = {}
        self._last_frame_times = {}

    def bootstrap(self):
        sdk_dir = os.path.abspath(self.config.sdk_dir)
        required = ("SimOneIOStruct.py", "SimOneServiceAPI.py", "HDMapAPI.pyd")
        missing = [name for name in required if not os.path.exists(os.path.join(sdk_dir, name))]
        if missing:
            raise RuntimeError("SimOne SDK 文件缺失: {0}".format(", ".join(missing)))
        if sdk_dir not in sys.path:
            sys.path.insert(0, sdk_dir)
        # The SDK's native DLL loader expects this directory to be current.
        os.chdir(sdk_dir)
        self.structs = importlib.import_module("SimOneIOStruct")
        self.sensor_api = importlib.import_module("SimOneSensorAPI")
        self.pnc_api = importlib.import_module("SimOnePNCAPI")
        self.service_api = importlib.import_module("SimOneServiceAPI")
        self.hdmap = importlib.import_module("HDMapAPI")
        version = self.service_api.SoAPIGetVersion()
        self.structs, repairs = polling_structs(self.structs, version)
        if repairs:
            self.logger.warning("已修正 SDK %s 轮询结构布局: %s",
                                _decode_sdk_text(version), ", ".join(repairs))
        self.logger.info("SimOne SDK 已加载: %s", sdk_dir)
        return True

    def initialize(self):
        if self.service_api is None:
            self.bootstrap()
        deadline = time.time() + self.config.connect_timeout_sec
        while time.time() < deadline:
            result = self.service_api.SoInitSimOneAPI(
                self.config.vehicle_id, 0, self.config.server_ip
            )
            if result:
                self.connected = True
                self.pnc_api.SoSetDriverName(self.config.vehicle_id, "Captain")
                self.logger.info(
                    "已连接 SimOne: vehicle=%s server=%s",
                    self.config.vehicle_id,
                    self.config.server_ip,
                )
                return True
            time.sleep(1.0)
        raise RuntimeError("连接 SimOne 超时，请确认案例和 BridgeIO 已启动")

    def load_hdmap(self):
        if self.hdmap is None:
            raise RuntimeError("SDK 尚未加载")
        self.map_loaded = bool(self.hdmap.loadHDMap(int(self.config.map_timeout_sec)))
        self._traffic_candidates = {}
        if not self.map_loaded:
            self.logger.warning("高精地图加载失败；GPS/目标数据仍可继续输出")
        else:
            self.logger.info("高精地图加载成功")
        return self.map_loaded

    def get_case_status(self):
        return int(self.service_api.SoGetCaseRunStatus())

    def get_case_info(self):
        result = {"case_name": "", "case_id": "", "task_id": ""}
        data = self.structs.SimOne_Data_CaseInfo()
        if self.service_api.SoAPIGetCaseInfo(data):
            result["case_name"] = _decode_sdk_text(data.caseName)
            result["case_id"] = _decode_sdk_text(data.caseId)
            result["task_id"] = _decode_sdk_text(data.taskId)
        return result

    def read_raw_snapshot(self):
        self._source_status = {}
        try:
            result = self._read_raw_snapshot()
        except Exception as exc:
            result = {"gps": None, "targets": [], "targets_valid": False,
                      "errors": ["SNAPSHOT_UNAVAILABLE:" + type(exc).__name__]}
        if result.get("gps") is None:
            result.update(self._read_reference_data())
            result.update(self._read_auxiliary_data(result.get("sensor_configurations", [])))
        result["source_status"] = copy.deepcopy(self._source_status)
        return result

    def _read_raw_snapshot(self):
        gps = self.structs.SimOne_Data_Gps()
        if not self.sensor_api.SoGetGps(self.config.vehicle_id, gps):
            return {"gps": None, "targets": [], "targets_valid": False,
                    "target_source": "none", "errors": ["GPS_UNAVAILABLE"]}
        gps_data = {
            "frame": int(gps.frame),
            "timestamp": int(gps.timestamp),
            "x": float(gps.posX),
            "y": float(gps.posY),
            "z": float(gps.posZ),
            "heading": float(gps.oriZ),
            "roll": float(gps.oriX),
            "pitch": float(gps.oriY),
            "vx": float(gps.velX),
            "vy": float(gps.velY),
            "vz": float(gps.velZ),
            "ax": float(gps.accelX),
            "ay": float(gps.accelY),
            "az": float(gps.accelZ),
            "yaw_rate": float(gps.angVelZ),
            "wheel_speeds": [float(gps.wheelSpeedFL), float(gps.wheelSpeedFR),
                             float(gps.wheelSpeedRL), float(gps.wheelSpeedRR)],
            "odometer": float(gps.odometer),
            "throttle": float(gps.throttle),
            "brake": float(gps.brake),
            "steering": float(gps.steering),
            "gear": int(gps.gear),
            "is_lost": bool(gps.isGPSLost),
        }
        gps_data["age_ms"] = self._frame_age_ms("gps", gps_data["frame"])
        gps_data["sdk_data"] = _native_dict(gps)
        count = int(getattr(gps, "extraStateSize", 0))
        extra = getattr(gps, "extraStates", [])
        if count < 0 or count > len(extra):
            raise ValueError("GPS extra state count out of range")
        gps_data["extra_states"] = [float(v) for v in extra[:count]]
        if "extraStates" in gps_data["sdk_data"]:
            gps_data["sdk_data"]["extraStates"] = gps_data["extra_states"]
        for name, sdk_name in (("roll_rate", "angVelX"), ("pitch_rate", "angVelY"),
                               ("engine_rpm", "engineRpm")):
            gps_data[name] = float(getattr(gps, sdk_name, 0.0))
        self._metadata("gps", gps, True)
        errors = []
        reference = self._read_reference_data()
        configurations = reference["sensor_configurations"]
        preferred_id = self.config.sensor_id
        if reference["sensor_configurations_valid"]:
            ids = [item.get("id", "") for item in configurations if isinstance(item, dict)]
            if preferred_id in ids:
                target_sensor_id = preferred_id
            else:
                candidates = [item.get("id", "") for item in configurations
                              if isinstance(item, dict) and item.get("id") and
                              any(word in item.get("type", "").lower() for word in
                                  ("camera", "lidar", "fusion", "perfect"))]
                target_sensor_id = candidates[0] if candidates else ""
            sensor_presence = "configured" if target_sensor_id else "not_configured"
        else:
            target_sensor_id = preferred_id
            sensor_presence = "unknown"
        target_frame, target_timestamp = -1, 0
        targets, source = None, "none"
        if target_sensor_id:
            try:
                targets, source, target_frame, target_timestamp = self._read_sensor_targets(
                    target_sensor_id)
            except Exception as exc:
                errors.append("SENSOR_TARGETS_INVALID:{0}".format(type(exc).__name__))
        if targets is None:
            try:
                targets, target_frame, target_timestamp = self._read_ground_truth_targets()
            except Exception as exc:
                errors.append("GROUND_TRUTH_INVALID:{0}".format(type(exc).__name__))
                targets = None
            source = "ground_truth" if targets is not None else "none"
        result = {"gps": gps_data, "targets": targets or [],
                  "targets_valid": targets is not None, "target_source": source,
                  "targets_frame": target_frame if targets is not None else -1,
                  "targets_timestamp": target_timestamp if targets is not None else 0,
                  "targets_age_ms": self._frame_age_ms("targets:" + source, target_frame)
                  if targets is not None else -1,
                  "errors": errors}
        result.update(reference)
        result.update(self._read_auxiliary_data(result["sensor_configurations"]))
        self._source_status["targets"] = {
            "read_ok": targets is not None, "source": source,
            "sensor_presence": sensor_presence,
            "sensor_id": target_sensor_id,
            "sensor_read_ok": source.startswith("sensor:"),
            "frame_id": result["targets_frame"], "timestamp": result["targets_timestamp"],
            "age_ms": result["targets_age_ms"], "clock": "sdk_frame"}
        return result

    def _frame_age_ms(self, source, frame):
        if frame < 0:
            return -1
        now = time.monotonic()
        previous = self._last_frame_times.get(source)
        if previous is not None and frame < previous[0]:
            return -2  # Regressed stream; retain high water mark until recovery.
        if previous is None or previous[0] != frame:
            self._last_frame_times[source] = (frame, now)
            return 0
        return max(0, int((now - previous[1]) * 1000))

    def _metadata(self, source, native, ok):
        frame = int(getattr(native, "frame", -1))
        self._source_status[source] = {
            "read_ok": bool(ok), "frame_id": frame,
            "timestamp": int(getattr(native, "timestamp", 0)),
            "age_ms": self._frame_age_ms(source, frame) if ok else -1,
            "clock": "sdk_frame" if frame >= 0 else "unavailable"}

    def _read_sensor_targets(self, sensor_id):
        data = self.structs.SimOne_Data_SensorDetections()
        ok = self.sensor_api.SoGetSensorDetections(
            self.config.vehicle_id, sensor_id, data
        )
        if not ok:
            return None, "none", -1, 0
        targets = []
        if int(data.objectSize) < 0 or int(data.objectSize) > len(data.objects):
            raise ValueError("sensor object count out of range")
        for index in range(int(data.objectSize)):
            item = data.objects[index]
            targets.append(self._target_dict(item, float(item.probability)))
        return targets, "sensor:{0}".format(sensor_id), int(data.frame), int(data.timestamp)

    def _read_ground_truth_targets(self):
        data = self.structs.SimOne_Data_Obstacle()
        if not self.sensor_api.SoGetGroundTruth(self.config.vehicle_id, data):
            return None, -1, 0
        targets = []
        if int(data.obstacleSize) < 0 or int(data.obstacleSize) > len(data.obstacle):
            raise ValueError("ground truth object count out of range")
        for index in range(int(data.obstacleSize)):
            targets.append(self._target_dict(data.obstacle[index], 1.0))
        return targets, int(data.frame), int(data.timestamp)

    @staticmethod
    def _target_dict(item, probability):
        result = {
            "sdk_data": _native_dict(item),
            "id": int(item.id),
            "type": _sdk_int(item.type),
            "x": float(item.posX),
            "y": float(item.posY),
            "z": float(item.posZ),
            "vx": float(item.velX),
            "vy": float(item.velY),
            "vz": float(item.velZ),
            "heading": float(getattr(item, "oriZ", 0.0)),
            "roll": float(getattr(item, "oriX", 0.0)),
            "pitch": float(getattr(item, "oriY", 0.0)),
            "ax": float(getattr(item, "accelX", 0.0)),
            "ay": float(getattr(item, "accelY", 0.0)),
            "az": float(getattr(item, "accelZ", 0.0)),
            "length": float(item.length),
            "width": float(item.width),
            "height": float(item.height),
            "probability": float(probability),
            "sensor_range": float(getattr(item, "range", -1.0)),
        }
        for name, sdk_name in (("relative_x", "relativePosX"),
                               ("relative_y", "relativePosY"),
                               ("relative_z", "relativePosZ"),
                               ("relative_vx", "relativeVelX"),
                               ("relative_vy", "relativeVelY"),
                               ("relative_vz", "relativeVelZ"),
                               ("relative_roll", "relativeRotX"),
                               ("relative_pitch", "relativeRotY"),
                               ("relative_heading", "relativeRotZ")):
            if hasattr(item, sdk_name):
                result[name] = float(getattr(item, sdk_name))
        if hasattr(item, "bbox2dMinX"):
            result["bbox2d"] = [float(item.bbox2dMinX), float(item.bbox2dMinY),
                                float(item.bbox2dMaxX), float(item.bbox2dMaxY)]
        return result

    def _read_reference_data(self):
        """Scenario metadata changes rarely; avoid polling it at 20 Hz."""
        now = time.monotonic()
        if self._reference_update and now - self._reference_update < 5.0:
            return copy.deepcopy(self._reference_snapshot)
        snapshot = {"route_points": [], "route_waypoints": [], "route_valid": False,
                    "sensor_configurations": [], "sensor_configurations_valid": False,
                    "environment": {},
                    "environment_valid": False, "traffic_signs": [],
                    "traffic_signs_valid": False, "reference_errors": []}
        try:
            if hasattr(self.pnc_api, "SoGetWayPoints"):
                route = self.structs.SimOne_Data_WayPoints()
                if self.pnc_api.SoGetWayPoints(self.config.vehicle_id, route):
                    count = int(route.wayPointsSize)
                    if count < 0 or count > len(route.wayPoints):
                        raise ValueError("waypoint count out of range")
                    route_points = [
                        (float(route.wayPoints[i].posX), float(route.wayPoints[i].posY))
                        for i in range(count)
                    ]
                    route_waypoints = [
                        {"index": int(getattr(route.wayPoints[i], "index", i)),
                         "x": float(route.wayPoints[i].posX),
                         "y": float(route.wayPoints[i].posY),
                         "heading_quaternion": [
                             float(getattr(route.wayPoints[i], field, 0.0))
                             for field in ("heading_x", "heading_y", "heading_z", "heading_w")
                         ]}
                        for i in range(count)
                    ]
                    snapshot["route_points"] = route_points
                    snapshot["route_waypoints"] = route_waypoints
                    snapshot["route_valid"] = bool(count)
        except Exception as exc:
            snapshot["reference_errors"].append("ROUTE_UNAVAILABLE:{0}".format(type(exc).__name__))
        try:
            if hasattr(self.sensor_api, "SoGetSensorConfigurations"):
                configurations = self.structs.SimOne_Data_SensorConfigurations()
                if self.sensor_api.SoGetSensorConfigurations(self.config.vehicle_id, configurations):
                    count = int(configurations.dataSize)
                    if count < 0 or count > len(configurations.data):
                        raise ValueError("sensor configuration count out of range")
                    values = [
                        {"id": _decode_sdk_text(configurations.data[i].sensorId),
                         "type": _decode_sdk_text(configurations.data[i].sensorType),
                         "hz": int(configurations.data[i].hz),
                         "x": float(configurations.data[i].x),
                         "y": float(configurations.data[i].y),
                         "z": float(configurations.data[i].z),
                         "roll": float(getattr(configurations.data[i], "roll", 0.0)),
                         "pitch": float(getattr(configurations.data[i], "pitch", 0.0)),
                         "sdk_data": _native_dict(configurations.data[i]),
                         "yaw": float(configurations.data[i].yaw)}
                        for i in range(count)
                    ]
                    snapshot["sensor_configurations"] = values
                    snapshot["sensor_configurations_valid"] = True
        except Exception as exc:
            snapshot["reference_errors"].append("SENSOR_CONFIG_UNAVAILABLE:{0}".format(type(exc).__name__))
        try:
            if hasattr(self.sensor_api, "SoGetEnvironment"):
                environment = self.structs.SimOne_Data_Environment()
                if self.sensor_api.SoGetEnvironment(environment):
                    snapshot["environment"] = _native_dict(environment)
                    snapshot["environment_valid"] = True
        except Exception as exc:
            snapshot["reference_errors"].append("ENVIRONMENT_UNAVAILABLE:{0}".format(type(exc).__name__))
        try:
            if self.map_loaded and self.hdmap is not None and hasattr(self.hdmap, "getTrafficSignList"):
                signs = self.hdmap.getTrafficSignList()
                count = int(signs.Size())
                if count < 0 or count > 10000:
                    raise ValueError("traffic sign count out of range")
                for i in range(count):
                    sign = signs.GetElement(i)
                    validities = getattr(sign, "validities", None)
                    scopes = []
                    for j in range(validities.Size() if validities else 0):
                        scope = validities.GetElement(j)
                        scopes.append({"road_id": int(scope.roadId),
                                       "section_index": int(scope.sectionIndex),
                                       "from_lane_id": int(scope.fromLaneId),
                                       "to_lane_id": int(scope.toLaneId)})
                    snapshot["traffic_signs"].append({
                        "id": int(sign.id), "type": _map_string(sign.type),
                        "sub_type": _map_string(sign.subType),
                        "value": _map_string(sign.value), "unit": _map_string(sign.unit),
                        "is_dynamic": bool(sign.isDynamic),
                        "heading": float(sign.heading),
                        "x": float(sign.pt.x), "y": float(sign.pt.y),
                        "z": float(sign.pt.z), "validities": scopes})
                snapshot["traffic_signs_valid"] = True
        except Exception as exc:
            snapshot["traffic_signs"] = []
            snapshot["reference_errors"].append("TRAFFIC_SIGNS_UNAVAILABLE:{0}".format(type(exc).__name__))
        self._reference_snapshot = snapshot
        self._reference_update = now
        return copy.deepcopy(snapshot)

    def _read_auxiliary_data(self, configurations):
        result = {"imu": {}, "imu_valid": False, "radar_detections": [],
                  "radar_status": {}, "ultrasonic_detections": [],
                  "ultrasonic_valid": False, "sensor_lane_observations": [],
                  "sensor_lane_status": {},
                  "sensor_errors": []}
        if hasattr(self.sensor_api, "SoGetImu"):
            try:
                imu = self.structs.SimOne_Data_IMU()
                if self.sensor_api.SoGetImu(self.config.vehicle_id, imu):
                    result["imu"] = {
                        "sdk_data": _native_dict(imu),
                        "vx": float(getattr(imu, "velX", 0.0)),
                        "vy": float(getattr(imu, "velY", 0.0)),
                        "vz": float(getattr(imu, "velZ", 0.0)),
                        "roll_rate": float(getattr(imu, "angVelX", 0.0)),
                        "pitch_rate": float(getattr(imu, "angVelY", 0.0)),
                        "ax": float(imu.accelX), "ay": float(imu.accelY),
                        "az": float(imu.accelZ), "yaw_rate": float(imu.angVelZ),
                        "roll": float(imu.rotX), "pitch": float(imu.rotY),
                        "yaw": float(imu.rotZ)
                    }
                    result["imu_valid"] = True
                self._metadata("imu", imu, result["imu_valid"])
            except Exception as exc:
                self._source_status["imu"] = {"read_ok": False, "frame_id": -1,
                                               "timestamp": 0, "age_ms": -1}
                result["sensor_errors"].append("IMU_UNAVAILABLE:{0}".format(type(exc).__name__))
        for sensor in configurations:
            sensor_id = sensor.get("id", "")
            kind = sensor.get("type", "").lower()
            if not sensor_id:
                continue
            if (("radar" in kind and "ultrasonic" not in kind) or kind == "3") and hasattr(self.sensor_api, "SoGetRadarDetections"):
                try:
                    radar = self.structs.SimOne_Data_RadarDetection()
                    radar_ok = self.sensor_api.SoGetRadarDetections(self.config.vehicle_id, sensor_id, radar)
                    self._metadata("radar:" + sensor_id, radar, radar_ok)
                    result["radar_status"][sensor_id] = bool(radar_ok)
                    if radar_ok:
                        count = int(radar.detectNum)
                        if count < 0 or count > len(radar.detections):
                            raise ValueError("radar count out of range")
                        for i in range(count):
                            hit = radar.detections[i]
                            result["radar_detections"].append({
                                "sdk_data": _native_dict(hit),
                                "sensor_id": sensor_id, "frame_id": int(radar.frame),
                                "timestamp": int(radar.timestamp), "id": int(hit.id),
                                "type": _sdk_int(hit.type), "x": float(hit.posX),
                                "y": float(hit.posY), "vx": float(hit.velX),
                                "vy": float(hit.velY), "range": float(hit.range),
                                "range_rate": float(hit.rangeRate),
                                "probability": float(hit.probability)
                            })
                            for name, field in (("sub_id", "subId"), ("z", "posZ"),
                                                ("vz", "velZ"), ("ax", "accelX"),
                                                ("ay", "accelY"), ("az", "accelZ"),
                                                ("roll", "oriX"), ("pitch", "oriY"),
                                                ("heading", "oriZ"), ("length", "length"),
                                                ("width", "width"), ("height", "height"),
                                                ("azimuth", "azimuth"), ("vertical", "vertical"),
                                                ("snr_db", "snrdb"), ("rcs_db", "rcsdb")):
                                if hasattr(hit, field):
                                    result["radar_detections"][-1][name] = float(getattr(hit, field))
                except Exception as exc:
                    self._source_status["radar:" + sensor_id] = {
                        "read_ok": False, "frame_id": -1, "timestamp": 0, "age_ms": -1}
                    result["radar_status"][sensor_id] = False
                    result["radar_detections"] = [
                        hit for hit in result["radar_detections"]
                        if hit["sensor_id"] != sensor_id
                    ]
                    result["sensor_errors"].append("RADAR_UNAVAILABLE:{0}:{1}".format(sensor_id, type(exc).__name__))
            if ("camera" in kind or "fusion" in kind or kind in ("1", "9")) and hasattr(self.sensor_api, "SoGetSensorLaneInfo"):
                try:
                    lane = self.structs.SimOne_Data_LaneInfo()
                    lane_ok = self.sensor_api.SoGetSensorLaneInfo(self.config.vehicle_id, sensor_id, lane)
                    self._metadata("lane:" + sensor_id, lane, lane_ok)
                    result["sensor_lane_status"][sensor_id] = bool(lane_ok)
                    if lane_ok:
                        result["sensor_lane_observations"].append({
                            "sdk_data": _native_dict(lane),
                            "lines": {name: _native_dict(getattr(lane, name))
                                      for name in ("ll_Line", "l_Line", "c_Line", "r_Line", "rr_Line")},
                            "left_lane_id": int(lane.laneLeftID),
                            "right_lane_id": int(lane.laneRightID),
                            "predecessor_ids_raw": _native_dict(lane.lanePredecessorID),
                            "successor_ids_raw": _native_dict(lane.laneSuccessorID),
                            "id_namespace": "sensor_local_not_hdmap",
                            "geometry_frame": "sdk_native_unverified",
                            "fixed_arrays_have_count": False,
                            "sensor_id": sensor_id, "frame_id": int(lane.frame),
                            "timestamp": int(lane.timestamp), "lane_id": int(lane.id),
                            "lane_type": _sdk_int(lane.laneType),
                            "left_line_type": _sdk_int(lane.l_Line.lineType),
                            "right_line_type": _sdk_int(lane.r_Line.lineType),
                            "left_line_color": _sdk_int(lane.l_Line.lineColor),
                            "right_line_color": _sdk_int(lane.r_Line.lineColor)
                        })
                except Exception as exc:
                    self._source_status["lane:" + sensor_id] = {
                        "read_ok": False, "frame_id": -1, "timestamp": 0, "age_ms": -1}
                    result["sensor_lane_status"][sensor_id] = False
                    result["sensor_errors"].append("SENSOR_LANE_UNAVAILABLE:{0}:{1}".format(sensor_id, type(exc).__name__))
        if hasattr(self.sensor_api, "SoGetUltrasonicRadars"):
            try:
                ultrasound = self.structs.SimOne_Data_UltrasonicRadars()
                if self.sensor_api.SoGetUltrasonicRadars(self.config.vehicle_id, ultrasound):
                    result["ultrasonic_valid"] = True
                    count = int(ultrasound.ultrasonicRadarNum)
                    if count < 0 or count > len(ultrasound.ultrasonicRadars):
                        raise ValueError("ultrasonic sensor count out of range")
                    for i in range(count):
                        sensor = ultrasound.ultrasonicRadars[i]
                        self._metadata("ultrasonic:" + _decode_sdk_text(sensor.sensorId), sensor, True)
                        hits = int(sensor.obstacleNum)
                        if hits < 0 or hits > len(sensor.obstacleDetections):
                            raise ValueError("ultrasonic target count out of range")
                        for j in range(hits):
                            hit = sensor.obstacleDetections[j]
                            result["ultrasonic_detections"].append({
                                "sensor_id": _decode_sdk_text(sensor.sensorId),
                                "frame_id": int(getattr(sensor, "frame", -1)),
                                "timestamp": int(getattr(sensor, "timestamp", 0)),
                                "range": float(hit.obstacleRanges),
                                "x": float(hit.x), "y": float(hit.y), "z": float(hit.z)
                            })
                    if not count:
                        self._metadata("ultrasonic:no_sensor", ultrasound, True)
            except Exception as exc:
                result["ultrasonic_valid"] = False
                result["ultrasonic_detections"] = []
                self._source_status["ultrasonic:all"] = {
                    "read_ok": False, "frame_id": -1, "timestamp": 0, "age_ms": -1}
                result["sensor_errors"].append("ULTRASONIC_UNAVAILABLE:{0}".format(type(exc).__name__))
        return result

    def read_traffic(self, lane_id=""):
        """Return traffic-light info for the main vehicle.

        Uses the dynamic per-frame signal truth via SoGetTrafficLights, keyed by
        the static light ids discovered from the HD map (getTrafficLightList).
        Returns a list of dicts, one per currently relevant traffic light:
          {opendrive_id, status, count_down, x, y}
        When no light or map is available it returns [] (perception keeps the
        TrafficControl fields at their UNKNOWN/invalid defaults).
        """
        lights = []
        if (not lane_id or not self.map_loaded or self.hdmap is None or
                not hasattr(self.hdmap, "getTrafficLightList") or
                not hasattr(self.hdmap, "getStoplineList")):
            return lights
        if lane_id not in self._traffic_candidates:
            candidates = []
            traffic_light_list = self.hdmap.getTrafficLightList()
            count = int(traffic_light_list.Size()) if traffic_light_list else 0
            for index in range(count):
                light = traffic_light_list.GetElement(index)
                stoplines = self.hdmap.getStoplineList(light, self.hdmap.pySimString(lane_id))
                if not stoplines:
                    continue
                stop_points = []
                for stop_index in range(stoplines.Size()):
                    stopline = stoplines.GetElement(stop_index)
                    knots = getattr(stopline, "boundaryKnots", None)
                    if knots and knots.Size():
                        sx = sum(float(knots.GetElement(j).x) for j in range(knots.Size())) / knots.Size()
                        sy = sum(float(knots.GetElement(j).y) for j in range(knots.Size())) / knots.Size()
                    else:
                        sx = float(stopline.pt.x)
                        sy = float(stopline.pt.y)
                    stop_points.append((sx, sy))
                if stop_points:
                    pt = getattr(light, "pt", None)
                    candidates.append((int(light.id), getattr(pt, "x", None),
                                       getattr(pt, "y", None), stop_points))
            self._traffic_candidates[lane_id] = candidates
        for opendrive_id, x, y, stop_points in self._traffic_candidates[lane_id]:
            native = self.structs.SimOne_Data_TrafficLight()
            if not self.sensor_api.SoGetTrafficLights(
                self.config.vehicle_id, opendrive_id, native
            ):
                continue
            for sx, sy in stop_points:
                lights.append({
                    "opendrive_id": opendrive_id,
                    "status": _sdk_int(native.status),
                    "count_down": int(native.countDown),
                    "stop_line_x": sx,
                    "stop_line_y": sy,
                    "x": float(x) if isinstance(x, (int, float)) else None,
                    "y": float(y) if isinstance(y, (int, float)) else None,
                })
        return lights

    def get_driver_control(self):
        native = self.structs.SimOne_Data_Control()
        if not self.pnc_api.SoGetDriverControl(self.config.vehicle_id, native):
            return None
        if any(_sdk_int(getattr(native, field)) != 0 for field in
               ("EThrottleMode", "EBrakeMode", "ESteeringMode")) or native.isManualGear:
            return None  # Do not reinterpret angles/torques as percentages.
        result = ControlOut()
        result.throttle = float(native.throttle)
        result.brake = float(native.brake)
        result.steering = float(native.steering)
        result.gear = _sdk_int(native.gear)
        result.handbrake = bool(native.handbrake)
        result.valid = True
        result.source = "SimOneDriver"
        result.frame_id = int(native.frame)
        result.timestamp = int(native.timestamp)
        # A driver snapshot is observational, not a fresh command authorization.
        result.valid_until = 0.0
        try:
            result.clamp()
        except ValueError:
            return None
        return result

    def send_control(self, control):
        receipt = {"mode_ok": False, "drive_ok": False, "signals_ok": False,
                   "ok": False, "error": ""}
        self.last_send_result = receipt
        try:
            if not isinstance(control, ControlOut) or not control.valid:
                raise ValueError("control invalid")
            control.clamp()
            if (type(control.frame_id) is not int or not 0 <= control.frame_id <= 2147483647
                    or type(control.timestamp) is not int or not 0 <= control.timestamp < 2 ** 63
                    or not math.isfinite(control.valid_until)
                    or time.monotonic() >= control.valid_until):
                raise ValueError("control provenance missing or expired")
            native = self.structs.SimOne_Data_Control()
            native.frame, native.timestamp = control.frame_id, control.timestamp
            native.EThrottleMode = native.EBrakeMode = native.ESteeringMode = 0
            native.throttle, native.brake, native.steering = control.throttle, control.brake, control.steering
            native.handbrake = control.handbrake
            native.isManualGear = False
            native.gear = control.gear
            receipt["mode_ok"] = bool(self.pnc_api.SoSetDriveMode(self.config.vehicle_id, 0))
            if not receipt["mode_ok"]:
                raise RuntimeError("drive mode rejected")
            receipt["drive_ok"] = bool(self.pnc_api.SoSetDrive(self.config.vehicle_id, native))
            receipt["signals_ok"] = bool(self._send_signals(control))
            receipt["ok"] = receipt["drive_ok"] and receipt["signals_ok"]
        except Exception as exc:
            receipt["error"] = "{0}:{1}".format(type(exc).__name__, exc)
        return receipt["ok"]

    def _send_signals(self, control):
        lights = self.structs.SimOne_Data_Signal_Lights()
        lights.frame, lights.timestamp = control.frame_id, control.timestamp
        if control.hazard_signal:
            lights.signalLights = 4
        else:
            lights.signalLights = (2 if control.left_signal else 0) | (
                1 if control.right_signal else 0
            )
        return self.pnc_api.SoSetSignalLights(self.config.vehicle_id, lights)

    def shutdown(self):
        if self.connected and self.service_api is not None:
            try:
                self.service_api.SoTerminateSimOneAPI()
            finally:
                self.connected = False

    @staticmethod
    def gps_speed(gps):
        return math.sqrt(gps["vx"] * gps["vx"] + gps["vy"] * gps["vy"])
