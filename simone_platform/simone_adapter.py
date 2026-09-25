"""The only module in this project allowed to call the official SimOne API."""

import importlib
import math
import os
import sys
import time

from core.interfaces import ControlOut


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
        gps = self.structs.SimOne_Data_Gps()
        if not self.sensor_api.SoGetGps(self.config.vehicle_id, gps):
            return {"gps": None, "targets": [], "target_source": "none"}
        gps_data = {
            "frame": int(gps.frame),
            "timestamp": int(gps.timestamp),
            "x": float(gps.posX),
            "y": float(gps.posY),
            "z": float(gps.posZ),
            "heading": float(gps.oriZ),
            "vx": float(gps.velX),
            "vy": float(gps.velY),
            "vz": float(gps.velZ),
            "ax": float(gps.accelX),
            "ay": float(gps.accelY),
            "throttle": float(gps.throttle),
            "brake": float(gps.brake),
            "steering": float(gps.steering),
            "gear": int(gps.gear),
            "is_lost": bool(gps.isGPSLost),
        }
        targets, source = self._read_sensor_targets()
        if targets is None:
            targets = self._read_ground_truth_targets()
            source = "ground_truth"
        return {"gps": gps_data, "targets": targets, "target_source": source}

    def _read_sensor_targets(self):
        data = self.structs.SimOne_Data_SensorDetections()
        ok = self.sensor_api.SoGetSensorDetections(
            self.config.vehicle_id, self.config.sensor_id, data
        )
        if not ok:
            return None, "none"
        targets = []
        for index in range(int(data.objectSize)):
            item = data.objects[index]
            targets.append(self._target_dict(item, float(item.probability)))
        return targets, "sensor:{0}".format(self.config.sensor_id)

    def _read_ground_truth_targets(self):
        data = self.structs.SimOne_Data_Obstacle()
        if not self.sensor_api.SoGetGroundTruth(self.config.vehicle_id, data):
            return []
        targets = []
        for index in range(int(data.obstacleSize)):
            targets.append(self._target_dict(data.obstacle[index], 1.0))
        return targets

    @staticmethod
    def _target_dict(item, probability):
        return {
            "id": int(item.id),
            "type": int(item.type),
            "x": float(item.posX),
            "y": float(item.posY),
            "z": float(item.posZ),
            "vx": float(item.velX),
            "vy": float(item.velY),
            "vz": float(item.velZ),
            "length": float(item.length),
            "width": float(item.width),
            "height": float(item.height),
            "probability": float(probability),
        }

    def get_driver_control(self):
        native = self.structs.SimOne_Data_Control()
        if not self.pnc_api.SoGetDriverControl(self.config.vehicle_id, native):
            return None
        result = ControlOut()
        result.throttle = float(native.throttle)
        result.brake = float(native.brake)
        result.steering = float(native.steering)
        result.gear = int(native.gear)
        result.handbrake = bool(native.handbrake)
        result.valid = True
        result.source = "SimOneDriver"
        return result

    def send_control(self, control):
        if not control or not control.valid:
            return False
        control.clamp()
        native = self.structs.SimOne_Data_Control()
        native.EThrottleMode = 0
        native.throttle = control.throttle
        native.EBrakeMode = 0
        native.brake = control.brake
        native.ESteeringMode = 0
        native.steering = control.steering
        native.handbrake = bool(control.handbrake)
        native.isManualGear = False
        native.gear = int(control.gear)
        self.pnc_api.SoSetDriveMode(self.config.vehicle_id, 0)
        ok = bool(self.pnc_api.SoSetDrive(self.config.vehicle_id, native))
        self._send_signals(control)
        return ok

    def _send_signals(self, control):
        lights = self.structs.SimOne_Data_Signal_Lights()
        if control.hazard_signal:
            lights.signalLights = 4
        else:
            lights.signalLights = (2 if control.left_signal else 0) | (
                1 if control.right_signal else 0
            )
        self.pnc_api.SoSetSignalLights(self.config.vehicle_id, lights)

    def shutdown(self):
        if self.connected and self.service_api is not None:
            try:
                self.service_api.SoTerminateSimOneAPI()
            finally:
                self.connected = False

    @staticmethod
    def gps_speed(gps):
        return math.sqrt(gps["vx"] * gps["vx"] + gps["vy"] * gps["vy"])
