"""INI configuration with project-relative defaults."""

import configparser
import os


class AppConfig(object):
    def __init__(self, project_dir, values):
        self.project_dir = project_dir
        self.sdk_dir = values.get("sdk_dir", r"E:\Sim-One\SimOneAPI\lib\Win64")
        self.python_exe = values.get("python_exe", r"E:\Sim-One\Tools\python36\python.exe")
        self.server_ip = values.get("server_ip", "127.0.0.1")
        self.vehicle_id = values.get("vehicle_id", "0")
        self.sensor_id = values.get("sensor_id", "perfectPerception1")
        self.loop_hz = max(1.0, float(values.get("loop_hz", "20")))
        self.map_timeout_sec = max(1.0, float(values.get("map_timeout_sec", "100")))
        self.connect_timeout_sec = max(1.0, float(values.get("connect_timeout_sec", "30")))
        self.scene_id_override = int(values.get("scene_id_override", "0"))
        self.send_control = _as_bool(values.get("send_control", "false"))
        self.publish_json = _as_bool(values.get("publish_json", "true"))
        self.log_level = values.get("log_level", "INFO").upper()
        self.runtime_dir = os.path.join(project_dir, "runtime_data")


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def load_config(project_dir, custom_path=None):
    parser = configparser.ConfigParser()
    default_path = os.path.join(project_dir, "config", "default.ini")
    loaded = parser.read(default_path, encoding="utf-8")
    if not loaded:
        raise RuntimeError("配置文件不存在: {0}".format(default_path))
    if custom_path:
        custom_path = os.path.abspath(custom_path)
        if not os.path.exists(custom_path):
            raise RuntimeError("指定配置文件不存在: {0}".format(custom_path))
        parser.read(custom_path, encoding="utf-8")
    if not parser.has_section("app"):
        raise RuntimeError("配置缺少 [app] 段")
    return AppConfig(project_dir, dict(parser.items("app")))

