"""Local evaluation bindings for the verified SimOne 3.0.0001 C ABI.

Reuse the DLL loaded by SimOneServiceAPI. The SDK records judge/mainvehicle
data itself; do not invent scoring records from the perception snapshot.
"""

import ctypes
import time


class LocalEvaluation(object):
    def __init__(self, library, version, logger, interval_sec=1.0):
        if isinstance(version, bytes):
            version = version.decode("ascii")
        # Later SDKs add another bool argument. Never call an unverified ABI.
        if version != "3.0.0001":
            raise RuntimeError("Unverified evaluation SDK ABI: " + str(version))
        self.logger = logger
        self.interval_sec = interval_sec
        try:
            self._init = library.InitEvaluationServiceWithLocalData
            self._save = library.SaveEvaluationRecord
        except AttributeError:
            raise RuntimeError("SDK 缺少本地评价初始化或保存接口")
        self._init.argtypes = [ctypes.c_char_p, ctypes.c_bool]
        self._init.restype = ctypes.c_bool
        self._save.argtypes = []
        self._save.restype = ctypes.c_bool
        self.active = False
        self.save_count = 0
        self.last_save_ok = None
        self.error = ""
        self._next_save = 0.0

    def initialize(self, vehicle_id):
        if self.active:
            return
        if not self._init(str(vehicle_id).encode("utf-8"), True):
            self.error = "initialization_failed"
            raise RuntimeError("本地评价初始化失败，未进入控制循环")
        self.active = True
        self._next_save = time.monotonic() + self.interval_sec
        self.logger.info("评价记录已初始化 mode=local vehicle=%s withMainVehicle=true",
                         vehicle_id)

    def flush(self, force=False):
        if not self.active:
            return True
        now = time.monotonic()
        if not force and now < self._next_save:
            return True
        self._next_save = now + self.interval_sec
        previous = self.last_save_ok
        try:
            ok = bool(self._save())
            self.error = "" if ok else "save_returned_false"
        except Exception as exc:
            ok = False
            self.error = "save_exception:" + type(exc).__name__
        self.last_save_ok = ok
        if ok:
            self.save_count += 1
            if force or previous is not True:
                self.logger.info("评价记录已保存 count=%s final=%s",
                                 self.save_count, force)
        elif force or previous is not False:
            self.logger.error("评价记录保存失败 reason=%s final=%s", self.error, force)
        return ok

    def status(self):
        return {"enabled": True, "mode": "local", "initialized": self.active,
                "save_count": self.save_count, "last_save_ok": self.last_save_ok,
                "error": self.error}
