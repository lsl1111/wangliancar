"""Repair the bundled 3.0.0001 polling structs without editing vendor files.

Sizes and field order were checked against this release's C++ header. Only
known legacy layouts are replaced; an unknown layout fails before native I/O.
Vendor callback typedefs are intentionally outside this polling-only adapter.
"""

import ctypes


class StructView(object):
    def __init__(self, original, overrides):
        self.original = original
        self.overrides = overrides

    def __getattr__(self, name):
        if name in self.overrides:
            return self.overrides[name]
        return getattr(self.original, name)


def _struct(name, base, fields):
    return type(name, (base,), {"_pack_": 1, "_fields_": fields})


def _array_fields(original, name, entry):
    fields = list(original.__dict__["_fields_"])
    for i, (field, kind) in enumerate(fields):
        if field == name:
            fields[i] = (field, entry * kind._length_)
            return fields
    raise RuntimeError("SDK array field missing: " + name)


def polling_structs(sdk, version):
    """Return (view, repaired names); all unrelated vendor types stay intact."""
    if isinstance(version, bytes):
        version = version.decode("ascii", "strict")
    layouts = {
        "SimOne_Data_WayPoints": (2820, 2884),
        "SimOne_Data_SensorDetections_Entry": (128, 228),
        "SimOne_Data_SensorDetections": (32788, 58388),
        "SimOne_Data_Obstacle_Entry": (76, 644),
        "SimOne_Data_Obstacle": (19400, 164240),
        "SimOne_Data_TrafficLight": (16, 13),
    }
    sizes = {name: ctypes.sizeof(getattr(sdk, name)) for name in layouts}
    mismatches = [name for name, (_, expected) in layouts.items()
                  if sizes[name] != expected]
    if not mismatches:
        return sdk, []
    if version != "3.0.0001" or any(sizes[name] not in layouts[name] for name in layouts):
        raise RuntimeError("Unverified SDK ABI: version={0}, sizes={1}".format(version, sizes))
    # Partial mixes are unsupported: array stride must match its entry type.
    for entry, array in (("SimOne_Data_SensorDetections_Entry", "SimOne_Data_SensorDetections"),
                         ("SimOne_Data_Obstacle_Entry", "SimOne_Data_Obstacle")):
        if (sizes[entry] == layouts[entry][0]) != (sizes[array] == layouts[array][0]):
            raise RuntimeError("Mixed SDK entry/array ABI: " + array)
    if ctypes.sizeof(sdk.SimOne_Data) != 16 or ctypes.sizeof(sdk.SimOneData_Vec3f) != 12:
        raise RuntimeError("Unexpected SDK header/vector ABI")
    overrides = {}
    if "SimOne_Data_TrafficLight" in mismatches:
        overrides["SimOne_Data_TrafficLight"] = _struct(
            "SimOne_Data_TrafficLight", ctypes.Structure,
            [("isMainVehicleNextTrafficLight", ctypes.c_bool),
             ("opendriveLightId", ctypes.c_int), ("countDown", ctypes.c_int),
             ("status", sdk.ESimOne_TrafficLight_Status)])
    if "SimOne_Data_WayPoints" in mismatches:
        fields = [("mainVehicleId", ctypes.c_char * 64)]
        fields.extend(sdk.SimOne_Data_WayPoints.__dict__["_fields_"])
        overrides["SimOne_Data_WayPoints"] = _struct(
            "SimOne_Data_WayPoints", sdk.SimOne_Data, fields)
    if "SimOne_Data_SensorDetections_Entry" in mismatches:
        entry = _struct("SimOne_Data_SensorDetections_Entry",
                        sdk.SimOne_Data_SensorDetections_Entry,
                        [("cornerPointSize", ctypes.c_int),
                         ("cornerPoints", sdk.SimOneData_Vec3f * 8)])
        overrides["SimOne_Data_SensorDetections_Entry"] = entry
        overrides["SimOne_Data_SensorDetections"] = _struct(
            "SimOne_Data_SensorDetections", sdk.SimOne_Data,
            _array_fields(sdk.SimOne_Data_SensorDetections, "objects", entry))
    if "SimOne_Data_Obstacle_Entry" in mismatches:
        rotation = _struct("Rotation", ctypes.Structure,
                           [("yaw", ctypes.c_float), ("pitch", ctypes.c_float),
                            ("roll", ctypes.c_float)])
        prediction = _struct("Prediction", ctypes.Structure,
                             [("trajectorySize", ctypes.c_uint32),
                              ("trajectoryInterval", ctypes.c_float),
                              ("trajectory", sdk.SimOneData_Vec3f * 20),
                              ("speed", ctypes.c_float * 20),
                              ("rotation", rotation * 20)])
        entry = _struct("SimOne_Data_Obstacle_Entry", sdk.SimOne_Data_Obstacle_Entry,
                        [("prediction", prediction)])
        overrides.update(Rotation=rotation, Prediction=prediction,
                         SimOne_Data_Obstacle_Entry=entry)
        overrides["SimOne_Data_Obstacle"] = _struct(
            "SimOne_Data_Obstacle", sdk.SimOne_Data,
            _array_fields(sdk.SimOne_Data_Obstacle, "obstacle", entry))
    view = StructView(sdk, overrides)
    for name, (_, expected) in layouts.items():
        if ctypes.sizeof(getattr(view, name)) != expected:
            raise RuntimeError("SDK repair size mismatch: " + name)
    return view, mismatches
