"""Detached observations from the installed HDMap Python binding (3.6).

Map observations are static, not synchronized sensor frames. Do not invent
objects when a binding lacks an independent lane-object query.
"""

import copy
import math
import time


MAP_OBSERVATION_VERSION = "map-observations-v1"


def text(value):
    value = value.GetString() if hasattr(value, "GetString") else value
    if isinstance(value, bytes):
        value = value.split(b"\0", 1)[0]
        for encoding in ("utf-8", "gb18030"):
            try:
                return value.decode(encoding)
            except UnicodeDecodeError:
                pass
        return value.decode("utf-8", "replace")
    return str(value)


def number(value):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("nonfinite map number")
    return result


def vector_items(vector):
    if vector is None:
        raise ValueError("missing native vector")
    count = int(vector.Size()) if hasattr(vector, "Size") else len(vector)
    if not 0 <= count <= 10000:
        raise ValueError("map vector count out of range")
    return [vector.GetElement(i) if hasattr(vector, "GetElement") else vector[i]
            for i in range(count)]


def point(value):
    return [number(value.x), number(value.y), number(value.z)]


def heading(value):
    if all(hasattr(value, field) for field in ("x", "y", "z")):
        direction = point(value)
        if math.hypot(direction[0], direction[1]) <= 1e-9:
            return {"heading": None, "heading_vector": direction,
                    "heading_valid": False}
        return {"heading": math.atan2(direction[1], direction[0]),
                "heading_vector": direction, "heading_valid": True}
    # Retain compatibility with already detached numeric-angle fixtures.
    angle = number(value)
    return {"heading": angle,
            "heading_vector": [math.cos(angle), math.sin(angle), 0.0],
            "heading_valid": True}


def signal_record(native):
    scopes = []
    for scope in vector_items(native.validities):
        item = {"road_id": int(scope.roadId),
                "section_index": int(scope.sectionIndex),
                "from_lane_id": int(scope.fromLaneId),
                "to_lane_id": int(scope.toLaneId)}
        for field, key in (("stopLineIds", "stop_line_ids"),
                           ("crosswalkIds", "crosswalk_ids")):
            if hasattr(scope, field):
                item[key] = [int(v) for v in vector_items(getattr(scope, field))]
        scopes.append(item)
    coordinates = point(native.pt)
    result = {"id": int(native.id), "type": text(native.type),
              "sub_type": text(native.subType), "value": text(native.value),
              "unit": text(native.unit), "is_dynamic": bool(native.isDynamic),
              "x": coordinates[0], "y": coordinates[1], "z": coordinates[2],
              "validities": scopes, "source": "hdmap", "valid": True}
    result.update(heading(native.heading))
    return result


def _polygon_area(points):
    return abs(sum(a[0] * b[1] - b[0] * a[1]
                   for a, b in zip(points, points[1:] + points[:1]))) * 0.5


def parking_record(native):
    coordinates = point(native.pt)
    knots = [point(knot) for knot in vector_items(native.boundaryKnots)]
    if len(knots) != 4 or _polygon_area(knots) <= 1e-6:
        raise ValueError("invalid parking boundary")
    result = {"id": int(native.id), "x": coordinates[0], "y": coordinates[1],
              "z": coordinates[2], "boundary_knots": knots,
              # Preserve SDK order: a-d is front, a-b left, b-c rear, c-d right.
              "entrance_edge": [knots[0], knots[3]],
              "source": "hdmap", "valid": True,
              "occupancy": "unknown", "markings": {}}
    if hasattr(native, "roadId"):
        result["road_id"] = int(native.roadId)
    result.update(heading(native.heading))
    for side in ("front", "rear", "left", "right"):
        marker = getattr(native, side, None)
        if marker is not None:
            width = number(marker.width)
            if width < 0:
                raise ValueError("negative parking marking width")
            result["markings"][side] = {"side": text(marker.side),
                "type": text(marker.type), "color": text(marker.color), "width": width}
    return result


def object_record(native, lane_id, kind):
    coordinates = point(native.pt)
    knots = [point(knot) for knot in vector_items(native.boundaryKnots)]
    if len(knots) < (3 if kind == "crosswalk" else 2):
        raise ValueError("map object boundary missing")
    if kind == "crosswalk" and _polygon_area(knots) <= 1e-6:
        raise ValueError("empty crosswalk polygon")
    if kind == "stop_line" and all(math.hypot(k[0] - knots[0][0],
                                              k[1] - knots[0][1]) <= 1e-6
                                   for k in knots[1:]):
        raise ValueError("empty stop line")
    return {"id": int(native.id), "type": text(native.type), "kind": kind,
            "x": coordinates[0], "y": coordinates[1], "z": coordinates[2],
            "boundary_knots": knots, "lane_ids": [lane_id],
            "association_signal_ids": [], "source": "hdmap", "valid": True}


def _status(read_ok=False, reason="api_missing", scope="map"):
    return {"read_ok": read_ok, "reason": reason, "source": "hdmap",
            "clock": "static_map", "query_scope": scope,
            "coverage_complete": False, "invalid_records": 0,
            "errors": [], "version": MAP_OBSERVATION_VERSION}


class MapObservationReader(object):
    def __init__(self, hdmap):
        self.hdmap = hdmap
        self._catalog = None
        self._signals = []
        self._updated = None
        self._lane_cache = {}

    def catalog(self):
        now = time.monotonic()
        if self._catalog is not None and now - self._updated < 5.0:
            return copy.deepcopy(self._catalog)
        result = {"traffic_signs": [], "traffic_signs_valid": False,
                  "parking_spaces": [], "parking_spaces_valid": False,
                  "map_observation_status": {}}
        signals = []
        for api, key, converter in (("getTrafficSignList", "traffic_signs", signal_record),
                                    ("getParkingSpaceList", "parking_spaces", parking_record)):
            meta = _status()
            meta["api"] = api
            result["map_observation_status"][key] = meta
            if not hasattr(self.hdmap, api):
                continue
            try:
                natives = vector_items(getattr(self.hdmap, api)())
                meta.update(read_ok=True, reason="ok" if natives else "empty",
                            coverage_complete=True)
                for native in natives:
                    try:
                        result[key].append(converter(native))
                        if key == "traffic_signs":
                            signals.append(native)
                    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
                        meta["invalid_records"] += 1
                        meta["errors"].append(type(exc).__name__)
                result[key + "_valid"] = meta["invalid_records"] == 0
                if meta["invalid_records"]:
                    meta["reason"] = "invalid_records"
                    meta["coverage_complete"] = False
            except Exception as exc:
                meta.update(reason="read_failed", errors=[type(exc).__name__])
        light_meta = _status()
        light_meta["api"] = "getTrafficLightList"
        result["map_observation_status"]["traffic_light_catalog"] = light_meta
        if hasattr(self.hdmap, "getTrafficLightList"):
            try:
                lights = vector_items(self.hdmap.getTrafficLightList())
                signals.extend(lights)
                light_meta.update(read_ok=True, reason="ok" if lights else "empty",
                                  coverage_complete=True)
            except Exception as exc:
                light_meta.update(reason="read_failed", errors=[type(exc).__name__])
        self._signals = signals
        self._catalog, self._updated = result, now
        self._lane_cache = {}
        return copy.deepcopy(result)

    def lane_objects(self, lane_ids):
        catalog = self.catalog()
        lane_ids = tuple(dict.fromkeys(str(v) for v in lane_ids if v))
        if lane_ids in self._lane_cache:
            return copy.deepcopy(self._lane_cache[lane_ids])
        result = {"map_stop_lines": [], "map_stop_lines_valid": False,
                  "map_crosswalks": [], "map_crosswalks_valid": False,
                  "map_observation_status": {}}
        for api, key, kind in (("getStoplineList", "map_stop_lines", "stop_line"),
                               ("getCrosswalkList", "map_crosswalks", "crosswalk")):
            meta = _status(scope="signal_associated_lanes")
            meta["api"] = api
            result["map_observation_status"][key] = meta
            # The installed binding only offers (signal, lane) queries. Empty
            # associated results must not claim absence of independent objects.
            meta["independent_lane_query"] = "not_available_in_binding"
            if not lane_ids:
                meta["reason"] = "lane_unavailable"
                continue
            if not hasattr(self.hdmap, api) or not hasattr(self.hdmap, "pySimString"):
                continue
            catalog_statuses = [catalog["map_observation_status"][name]
                                for name in ("traffic_signs", "traffic_light_catalog")]
            meta["signal_catalog_complete"] = all(m["read_ok"] and not m["invalid_records"]
                                                   for m in catalog_statuses)
            if not any(m["read_ok"] for m in catalog_statuses):
                meta["reason"] = "signal_catalog_incomplete"
                continue
            objects = {}
            try:
                for lane_id in lane_ids:
                    for signal in self._signals:
                        try:
                            natives = vector_items(getattr(self.hdmap, api)(
                                signal, self.hdmap.pySimString(lane_id)))
                        except Exception as exc:
                            meta["errors"].append("query:" + type(exc).__name__)
                            continue
                        for native in natives:
                            try:
                                item = object_record(native, lane_id, kind)
                                existing = objects.setdefault(item["id"], item)
                                if existing["boundary_knots"] != item["boundary_knots"]:
                                    raise ValueError("conflicting map object identity")
                                if lane_id not in existing["lane_ids"]:
                                    existing["lane_ids"].append(lane_id)
                                if int(signal.id) not in existing["association_signal_ids"]:
                                    existing["association_signal_ids"].append(int(signal.id))
                            except (AttributeError, TypeError, ValueError, OverflowError) as exc:
                                meta["invalid_records"] += 1
                                meta["errors"].append(type(exc).__name__)
                result[key] = list(objects.values())
                query_failed = any(error.startswith("query:") for error in meta["errors"])
                meta.update(read_ok=not query_failed,
                            reason="ok" if objects else "empty_associated_scope")
                result[key + "_valid"] = not query_failed and meta["invalid_records"] == 0
                if meta["invalid_records"]:
                    meta["reason"] = "invalid_records"
                if query_failed:
                    meta["reason"] = "partial_read_failed"
            except Exception as exc:
                meta.update(reason="read_failed", errors=[type(exc).__name__])
        self._lane_cache[lane_ids] = result
        return copy.deepcopy(result)
