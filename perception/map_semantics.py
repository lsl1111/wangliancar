"""Interpret verified map facts without selecting driving behaviour (3.6)."""

import math

from core.geometry import project_polyline


# Installed SDK Autopilot/util/GetSignType.h: SpeedLimit_Sign.
# Do not identify other countries' codes or any numeric sign as a speed limit.
SPEED_LIMIT_SIGN_TYPE = "1010203800001413"


def _speed(value, unit):
    value = float(value)
    factor = {"km/h": 1.0 / 3.6, "kmh": 1.0 / 3.6, "kph": 1.0 / 3.6,
              "m/s": 1.0, "mph": 0.44704}.get(str(unit).strip().lower())
    if factor is None or not math.isfinite(value) or value <= 0:
        raise ValueError("unknown unit or invalid speed limit")
    return value * factor


def _matches_lane(scopes, lane_id):
    try:
        road, section, lane = (int(part) for part in lane_id.split("_"))
    except (ValueError, AttributeError):
        return False
    for scope in scopes:
        low, high = int(scope["from_lane_id"]), int(scope["to_lane_id"])
        if (int(scope["road_id"]) == road and int(scope["section_index"]) == section
                and min(low, high) <= lane <= max(low, high)
                and low != -99 and high != -99):
            return True
    return False


def speed_limit_observations(signs, signs_valid, ego, lane):
    """Expose approach distances; apply only a passed sign on its verified lane.

    Sign heading is its front-side normal, hence faces opposite approaching
    lane travel. Upcoming limits stay observations; anticipatory braking is
    a planning responsibility. No limit is carried across an unverified lane.
    """
    observations = []
    origin = project_polyline(lane.center_line, ego.x, ego.y) if ego.valid and lane.valid else None
    active = []
    for sign in signs:
        if not isinstance(sign, dict) or str(sign.get("type", "")) != SPEED_LIMIT_SIGN_TYPE:
            continue
        item = {"sign_id": sign.get("id"), "source": "hdmap", "speed_mps": -1.0,
                "along_distance": None, "applicable": False, "active": False,
                "reason": "unresolved"}
        try:
            item["speed_mps"] = _speed(sign["value"], sign["unit"])
            if not signs_valid or sign.get("valid") is not True:
                item["reason"] = "catalog_unusable"
            elif origin is None or not _matches_lane(sign["validities"], lane.lane_id):
                item["reason"] = "lane_scope_unresolved"
            elif sign.get("is_dynamic") is not False:
                item["reason"] = "dynamic_sign_unresolved"
            elif sign.get("heading_valid") is not True:
                item["reason"] = "heading_unresolved"
            else:
                projection = project_polyline(lane.center_line, sign["x"], sign["y"])
                angle = float(sign["heading"])
                if not math.isfinite(angle):
                    raise ValueError("bad heading")
                if projection is None or not 0 <= projection["raw_ratio"] <= 1:
                    item["reason"] = "sign_outside_lane_coverage"
                elif math.cos(angle - projection["heading"]) > -0.5:
                    item["reason"] = "sign_faces_other_direction"
                else:
                    item["along_distance"] = projection["s"] - origin["s"]
                    item["applicable"] = True
                    item["active"] = item["along_distance"] <= 0
                    item["reason"] = "passed_sign" if item["active"] else "upcoming_sign"
                    if item["active"]:
                        active.append(item)
        except (KeyError, TypeError, ValueError, OverflowError):
            item.update(applicable=False, active=False, reason="invalid_value_unit_or_scope")
        observations.append(item)
    limit, source = -1.0, "unavailable"
    if active:
        latest = max(item["along_distance"] for item in active)
        chosen = min((item for item in active if latest - item["along_distance"] < 1.0),
                     key=lambda item: item["speed_mps"])
        limit, source = chosen["speed_mps"], "hdmap_sign:{0}".format(chosen["sign_id"])
    return observations, limit, source
