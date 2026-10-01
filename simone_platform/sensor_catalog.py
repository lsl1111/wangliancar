"""Sensor capabilities from the bundled SDK's ESimOne_Node_Type enum."""

TARGET_INGESTION_VERSION = "sensor-discovery-v2"


def sensor_kind(value):
    """Accept SDK numeric strings and platform names without guessing from IDs."""
    if isinstance(value, bytes):
        value = value.split(b"\0", 1)[0].decode("utf-8", "replace")
    value = str(getattr(value, "value", value)).strip().lower()
    numeric = {"1": "camera", "2": "lidar", "3": "radar",
               "4": "ultrasonic", "5": "ultrasonic", "6": "gnss",
               "7": "perfect", "8": "v2x", "9": "fusion"}
    if value in numeric:
        return numeric[value]
    for word in ("ultrasonic", "perfect", "fusion", "camera", "lidar", "radar"):
        if word in value:
            return word
    return "unknown"


def target_sensor_ids(configurations, preferred=""):
    """Configured detection sources only; GNSS/radar/ultrasound use other APIs."""
    ids = []
    for item in configurations:
        if not isinstance(item, dict):
            continue
        identifier = item.get("id", "")
        if (isinstance(identifier, str) and identifier
                and sensor_kind(item.get("type", "")) in
                ("camera", "lidar", "perfect", "fusion") and identifier not in ids):
            ids.append(identifier)
    if preferred in ids:
        ids.remove(preferred)
        ids.insert(0, preferred)
    return ids
