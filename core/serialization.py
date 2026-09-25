"""JSON-friendly views used for team debugging and interface inspection."""


def perception_to_dict(value):
    return {
        "scene_id": value.scene_id,
        "case_name": value.case_name,
        "frame_id": value.frame_id,
        "timestamp": value.timestamp,
        "valid": value.valid,
        "errors": list(value.errors),
        "ego": _slots(value.ego),
        "lane": _slots(value.lane),
        "traffic": _slots(value.traffic),
        "targets": [_slots(item) for item in value.targets],
    }


def _slots(value):
    return {name: getattr(value, name) for name in value.__dict__}

