import re


def resolve_scene_id(case_name, override=0):
    if int(override) > 0:
        return int(override)
    text = case_name or ""
    match = re.match(r"^\s*(\d{1,2})(?:[\.、_\-\s]|$)", text)
    if not match:
        return 0
    return int(match.group(1))

