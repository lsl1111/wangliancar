"""Validate segment indices after a consumer has checked route geometry."""


def verified_spans(current_count, forward_count, ids, spans):
    if (not isinstance(ids, (list, tuple)) or not ids
            or not all(isinstance(identity, str) and identity for identity in ids)
            or not isinstance(spans, (list, tuple)) or len(spans) != len(ids)):
        return {}
    previous_end, checked = 0, {}
    for index, span in enumerate(spans):
        if not isinstance(span, dict) or span.get('lane_id') != ids[index]:
            return {}
        start, end = span.get('start_index'), span.get('end_index')
        if (type(start) is not int or type(end) is not int
                or start != previous_end or not start < end < forward_count
                or ids[index] in checked
                or (index == 0 and end != current_count - 1)):
            return {}
        checked[ids[index]] = (start, end)
        previous_end = end
    return checked if previous_end == forward_count - 1 else {}
