"""Exact road/section/lane applicability of detached HD-map validity ranges."""


def matches_lane(scopes, lane_id):
    road, section, lane = (int(part) for part in lane_id.split('_'))
    if not isinstance(scopes, (list, tuple)) or not scopes:
        raise ValueError('map validity unavailable')
    matched = False
    for scope in scopes:
        fields = ('road_id', 'section_index', 'from_lane_id', 'to_lane_id')
        if not isinstance(scope, dict) or any(type(scope.get(key)) is not int for key in fields):
            raise ValueError('invalid map validity')
        low, high = scope['from_lane_id'], scope['to_lane_id']
        if scope['road_id'] < 0 or scope['section_index'] < 0 or -99 in (low, high):
            raise ValueError('undefined map validity')
        if (scope['road_id'] == road and scope['section_index'] == section
                and min(low, high) <= lane <= max(low, high) and lane != 0):
            matched = True
    return matched
