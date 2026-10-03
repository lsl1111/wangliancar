"""Narrow semantic association for Sensor API traffic-light fixtures.

Keep the original target records. Signal state and stop-line constraints remain
the responsibility of the traffic channel; an ID alone is never an exemption.
The SDK Sensor API enum (SimOneIOStruct.h) defines TrafficLight as 15.
"""

import math

from core.validation import number


def mapped_traffic_light(target, perception):
    """Recognize a stationary light only with current, matching map evidence."""
    statuses = getattr(perception, 'source_status', {})
    if not isinstance(statuses, dict):
        return False
    targets = statuses.get('targets', {})
    traffic_status = statuses.get('traffic', {})
    traffic = getattr(perception, 'traffic', None)
    if (getattr(target, 'valid', False) is not True
            or type(getattr(target, 'type', None)) is not int or target.type != 15
            or type(getattr(target, 'id', None)) is not int or target.id < 0
            or getattr(perception, 'targets_valid', False) is not True
            or not isinstance(getattr(perception, 'target_source', None), str)
            or not perception.target_source.startswith('sensor:')
            or not isinstance(targets, dict) or targets.get('usable') is not True
            or traffic is None or traffic.association_valid is not True
            or not isinstance(traffic_status, dict)
            or traffic_status.get('association_valid') is not True):
        return False
    coordinates = (target.x, target.y, target.z)
    velocity = (target.vx, target.vy, target.vz)
    dimensions = (target.length, target.width, target.height)
    if (not all(number(value) for value in coordinates + velocity)
            or not all(number(value) and value >= 0 for value in dimensions)
            or math.sqrt(sum(value * value for value in velocity)) > .05):
        return False
    candidates = getattr(traffic, 'candidates', None)
    if not isinstance(candidates, (list, tuple)):
        return False
    for candidate in candidates:
        if (not isinstance(candidate, dict)
                or type(candidate.get('opendrive_id')) is not int
                or candidate['opendrive_id'] != target.id
                or not number(candidate.get('x')) or not number(candidate.get('y'))):
            continue
        if math.hypot(target.x - candidate['x'], target.y - candidate['y']) <= .5:
            return True
    return False
