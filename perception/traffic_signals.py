"""Publish separate applicable signal boundaries, never one catalog fault."""
import copy
import math

from core.geometry import project_polyline, projection_within_polyline
from core.interfaces import TrafficControl
from core.map_scope import matches_lane
from core.route_segments import verified_spans
from perception.traffic_geometry import signal_reference, stop_line_position


# ESimOne_TrafficLight_Status, SoGetTrafficLights.
STATES = {0: 'INVALID', 1: 'RED', 2: 'GREEN', 3: 'YELLOW', 4: 'RED_BLINK',
          5: 'GREEN_BLINK', 6: 'YELLOW_BLINK', 7: 'BLACK'}


def _length(points):
    return sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:]))


def _heading(points, progress):
    for a, b in zip(points, points[1:]):
        length = math.hypot(b[0]-a[0], b[1]-a[1])
        if progress <= length+1e-6 and length > 1e-6:
            return math.atan2(b[1]-a[1], b[0]-a[0])
        progress -= length
    return math.atan2(points[-1][1]-points[-2][1], points[-1][0]-points[-2][0])


def _movement_matches(record, lane, identities):
    scopes = record.get('signal_validities')
    if scopes is None or record.get('signal_scope_valid') is False or len(identities) < 2:
        return True
    branches = lane.successor_lane_ids
    if any(matches_lane(scopes, branch) for branch in branches):
        return matches_lane(scopes, identities[1])
    return True


def _unknown(distance, reason, identity=-1, kind='unknown'):
    return dict(stop_line_distance=distance, boundary_kind=kind,
                association_valid=True, valid=False, ambiguous=False,
                signal_state='UNKNOWN', signal_ids=[identity], count_down=-1,
                signal_distance=-1.0, reason=reason)


def build_traffic(adapter, ego, lane, errors):
    traffic = TrafficControl()
    traffic.speed_limit = lane.speed_limit
    if not ego.valid or not lane.valid:
        return traffic
    reference, identities = signal_reference(lane)
    if reference is None:
        return traffic
    origin = project_polyline(lane.center_line, ego.x, ego.y)
    if (origin is None or origin['distance'] > max(2.0, lane.lane_width)
            or not projection_within_polyline(origin, len(lane.center_line))):
        traffic.reason = 'signal_route_anchor_unavailable'
        return traffic
    spans = verified_spans(len(lane.center_line), len(reference), identities, lane.forward_lane_spans)
    def scope_boundary(identity):
        if identity == lane.lane_id or identity not in spans:
            return -1.0
        entry = _length(reference[:spans[identity][0]+1])-origin['s']
        return max(0.0, entry)
    receipts, failures, queries = [], [], []
    for identity in identities:
        try:
            records = adapter.read_traffic(identity) or []
            query = getattr(adapter, 'last_traffic_query', {})
            queries.append(isinstance(query, dict) and query.get('association_valid') is True)
            if isinstance(query, dict) and query.get('association_valid') is False:
                failures.append(identity)
            for record in records:
                item = copy.deepcopy(record)
                if not isinstance(item, dict):
                    raise ValueError('invalid signal record')
                item['association_lane_id'] = identity
                receipts.append(item)
        except Exception as exc:
            errors.append('TRAFFIC_UNAVAILABLE:'+type(exc).__name__)
            failures.append(identity)
    seen, unknown = [], []
    for record in receipts:
        identity = record['association_lane_id']
        diagnostic = dict(opendrive_id=record.get('opendrive_id'), lane_id=identity)
        try:
            scopes = record.get('signal_validities')
            if (scopes is not None and record.get('signal_scope_valid') is not False
                    and not matches_lane(scopes, identity)):
                diagnostic['reason'] = 'signal_lane_scope_excluded'
                traffic.diagnostics.append(diagnostic)
                continue
            if identity == lane.lane_id and not _movement_matches(record, lane, identities):
                diagnostic['reason'] = 'signal_selected_movement_excluded'
                traffic.diagnostics.append(diagnostic)
                continue
            status = int(record.get('status', 0))
            stop_s, lower = stop_line_position(reference, record, lane.lane_width)
            if stop_s is None:
                distance = scope_boundary(identity) if lower is None else max(0., lower-origin['s'])
                fact = _unknown(distance, 'signal_stopline_unresolved',
                    int(record.get('opendrive_id', -1)),
                    'scope_entry' if lower is None and distance >= 0 else
                    'coverage_end' if lower is not None else 'unknown')
                if fact not in unknown:
                    unknown.append(fact)
                diagnostic.update(reason=fact['reason'], boundary_kind=fact['boundary_kind'],
                                  boundary_distance=distance)
                traffic.diagnostics.append(diagnostic)
                continue
            distance = stop_s-origin['s']
            if distance < 0:
                continue
            heading = record.get('signal_heading')
            if heading is not None:
                if type(heading) not in (int, float) or not math.isfinite(heading):
                    raise ValueError('invalid signal heading')
                if math.cos(heading-_heading(reference, stop_s)) > -.5:
                    diagnostic['reason'] = 'signal_faces_other_direction'
                    traffic.diagnostics.append(diagnostic)
                    continue
            item = copy.deepcopy(record)
            item.pop('association_lane_id', None)
            item.update(opendrive_id=int(record.get('opendrive_id', -1)),
                count_down=int(record.get('count_down', -1)), stop_distance=distance,
                signal_state=STATES.get(status, 'UNKNOWN'),
                read_ok=(record.get('read_ok', True) is True and status in STATES and status != 0
                         and record.get('signal_scope_valid', True) is True), signal_distance=-1.)
            if record.get('x') is not None and record.get('y') is not None:
                x, y = float(record['x']), float(record['y'])
                if not all(math.isfinite(v) for v in (x, y)):
                    raise ValueError('invalid signal position')
                item['signal_distance'] = math.hypot(x-ego.x, y-ego.y)
            if item not in seen:
                seen.append(item)
        except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
            errors.append('TRAFFIC_RECORD_INVALID')
            distance = scope_boundary(identity)
            unknown.append(_unknown(distance, 'signal_record_invalid',
                                    kind='scope_entry' if distance >= 0 else 'unknown'))
            diagnostic['reason'] = 'signal_record_invalid'
            traffic.diagnostics.append(diagnostic)
    # A failed future query is confined to its verified entry boundary. With
    # no actual signal evidence, retain the legacy optional-source contract.
    if seen or unknown:
        for identity in set(failures):
            distance = scope_boundary(identity)
            unknown.append(_unknown(distance, 'signal_association_unavailable',
                                    kind='scope_entry' if distance >= 0 else 'unknown'))
    traffic.candidates = sorted(seen, key=lambda item: item['stop_distance'])
    pending = list(traffic.candidates)
    groups = list(unknown)
    while pending:
        first = pending[0]
        group = [item for item in pending if item['stop_distance']-first['stop_distance'] < 1.0]
        pending = pending[len(group):]
        states = set(item['signal_state'] for item in group)
        ids = sorted(set(item['opendrive_id'] for item in group))
        valid = len(states) == 1 and all(item['read_ok'] is True for item in group)
        reason = ('unique_lane_stopline_signal' if len(ids) == 1 else 'unanimous_lane_stopline_signals')
        if not all(item['read_ok'] is True for item in group):
            reason = 'signal_read_unavailable'
            errors.append('TRAFFIC_STATE_UNAVAILABLE')
        elif len(states) != 1:
            reason = 'signal_direction_unresolved'
            errors.append('TRAFFIC_AMBIGUOUS')
        groups.append(dict(stop_line_distance=first['stop_distance'], boundary_kind='stop_line',
            association_valid=True, valid=valid, ambiguous=len(states) != 1,
            signal_state=first['signal_state'] if valid else 'UNKNOWN', signal_ids=ids,
            count_down=min(item['count_down'] for item in group),
            signal_distance=first['signal_distance'], reason=reason))
    groups.sort(key=lambda group: (group['stop_line_distance'],
        0 if group['reason'] == 'signal_record_invalid' else 1))
    traffic.signal_groups, traffic.signal_groups_valid = groups, bool(groups) or (all(queries) and bool(queries) and not failures)
    if not groups:
        traffic.association_valid = all(queries) and bool(queries) and not failures
        if traffic.association_valid:
            traffic.signal_presence, traffic.reason = 'absent', 'no_applicable_signal_ahead'
        elif failures:
            traffic.reason = 'signal_association_unavailable'
        return traffic
    nearest = groups[0]
    traffic.association_valid = nearest['association_valid']
    traffic.required = traffic.observed = True
    traffic.signal_presence = 'present'
    traffic.stop_line_distance = nearest['stop_line_distance'] if nearest['boundary_kind'] == 'stop_line' else -1.
    traffic.signal_state, traffic.valid = nearest['signal_state'], nearest['valid']
    traffic.reason, traffic.ambiguous = nearest['reason'], nearest['ambiguous']
    traffic.signal_id = nearest['signal_ids'][0] if len(nearest['signal_ids']) == 1 else -1
    traffic.count_down, traffic.signal_distance = nearest['count_down'], nearest['signal_distance']
    return traffic
