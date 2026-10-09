"""Read captain-produced maneuver facts with their original source deadlines.

SDK-free shared consumer. A readable frame is not clearance or crossing
permission. This reader neither creates geometry nor refreshes evidence.
"""
import copy
import re
import time

from core.interfaces import Perception, ManeuverEnvironment
from core.behavior_contract import require as _require
from core.validation import number
from core.region_geometry import convex_polygon, inside, outline_contains_point


def _points(raw, dimensions, minimum=2, maximum=20000):
    _require(isinstance(raw, (list, tuple)) and minimum <= len(raw) <= maximum
             and all(isinstance(p, (list, tuple)) and len(p) == dimensions
                     and all(number(v) for v in p) for p in raw), 'MANEUVER_GEOMETRY_INVALID')
    return [tuple(p) for p in raw]


class ManeuverFacts(object):
    def __init__(self, perception, clock=None):
        self.p, self.clock = perception, clock or time.monotonic
        _require(isinstance(perception, Perception), 'MANEUVER_PERCEPTION_UNAVAILABLE')
        self.env = perception.maneuver_environment
        self.scope = (perception.case_id, perception.task_id, perception.scene_id, perception.frame_id)
        self._fresh()

    def _fresh(self, dynamic=False):
        p, env, now = self.p, self.env, self.clock()
        _require(number(now) and now >= 0 and isinstance(env, ManeuverEnvironment)
                 and p.maneuver_environment is env and env.contract_version == 'maneuver-environment-v1'
                 and p.valid is True and p.ego.valid is True and env.valid is True
                 and isinstance(p.case_id, str) and bool(p.case_id)
                 and isinstance(p.task_id, str) and bool(p.task_id) and type(p.scene_id) is int
                 and type(p.frame_id) is int and p.frame_id >= 0
                 and type(p.ego.frame_id) is int and p.ego.frame_id == p.frame_id
                 and type(env.frame_id) is int and env.frame_id == p.frame_id
                 and (p.case_id, p.task_id, p.scene_id, p.frame_id) == self.scope
                 and (env.case_id, env.task_id, env.scene_id, env.frame_id) == self.scope
                 and type(env.timestamp) is int and type(p.timestamp) is int and env.timestamp == p.timestamp
                 and all(number(v) for v in (p.valid_until, env.valid_until, env.observed_at_s))
                 and 0 <= env.observed_at_s <= now < env.valid_until <= p.valid_until
                 and all(number(getattr(p.ego, v)) for v in ('x', 'y', 'z', 'heading')),
                 'MANEUVER_FRAME_EXPIRED_OR_MISMATCHED')
        gps = p.source_status.get('gps', {}) if isinstance(p.source_status, dict) else {}
        _require(isinstance(gps, dict) and gps.get('usable') is True
                 and gps.get('quality') not in ('stale', 'unsynchronized', 'regressed', 'invalid', 'timing_unknown')
                 and ('frame_id' not in gps or type(gps['frame_id']) is int and gps['frame_id'] == p.frame_id),
                 'MANEUVER_GPS_UNAVAILABLE')
        if dynamic:
            meta = p.source_status.get('targets', {})
            status = env.status.get('objects', {}) if isinstance(env.status, dict) else {}
            _require(p.targets_valid is True and isinstance(meta, dict) and meta.get('usable') is True
                     and meta.get('quality') not in ('stale', 'unsynchronized', 'regressed', 'invalid', 'timing_unknown')
                     and ('frame_id' not in meta or type(meta['frame_id']) is int and meta['frame_id'] == p.targets_frame_id)
                     and isinstance(p.target_source, str) and p.target_source.startswith('sensor:')
                     and bool(p.target_source[7:]) and isinstance(status, dict)
                     and status.get('usable') is True and status.get('source') == p.target_source
                     and type(p.targets_frame_id) is int and 0 <= p.targets_frame_id <= p.frame_id
                     and all(number(v) for v in (env.dynamic_observed_at_s, env.dynamic_valid_until))
                     and 0 <= env.dynamic_observed_at_s <= env.observed_at_s
                     and now < env.dynamic_valid_until <= env.valid_until,
                     'MANEUVER_DYNAMIC_SOURCE_UNAVAILABLE')
        return now

    def _evidence(self, record, kind, covered=False):
        dynamic = kind in ('sensor', 'verified_fusion')
        now = self._fresh(dynamic)
        env = self.env
        observed = env.dynamic_observed_at_s if dynamic else env.observed_at_s
        deadline = env.dynamic_valid_until if dynamic else env.valid_until
        _require(isinstance(record, dict) and type(record.get('frame_id')) is int and record['frame_id'] == env.frame_id
                 and record.get('clock_id') == 'process_monotonic' and record.get('source_kind') == kind
                 and record.get('source') == (self.p.target_source if dynamic else 'sdk_matched_opendrive')
                 and all(number(record.get(v)) for v in ('observed_at_s', 'valid_until_s'))
                 and 0 <= record['observed_at_s'] <= observed <= now < record['valid_until_s'] <= deadline
                 and type(record.get('coverage_verified')) is bool
                 and (not covered or record['coverage_verified'] is True),
                 'MANEUVER_EVIDENCE_EXPIRED_OR_MISMATCHED')
        return copy.deepcopy(record)

    def check_current(self, dynamic=False):
        return self._fresh(dynamic)

    def map_digest(self):
        self._fresh()
        value = self.env.map_semantics
        _require(isinstance(value, dict) and value.get('verified') is True
                 and value.get('semantic_verified') is True and value.get('geometry_bound') is True
                 and value.get('current_lane_id') == self.p.lane.lane_id and self.p.lane.valid is True
                 and isinstance(value.get('digest'), str)
                 and re.fullmatch('[0-9a-f]{32}', value['digest']) is not None,
                 'MANEUVER_MAP_IDENTITY_UNAVAILABLE')
        self._evidence(value.get('evidence'), 'map')
        return value['digest']

    def objects(self):
        self._fresh(True)
        raw = self.env.objects
        _require(isinstance(raw, list) and len(raw) <= 4096 and isinstance(self.p.targets, list)
                 and len(raw) == len(self.p.targets), 'MANEUVER_OBJECT_SET_INCOMPLETE')
        targets, identities, result = {}, set(), []
        for target in self.p.targets:
            self._fresh(True)
            _require(type(target.id) is int and target.id >= 0 and target.id not in targets
                     and target.valid is True and target.source == self.p.target_source,
                     'MANEUVER_OBJECT_SOURCE_MISMATCH')
            targets[target.id] = target
        for value in raw:
            self._fresh(True)
            _require(isinstance(value, dict) and type(value.get('id')) is int
                     and value['id'] in targets and value['id'] not in identities
                     and value.get('source') == self.p.target_source
                     and type(value.get('source_frame_id')) is int and value['source_frame_id'] == self.p.targets_frame_id
                     and all(number(value.get(v)) for v in ('x', 'y', 'z', 'vx', 'vy', 'length', 'width', 'probability'))
                     and min(value['length'], value['width']) > 0 and .05 <= value['probability'] <= 1
                     and (value.get('heading') is None or number(value['heading'])),
                     'MANEUVER_OBJECT_GEOMETRY_OR_SOURCE_INVALID')
            target = targets[value['id']]
            _require(all(value[v] == getattr(target, v) for v in ('x', 'y', 'z', 'vx', 'vy', 'length', 'width', 'probability'))
                     and value.get('heading') == (target.heading if number(target.heading) else None),
                     'MANEUVER_OBJECT_SOURCE_MISMATCH')
            self._evidence(value.get('evidence'), 'sensor')
            identities.add(value['id'])
            result.append(copy.deepcopy(value))
        self._fresh(True)
        return result

    def coverage(self, geometry_xyz):
        self._fresh(True)
        geometry = _points(geometry_xyz, 3, minimum=1, maximum=60000)
        raw = self.env.coverage_regions
        _require(isinstance(raw, list) and len(raw) <= 32, 'MANEUVER_COVERAGE_INVALID')
        result = []
        for value in raw:
            self._fresh(True)
            _require(isinstance(value, dict), 'MANEUVER_COVERAGE_INVALID')
            self._evidence(value, 'sensor', True)
            _require(value.get('complete_detections') is True
                     and type(value.get('source_frame_id')) is int and value['source_frame_id'] == self.p.targets_frame_id
                     and type(value.get('pose_frame_id')) is int and value['pose_frame_id'] == self.p.targets_frame_id
                     and value.get('pose_source') in ('current_frame_pose', 'sensor_pose_history_exact_frame')
                     and value.get('sensor_id') == self.p.target_source[7:]
                     and isinstance(value.get('verification_reference'), str)
                     and bool(value['verification_reference'].strip()) and number(value.get('reference_z_m')),
                     'MANEUVER_COVERAGE_SOURCE_UNVERIFIED')
            polygon = convex_polygon(value.get('polygon'))
            if all(abs(p[2] - value['reference_z_m']) <= .1 for p in geometry):
                item = copy.deepcopy(value)
                item['polygon'] = polygon
                result.append(item)
        self._fresh(True)
        return result

    def road(self, lane_id, require_coverage=True):
        digest = self.map_digest()
        _require(isinstance(lane_id, str) and bool(lane_id) and isinstance(self.env.road_regions, list)
                 and len(self.env.road_regions) <= 32, 'MANEUVER_ROAD_REGION_INVALID')
        matches = [v for v in self.env.road_regions if isinstance(v, dict) and v.get('lane_id') == lane_id]
        _require(len(matches) == 1, 'MANEUVER_ROAD_REGION_UNAVAILABLE_OR_DUPLICATE')
        road = matches[0]
        _require(road.get('geometry_valid') is True and road.get('drivable_verified') is True
                 and road.get('source') == 'sdk_matched_opendrive' and road.get('map_digest') == digest
                 and road.get('coordinate_system') == 'world_xyz_metres'
                 and road.get('geometry_model') == 'sdk_piecewise_linear_lane_v1'
                 and road.get('role') == ('current' if lane_id == self.p.lane.lane_id else 'neighbor'),
                 'MANEUVER_ROAD_SOURCE_OR_TYPE_UNVERIFIED')
        self._evidence(road.get('geometry_evidence'), 'map')
        geometry = {name: _points(road.get(name), 3) for name in ('center_line', 'left_boundary', 'right_boundary')}
        polygon = _points(road.get('polygon'), 2, minimum=3, maximum=40000)
        _require(polygon == [p[:2] for p in geometry['left_boundary']]
                 + [p[:2] for p in reversed(geometry['right_boundary'])]
                 and all(abs(p[2] - self.p.ego.z) <= .1 for line in geometry.values() for p in line),
                 'MANEUVER_ROAD_OUTLINE_OR_PLANE_MISMATCH')
        views = []
        if require_coverage:
            self.objects()
            self._evidence(road.get('evidence'), 'verified_fusion', True)
            _require(road.get('coverage_verified') is True, 'MANEUVER_ROAD_COVERAGE_UNKNOWN')
            raw = [p for line in geometry.values() for p in line]
            views = [v for v in self.coverage(raw) if all(inside(p[:2], v['polygon']) for p in raw)]
            _require(bool(views), 'MANEUVER_ROAD_COVERAGE_UNKNOWN')
        result = dict(geometry, lane_id=lane_id, polygon=polygon, map_digest=digest,
                      geometry_model=road['geometry_model'], coverage=views,
                      occupancy=road.get('occupancy', 'unknown'), object_ids=copy.deepcopy(road.get('object_ids', [])),
                      evidence=copy.deepcopy(road.get('evidence')), geometry_evidence=copy.deepcopy(road['geometry_evidence']))
        self._fresh(require_coverage)
        return result

    def neighbors(self):
        self._fresh()
        _require(isinstance(self.env.neighbor_lanes, list) and len(self.env.neighbor_lanes) <= 32,
                 'MANEUVER_NEIGHBOR_LIST_INVALID')
        _require(all(isinstance(v, dict) and isinstance(v.get('lane_id'), str) for v in self.env.neighbor_lanes),
                 'MANEUVER_NEIGHBOR_LIST_INVALID')
        identities = [v['lane_id'] for v in self.env.neighbor_lanes]
        _require(len(identities) == len(set(identities)), 'MANEUVER_NEIGHBOR_DUPLICATE')
        return list(identities)

    def crossing(self, lane_id):
        digest = self.map_digest()
        _require(lane_id in self.neighbors(), 'MANEUVER_NEIGHBOR_UNAVAILABLE')
        value = next(v for v in self.env.neighbor_lanes if v['lane_id'] == lane_id)
        _require(all(value.get(k) is True for k in ('geometry_valid', 'shared_boundary_verified',
                    'same_direction', 'crossing_range_verified', 'crossing_geometry_bound',
                    'travel_direction_verified', 'travel_matches_declared_direction'))
                 and value.get('marking_map_digest') == digest and value.get('side') in ('left', 'right'),
                 'MANEUVER_CROSSING_UNVERIFIED')
        self._evidence(value.get('marking_evidence'), 'map')
        window = value.get('crossing_boundary_window')
        _require(isinstance(window, dict) and window.get('verification_model') == 'sdk_piecewise_linear_boundary_v1',
                 'MANEUVER_CROSSING_WINDOW_UNVERIFIED')
        boundary = _points(window.get('shared_boundary_world'), 3)
        stations = window.get('road_s_samples_m')
        _require(isinstance(stations, list) and len(stations) == len(boundary)
                 and all(number(v) for v in stations)
                 and window.get('travel_direction_verified') is True
                 and window.get('travel_matches_declared_direction') is True
                 and type(window.get('declared_road_s_direction')) is int
                 and window['declared_road_s_direction'] in (-1, 1)
                 and all((b - a) * window['declared_road_s_direction'] > 0 for a,b in zip(stations, stations[1:]))
                 and number(window.get('road_s_transition_guard_m')) and window['road_s_transition_guard_m'] >= .25,
                 'MANEUVER_CROSSING_WINDOW_UNVERIFIED')
        positions = {point:index for index,point in enumerate(boundary)}
        _require(len(positions) == len(boundary), 'MANEUVER_CROSSING_BOUNDARY_AMBIGUOUS')
        ranges = value.get('crossing_ranges_world')
        _require(isinstance(ranges, list) and len(ranges) <= 20000, 'MANEUVER_CROSSING_RANGES_INVALID')
        fragments, count = [], 0
        for part in ranges:
            self._fresh()
            _require(isinstance(part, dict) and isinstance(part.get('marking'), dict)
                     and part['marking'].get('semantic_verified') is True
                     and part['marking'].get('crossing_permitted') is True
                     and part['marking'].get('type') == 'broken', 'MANEUVER_CROSSING_RANGES_INVALID')
            fragment = _points(part.get('shared_boundary_world'), 3)
            index = positions.get(fragment[0])
            _require(index is not None and boundary[index:index+len(fragment)] == fragment
                     and part.get('road_s_samples_m') == stations[index:index+len(fragment)]
                     and all(number(part['marking'].get(k)) for k in ('road_s_start_m', 'road_s_end_m'))
                     and min(part['road_s_samples_m']) >= part['marking']['road_s_start_m'] + window['road_s_transition_guard_m']
                     and max(part['road_s_samples_m']) <= part['marking']['road_s_end_m'] - window['road_s_transition_guard_m'],
                     'MANEUVER_CROSSING_FRAGMENT_NOT_BOUND')
            fragments.append(fragment)
            count += len(fragment)
            _require(count <= 40000, 'MANEUVER_CROSSING_RANGES_INVALID')
        self._fresh()
        return dict(lane_id=lane_id, side=value['side'], shared_boundary_world=boundary,
                    permitted_fragments=fragments, map_digest=digest,
                    evidence=copy.deepcopy(value['marking_evidence']))

    def road_clear_at(self, lane_id, anchor_xyz):
        """Current complete occupancy of the old blockage location, not a path."""
        anchor = _points([anchor_xyz], 3, minimum=1)[0]
        road = self.road(lane_id)
        views = self.coverage([anchor])
        clear = (road['occupancy'] == 'clear' and road['object_ids'] == []
                 and outline_contains_point(anchor[:2], road['polygon'])
                 and any(inside(anchor[:2], v['polygon']) for v in views))
        self._fresh(True)
        return clear
