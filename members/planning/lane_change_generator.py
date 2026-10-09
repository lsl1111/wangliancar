"""Bounded P03 quintic candidate search using the existing P02 validator.

Rear-axle world XY, body yaw, nonnegative forward speed, relative seconds.
The sampled points define P02's nominal interpolation, not the polynomial's
continuous execution. Road/crossing authority and visibility stay separate.
No capabilities, stage changes, SDK calls or actuator guarantees are created.
"""
import math
import time

from core.geometry import normalize_angle, projection_within_polyline, opposes_direction
from core.interfaces import TrajectoryPoint
from core.validation import number
from members.planning.candidate_validation import (
    VehicleGeometry, MotionLimits, ValidationBudget, ObstaclePrediction, CorridorRegion,
    validate_candidate,
)
from members.planning.crossing_corridor import CrossingCorridor
from members.planning.maneuver_scene import read_maneuver_scene
from core.maneuver_facts import ManeuverFacts


EPS = 1e-9


class LaneChangeSearch(object):
    """Explicit search choices, never inferred vehicle/source capabilities."""
    def __init__(self, distances_m, tangent_scales, speed_scales, spacing_m,
                 max_points, projection_ambiguity_m):
        self.distances_m, self.tangent_scales = distances_m, tangent_scales
        self.speed_scales, self.spacing_m = speed_scales, spacing_m
        self.max_points, self.projection_ambiguity_m = max_points, projection_ambiguity_m


class PredictionEnvelope(object):
    """Explicit constant-velocity model bounds measured from source time.

    horizon_s is an authorized prediction horizon, not a Sensor TTL. Bounds
    must come from the caller's verified model; probability supplies neither.
    """
    def __init__(self, horizon_s, position_uncertainty_m, velocity_uncertainty_mps):
        self.horizon_s = horizon_s
        self.position_uncertainty_m = position_uncertainty_m
        self.velocity_uncertainty_mps = velocity_uncertainty_mps


def _source_predictions(objects, prediction_envelopes, now):
    """Shared P03/P04 original-observation prediction adaptation."""
    _require(isinstance(prediction_envelopes,dict)
             and set(prediction_envelopes)==set(v['id'] for v in objects),'PREDICTION_OBJECT_SET_MISMATCH')
    predictions=[]
    for obj in objects:
        envelope=prediction_envelopes[obj['id']]
        _require(isinstance(envelope,PredictionEnvelope)
                 and number(envelope.horizon_s) and envelope.horizon_s>0
                 and all(number(v) and v>=0 for v in (envelope.position_uncertainty_m,
                                                      envelope.velocity_uncertainty_mps)),
                 'PREDICTION_ENVELOPE_UNAVAILABLE')
        age=now-obj['evidence']['observed_at_s']
        _require(number(age) and age>=0 and age<envelope.horizon_s,'PREDICTION_SOURCE_HORIZON_EXPIRED')
        predictions.append(ObstaclePrediction(str(obj['id']),obj['x']+obj['vx']*age,
            obj['y']+obj['vy']*age,obj['length'],obj['width'],obj['heading'],obj['vx'],obj['vy'],
            envelope.horizon_s-age,envelope.position_uncertainty_m+envelope.velocity_uncertainty_mps*age,
            envelope.velocity_uncertainty_mps))
    return predictions


class _Failure(Exception):
    def __init__(self, code, status='invalid'):
        self.code, self.status = code, status


def _require(condition, code):
    if not condition:
        raise _Failure(code)


class _Work(object):
    def __init__(self, budget):
        self.budget, self.count = budget, 0

    def step(self):
        self.count += 1
        if self.count > self.budget.max_checks or time.monotonic() >= self.budget.deadline_monotonic_s:
            raise _Failure('BUDGET_EXHAUSTED', 'inconclusive')

    def validate(self, points, vehicle, limits, region, objects, direction=1):
        self.step()
        remaining = self.budget.max_checks - self.count
        if remaining <= 0:
            raise _Failure('BUDGET_EXHAUSTED', 'inconclusive')
        budget = ValidationBudget(remaining, self.budget.max_depth,
                                  self.budget.min_interval_s, self.budget.deadline_monotonic_s)
        result = validate_candidate(points, direction, vehicle, limits, region, objects, budget)
        self.count += result['checks']
        if result['reason_code'] == 'BUDGET_EXHAUSTED':
            return result
        self.step()
        return result


def _motion_parameters(start, speed, curvature, cap, vehicle, limits, budget):
    _require(isinstance(start, (list, tuple)) and len(start) == 3
             and all(number(v) for v in start), 'START_REAR_AXLE_POSE_INVALID')
    _require(number(speed) and speed >= 0 and number(curvature)
             and number(cap) and cap > 0, 'START_MOTION_OR_SPEED_CAP_UNAVAILABLE')
    _require(isinstance(vehicle, VehicleGeometry) and isinstance(limits, MotionLimits)
             and all(number(v) and v > 0 for v in vars(vehicle).values())
             and vehicle.front_offset_m >= vehicle.wheelbase_m, 'VEHICLE_GEOMETRY_UNAVAILABLE')
    _require(all(number(v) and (v >= 0 if k in ('heading_tolerance_rad', 'distance_tolerance_m') else v > 0)
                 for k, v in vars(limits).items())
             and limits.max_front_steer_rad < math.pi/2
             and limits.heading_tolerance_rad < math.pi/2, 'MOTION_LIMITS_UNAVAILABLE')
    _require(speed <= min(cap, limits.max_speed_mps)
             and abs(math.atan(vehicle.wheelbase_m*curvature)) <= limits.max_front_steer_rad,
             'INITIAL_MOTION_OUTSIDE_LIMITS')
    _require(isinstance(budget, ValidationBudget)
             and type(budget.max_checks) is int and budget.max_checks > 0
             and type(budget.max_depth) is int and 0 <= budget.max_depth <= 40
             and number(budget.min_interval_s) and budget.min_interval_s > 0
             and number(budget.deadline_monotonic_s) and budget.deadline_monotonic_s > 0,
             'VALIDATION_BUDGET_INVALID')


def _parameters(start, speed, curvature, cap, vehicle, limits, search, budget):
    _motion_parameters(start,speed,curvature,cap,vehicle,limits,budget)
    _require(isinstance(search, LaneChangeSearch), 'LANE_CHANGE_SEARCH_UNAVAILABLE')
    for values in (search.distances_m, search.tangent_scales, search.speed_scales):
        _require(isinstance(values, (list, tuple)) and 1 <= len(values) <= 8
                 and all(number(v) and v > 0 for v in values), 'SEARCH_CHOICES_INVALID')
    _require(len(search.distances_m)*len(search.tangent_scales)*len(search.speed_scales) <= 64
             and all(v <= 1 for v in search.speed_scales)
             and number(search.spacing_m) and search.spacing_m > 0
             and type(search.max_points) is int and 3 <= search.max_points <= 2000
             and number(search.projection_ambiguity_m) and search.projection_ambiguity_m >= 0,
             'SEARCH_BOUNDS_INVALID')


def _reference(raw, start, ambiguity, vehicle, work):
    _require(isinstance(raw, (list, tuple)) and 2 <= len(raw) <= 20000,
             'TARGET_REFERENCE_UNAVAILABLE')
    points, stations, projections = [], [0.], []
    for point in raw:
        work.step()
        _require(isinstance(point, (list, tuple)) and len(point) in (2, 3)
                 and all(number(v) for v in point), 'TARGET_REFERENCE_INVALID')
        point = tuple(point[:2])
        if not points or point != points[-1]:
            points.append(point)
    _require(len(points) >= 2, 'TARGET_REFERENCE_DEGENERATE')
    for index, (a, b) in enumerate(zip(points, points[1:])):
        work.step()
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(dx, dy)
        _require(number(length) and length > EPS, 'TARGET_REFERENCE_NUMERIC_RESOLUTION')
        raw_ratio = ((start[0]-a[0])*dx+(start[1]-a[1])*dy)/(length*length)
        ratio = max(0., min(1., raw_ratio))
        foot = a[0]+ratio*dx, a[1]+ratio*dy
        projections.append(dict(index=index, raw_ratio=raw_ratio,
            s=stations[-1]+ratio*length, distance=math.hypot(start[0]-foot[0], start[1]-foot[1]),
            heading=math.atan2(dy, dx)))
        stations.append(stations[-1]+length)
    nearest = min(projections, key=lambda v: v['distance'])
    _require(projection_within_polyline(nearest, len(points)), 'TARGET_REFERENCE_EXTRAPOLATION')
    for candidate in projections:
        work.step()
        _require(candidate['distance'] > nearest['distance']+ambiguity
                 or abs(candidate['s']-nearest['s']) <= vehicle.front_offset_m+vehicle.rear_offset_m,
                 'TARGET_REFERENCE_BRANCH_AMBIGUOUS')
    _require(abs(normalize_angle(nearest['heading']-start[2])) < math.pi/2,
             'TARGET_REFERENCE_OPPOSITE_DIRECTION')
    return points, stations, nearest['s']


def _goal(reference, stations, station, work):
    _require(station < stations[-1], 'TARGET_REFERENCE_TOO_SHORT')
    for index, (a, b) in enumerate(zip(reference, reference[1:])):
        work.step()
        if stations[index] <= station <= stations[index+1]:
            ratio = (station-stations[index])/(stations[index+1]-stations[index])
            # A corner has two tangents. Do not silently select a branch or
            # claim a curved join to subsequent target-lane tracking.
            if ratio == 1 and index+2 < len(reference):
                c = reference[index+2]
                before, after = math.atan2(b[1]-a[1], b[0]-a[0]), math.atan2(c[1]-b[1], c[0]-b[0])
                _require(abs(normalize_angle(after-before)) <= EPS, 'TARGET_TANGENT_AT_CORNER_UNKNOWN')
            return (a[0]+ratio*(b[0]-a[0]), a[1]+ratio*(b[1]-a[1]), math.atan2(b[1]-a[1], b[0]-a[0]))
    raise _Failure('TARGET_REFERENCE_TOO_SHORT')


def _coefficients(delta, first, last, second):
    # Quintic Hermite in u: endpoint positions, tangents and second derivatives.
    # The target lies on a nominal polyline edge; its local curvature is zero.
    c0, c1, c2 = 0., first, second*.5
    a, b, c = delta-c1-c2, last-c1-2*c2, -2*c2
    return c0, c1, c2, 10*a-4*b+c*.5, -15*a+7*b-c, 6*a-3*b+c*.5


def _value(coefficients, u):
    value, derivative = coefficients[-1], 5*coefficients[-1]
    for i in range(4, -1, -1):
        value = value*u+coefficients[i]
        if i > 0:
            derivative = derivative*u+i*coefficients[i]
    return value, derivative


def _shape(start, goal, curvature, scale, search, work, direction=1, end_scale=None):
    chord = math.hypot(goal[0]-start[0], goal[1]-start[1])
    _require(number(chord) and chord > EPS, 'CONNECTOR_DEGENERATE')
    tangent = chord*scale
    end_tangent = chord*(scale if end_scale is None else end_scale)
    c, s = math.cos(start[2]), math.sin(start[2])
    end_c, end_s = math.cos(goal[2]), math.sin(goal[2])
    cx = _coefficients(goal[0]-start[0], direction*tangent*c, direction*end_tangent*end_c, -s*curvature*tangent*tangent)
    cy = _coefficients(goal[1]-start[1], direction*tangent*s, direction*end_tangent*end_s, c*curvature*tangent*tangent)
    intervals = max(2, int(math.ceil(chord/search.spacing_m)))
    while intervals+1 <= search.max_points:
        shape = []
        for index in range(intervals+1):
            work.step()
            u = float(index)/intervals
            x, dx = _value(cx, u); y, dy = _value(cy, u)
            _require(all(number(v) for v in (x, y, dx, dy)) and math.hypot(dx, dy) > EPS,
                     'CONNECTOR_TANGENT_UNRESOLVED')
            heading = math.atan2(dy,dx)+(math.pi if direction==-1 else 0.)
            shape.append((start[0]+x, start[1]+y, normalize_angle(heading)))
        shape[0], shape[-1] = tuple(start), tuple(goal)
        lengths = [math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(shape, shape[1:])]
        _require(all(number(v) and v > EPS for v in lengths), 'CONNECTOR_SEGMENT_UNRESOLVED')
        if max(lengths) <= search.spacing_m:
            return shape, lengths
        intervals *= 2
    raise _Failure('CANDIDATE_POINT_LIMIT', 'inconclusive')


def _timed(shape, lengths, initial, desired, cap, curvature, vehicle, limits, work,
           direction=1, stop_at_goal=False):
    bounds = [min(cap, limits.max_speed_mps)]*len(shape)
    def restrict(indices, k):
        _require(number(k) and abs(math.atan(vehicle.wheelbase_m*k)) <= limits.max_front_steer_rad,
                 'CONNECTOR_STEERING_LIMIT')
        speed = (math.sqrt(limits.max_lateral_acceleration_mps2/abs(k)) if abs(k) > EPS
                 else min(cap,limits.max_speed_mps))
        for i in indices:
            bounds[i] = min(bounds[i], speed)
    for i, (a, b) in enumerate(zip(shape, shape[1:])):
        work.step()
        restrict((i, i+1), normalize_angle(b[2]-a[2])/(direction*lengths[i]))
    for i in range(1, len(shape)-1):
        work.step()
        a, b, c = shape[i-1:i+2]
        dx, dy = b[0]-a[0], b[1]-a[1]
        ex, ey = c[0]-b[0], c[1]-b[1]
        long = math.hypot(c[0]-a[0], c[1]-a[1])
        _require(long > EPS, 'CONNECTOR_TURN_UNRESOLVED')
        restrict((i-1, i, i+1), 2*(dx*ey-dy*ex)/(lengths[i-1]*lengths[i]*long))
    along = 0.
    for i in range(len(bounds)):
        work.step()
        if i:
            along += lengths[i-1]
        braking_floor = math.sqrt(max(0., initial*initial-2*limits.max_deceleration_mps2*along))
        bounds[i] = min(bounds[i], max(desired, braking_floor))
    bounds[-1] = 0. if stop_at_goal else min(bounds[-1], desired)
    for i in range(len(bounds)-2, -1, -1):
        work.step()
        bounds[i] = min(bounds[i], math.sqrt(bounds[i+1]**2+2*limits.max_deceleration_mps2*lengths[i]))
    _require(initial <= bounds[0]+EPS, 'INITIAL_SPEED_CANNOT_REACH_CURVATURE_LIMIT')
    speeds, elapsed, points = [initial], 0., []
    for i in range(1, len(bounds)):
        work.step()
        speeds.append(min(bounds[i], math.sqrt(speeds[-1]**2+2*limits.max_acceleration_mps2*lengths[i-1])))
    for i, pose in enumerate(shape):
        work.step()
        if i:
            _require(speeds[i-1]+speeds[i] > EPS, 'ZERO_SPEED_CANNOT_TRAVERSE_CONNECTOR')
            elapsed += 2*lengths[i-1]/(speeds[i-1]+speeds[i])
        points.append(TrajectoryPoint(pose[0], pose[1], speeds[i], pose[2], elapsed))
    first_steer = math.atan(vehicle.wheelbase_m*normalize_angle(shape[1][2]-shape[0][2])/(direction*lengths[0]))
    initial_rate = abs(first_steer-math.atan(vehicle.wheelbase_m*curvature))/(points[1].relative_time*.5)
    _require(initial_rate <= limits.max_steer_rate_rad_s+EPS, 'INITIAL_STEERING_RATE_LIMIT')
    return points


def generate_lane_change(start_pose, initial_speed_mps, initial_curvature_m_inv,
                         target_reference, speed_cap_mps, vehicle, limits,
                         crossing_corridor, coverage, obstacles, search, budget,
                         target_corridor):
    """Return the first certified candidate in explicit deterministic order.

    Independent sweeps and target-body containment share one budget/deadline. An output is
    not a request acceptance or a driving authorization. Replanning/source-lane
    identity and completion belong to the formal behavior integration.
    """
    report = dict(status='invalid', reason_code='INVALID_INPUT', points=[],
                  attempts=[], checks=0, motion_direction=1,
                  authority='nominal_candidate_only')
    work = None
    try:
        _parameters(start_pose, initial_speed_mps, initial_curvature_m_inv, speed_cap_mps,
                    vehicle, limits, search, budget)
        work = _Work(budget); work.step()
        _require(isinstance(crossing_corridor, CrossingCorridor), 'SOURCE_BOUND_CROSSING_CORRIDOR_REQUIRED')
        _require(coverage is not None, 'VISIBILITY_REGION_REQUIRED')
        _require(target_corridor is not None, 'TARGET_LANE_REGION_REQUIRED')
        reference, stations, origin = _reference(target_reference, start_pose,
                                                search.projection_ambiguity_m, vehicle, work)
        for distance in search.distances_m:
            for scale in search.tangent_scales:
                for speed_scale in search.speed_scales:
                    work.step()
                    attempt = dict(distance_m=distance, tangent_scale=scale, speed_scale=speed_scale)
                    try:
                        goal = _goal(reference, stations, origin+distance, work)
                        shape, lengths = _shape(start_pose, goal, initial_curvature_m_inv, scale, search, work)
                        points = _timed(shape, lengths, initial_speed_mps, speed_cap_mps*speed_scale,
                                        speed_cap_mps, initial_curvature_m_inv, vehicle, limits, work)
                        legal = work.validate(points, vehicle, limits, crossing_corridor, obstacles)
                        attempt['legal_report'] = legal
                        if legal['status'] == 'safe':
                            visible = work.validate(points, vehicle, limits, coverage, [])
                            attempt['visibility_report'] = visible
                            if visible['status'] == 'safe':
                                # Geometry-only stationary probe, never inserted
                                # into the moving trajectory or reported as an
                                # actual stop/dwell/settlement observation.
                                end = points[-1]
                                probe = [TrajectoryPoint(end.x,end.y,0.,end.heading,0.),
                                         TrajectoryPoint(end.x,end.y,0.,end.heading,1.)]
                                target_body = work.validate(probe,vehicle,limits,target_corridor,[])
                                attempt['target_pose_report'] = target_body
                                if target_body['status'] == 'safe':
                                    work.step()
                                    report.update(status='safe', reason_code='NOMINAL_LANE_CHANGE_CANDIDATE_READY',
                                                  points=points, target_pose=goal, path_distance_m=sum(lengths),
                                                  legal_report=legal, visibility_report=visible,
                                                  target_pose_report=target_body)
                                    report['attempts'].append(attempt)
                                    return report
                                result = target_body
                            else:
                                result = visible
                        else:
                            result = legal
                        attempt.update(status=result['status'], reason_code=result['reason_code'])
                        if result['reason_code'] in ('BUDGET_EXHAUSTED', 'REGION_PREPARATION_LIMIT'):
                            report['attempts'].append(attempt)
                            report.update(status='inconclusive', reason_code=result['reason_code'])
                            return report
                    except _Failure as error:
                        attempt.update(status=error.status, reason_code=error.code)
                        if error.code == 'BUDGET_EXHAUSTED':
                            report['attempts'].append(attempt)
                            raise
                    report['attempts'].append(attempt)
        report.update(status='inconclusive' if any(a['status']=='inconclusive' for a in report['attempts']) else 'unsafe',
                      reason_code='NO_CERTIFIED_LANE_CHANGE_CANDIDATE')
    except _Failure as error:
        report.update(status=error.status, reason_code=error.code, points=[])
    except (ValueError, TypeError, AttributeError, OverflowError, ZeroDivisionError) as error:
        report.update(status='invalid', reason_code='INVALID_INPUT', details=str(error), points=[])
    finally:
        report['checks'] = work.count if work else 0
    return report


def plan_lane_change_candidate(perception, target_lane_id, initial_curvature_m_inv,
                               speed_cap_mps, vehicle, limits, search, budget,
                               prediction_envelopes, clock=None, geometry_cache=None):
    """Use formal current perception; preserve original Sensor age/deadlines.

    Per-object PredictionEnvelope values are explicit and cover the original
    object set. An observed empty set still requires actual complete visibility.
    Current source lane only: active-lane identity transitions need the behavior
    session integration before this component can be dispatched in production.
    """
    clock = clock or time.monotonic
    try:
        source_lane_id = perception.lane.lane_id
        _require(all(number(v) for v in (perception.ego.vx,perception.ego.vy,perception.ego.speed))
                 and not opposes_direction(perception.ego.vx,perception.ego.vy,
                                            perception.ego.heading,perception.ego.speed),
                 'ACTUAL_MOTION_OPPOSES_FORWARD_LANE_CHANGE')
        _require(target_lane_id != source_lane_id, 'TARGET_ALREADY_CURRENT_LANE')
        scene = read_maneuver_scene(perception, (source_lane_id, target_lane_id), clock, geometry_cache)
        now = clock()
        predictions = _source_predictions(scene['objects'],prediction_envelopes,now)
        facts = ManeuverFacts(perception, clock)
        target = next(v for v in scene['roads'] if v['lane_id']==target_lane_id)
        target_region = CorridorRegion([target['polygon']])
        if geometry_cache is not None:
            _require(geometry_cache.max_entries>=4, 'LANE_CHANGE_CACHE_NEEDS_FOUR_ENTRIES')
            scope = (perception.case_id,perception.task_id,perception.scene_id,scene['map_digest'])
            target_region = geometry_cache.region(scope,('target_body',target_lane_id),[target['polygon']])
        _require(isinstance(budget, ValidationBudget) and number(budget.deadline_monotonic_s),
                 'VALIDATION_BUDGET_INVALID')
        # Convert only the remaining source lifetime to the validator's clock;
        # injected replay clocks may have a different origin. No renewal.
        effective = ValidationBudget(budget.max_checks, budget.max_depth, budget.min_interval_s,
            min(budget.deadline_monotonic_s, time.monotonic()+scene['source_valid_until_s']-clock()))
        result = generate_lane_change((perception.ego.x, perception.ego.y, perception.ego.heading),
            perception.ego.speed, initial_curvature_m_inv, target['center_line'], speed_cap_mps,
            vehicle, limits, scene['legal_crossing_corridor'], scene['coverage'], predictions, search, effective,
            target_region)
        facts.check_current(True)
        _require(perception.lane.lane_id==source_lane_id and facts.map_digest()==scene['map_digest'],
                 'LANE_CHANGE_SOURCE_IDENTITY_CHANGED_DURING_SEARCH')
        _require(clock() < scene['source_valid_until_s'], 'LANE_CHANGE_SOURCE_EXPIRED_DURING_SEARCH')
        result.update(target_lane_id=target_lane_id, source_lane_id=source_lane_id,
                      frame_id=perception.frame_id, source_valid_until_s=scene['source_valid_until_s'],
                      nominal_start_time_s=now, map_digest=scene['map_digest'])
        return result
    except (_Failure, ValueError, TypeError, AttributeError, KeyError, OverflowError) as error:
        return dict(status=error.status if isinstance(error, _Failure) else 'invalid',
                    reason_code=error.code if isinstance(error, _Failure) else str(error),
                    points=[], authority='nominal_candidate_only')
