"""Incrementally bind marking intervals to a bounded SDK boundary window.

The nominal geometry is the SDK's piecewise-linear lane sample, in world XYZ.
Results describe this local border only; they never authorize a whole vehicle
trajectory, establish visibility, or manufacture a road/free-space polygon.
"""
import copy
import math
import re
import time
from collections import OrderedDict

from core.geometry import project_polyline,projection_within_polyline
from core.validation import number

MAX_NATIVE_POINTS = 20000
MAX_LOCAL_POINTS = 4096
MAX_ATOMIC_STEPS = 16
MAX_FRAME_SECONDS = .005
SAMPLE_SPACING_M = 1.
ROAD_S_GUARD_M = .25
XY_TOLERANCE_M = .2
Z_TOLERANCE_M = .1


def points3(raw):
    if not isinstance(raw,(tuple,list)) or not 2<=len(raw)<=MAX_NATIVE_POINTS:
        raise ValueError("BOUNDARY_POINT_BUDGET_OR_MISSING")
    result = []
    for point in raw:
        if not isinstance(point,(tuple,list)) or len(point)!=3 or not all(number(v) for v in point):
            raise ValueError("BOUNDARY_COORDINATE_UNAVAILABLE")
        value = tuple(float(v) for v in point)
        if result and math.hypot(value[0]-result[-1][0],value[1]-result[-1][1])<=1e-6:
            if abs(value[2]-result[-1][2])>1e-6:
                raise ValueError("VERTICAL_BOUNDARY_SEGMENT_UNSUPPORTED")
            continue
        result.append(value)
    if len(result)<2:
        raise ValueError("BOUNDARY_DEGENERATE")
    return tuple(result)


def length3(points):
    return sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(points,points[1:]))


def window3(points,start,end):
    """Clip existing segments by their own XY arc length; no extrapolation.

    This coordinate is explicitly boundary arc length, never OpenDRIVE road-s.
    All original vertices within the window and linearly interpolated Z survive.
    """
    if not all(number(v) for v in (start,end)) or not start<end:
        raise ValueError("BOUNDARY_WINDOW_INVALID")
    result,along = [],0.
    start = max(0.,start)
    for a,b in zip(points,points[1:]):
        length = math.hypot(b[0]-a[0],b[1]-a[1])
        low,high = max(start,along),min(end,along+length)
        if high>low+1e-8:
            for at in (low,high):
                ratio = (at-along)/length
                point = tuple(a[i]+ratio*(b[i]-a[i]) for i in range(3))
                if not result or math.hypot(point[0]-result[-1][0],point[1]-result[-1][1])>1e-6:
                    result.append(point)
            if len(result)>MAX_LOCAL_POINTS:
                raise ValueError("LOCAL_BOUNDARY_POINT_BUDGET")
        along += length
        if along>=end:
            break
    if len(result)<2:
        raise ValueError("BOUNDARY_WINDOW_EMPTY")
    return result


def densify3(points):
    result = [points[0]]
    for a,b in zip(points,points[1:]):
        steps = max(1,int(math.ceil(math.hypot(b[0]-a[0],b[1]-a[1])/SAMPLE_SPACING_M)))
        if len(result)+steps>MAX_LOCAL_POINTS:
            raise ValueError("LOCAL_BOUNDARY_POINT_BUDGET")
        for index in range(1,steps+1):
            result.append(tuple(a[i]+index/float(steps)*(b[i]-a[i]) for i in range(3)))
    return result


def _sdk_text(value):
    if hasattr(value,"GetString"):
        value = value.GetString()
    return str(value).lower().split(".")[-1]


def _reason(error):
    value = str(error)
    return value if isinstance(error,ValueError) and re.fullmatch(r"[A-Z_]+",value) else "BOUNDARY_QUERY_FAILED"


def _project3(points,point):
    value = project_polyline(points,point[0],point[1])
    if not projection_within_polyline(value,len(points)):
        raise ValueError("SHARED_BOUNDARY_COVERAGE_MISMATCH")
    i,t = value["index"],value["ratio"]
    z = points[i][2]+t*(points[i+1][2]-points[i][2])
    if value["distance"]>XY_TOLERANCE_M or abs(z-point[2])>Z_TOLERANCE_M:
        raise ValueError("SHARED_BOUNDARY_SHAPE_OR_HEIGHT_MISMATCH")
    return value["s"]


class _Job(object):
    def __init__(self,hdmap,identity,road,section,current,neighbor,native_side,intervals,ego,clock):
        self.clock,self.hdmap = clock,hdmap
        self.current,self.neighbor,self.intervals = current,neighbor,copy.deepcopy(intervals)
        self.identity,self.road,self.section,self.native_side = identity,road,section,native_side
        projection = project_polyline(current,ego.x,ego.y)
        other = project_polyline(neighbor,ego.x,ego.y)
        if not projection_within_polyline(projection,len(current)) or not projection_within_polyline(other,len(neighbor)):
            raise ValueError("EGO_OUTSIDE_SHARED_BOUNDARY")
        # Fixed window while pending: vehicle movement cannot restart a job
        # every frame and prevent it ever finishing. Later windows overlap.
        self.origin_s = projection["s"]
        self.window_start,self.window_end = max(0.,self.origin_s-25.),min(length3(current),self.origin_s+75.)
        self.local = densify3(window3(current,self.window_start,self.window_end))
        # Wider other-side window avoids confusing crop endpoints with an
        # actual mismatch. It is evidence for shape, not a permitted border.
        self.other = window3(neighbor,other["s"]-40.,other["s"]+90.)
        self.result,self.previous,self.reason,self.retry_after = None,None,"BOUNDARY_BINDING_PENDING",0.
        self.iterator = self._build()

    def _build(self):
        # Require bidirectional, ordered coincidence over the chosen window.
        prior = -1.
        for point in self.local:
            at = _project3(self.other,point)
            if at<prior-XY_TOLERANCE_M:
                raise ValueError("SHARED_BOUNDARY_ORDER_AMBIGUOUS")
            prior = at
            yield
        low = _project3(self.other,self.local[0]); high = _project3(self.other,self.local[-1])
        reverse_window = window3(self.other,low,high)
        for point in reverse_window:
            _project3(self.local,point)
            yield
        samples = []
        native_road,native_lane = self.hdmap.pySimString(self.road["id"]),self.hdmap.pySimString(self.identity)
        for point in self.local:
            st = self.hdmap.getRoadST(native_road,self.hdmap.pySimPoint3D(*point))
            if getattr(st,"exists",False) is not True or not number(getattr(st,"s",None)):
                raise ValueError("BOUNDARY_ROAD_S_UNAVAILABLE")
            s = float(st.s)
            if not self.section["start_m"]-1e-6<=s<=self.section["end_m"]+1e-6:
                raise ValueError("BOUNDARY_OUTSIDE_LANE_SECTION")
            yield
            active = next((v for v in self.intervals if v["road_s_start_m"]<=s<v["road_s_end_m"]),None)
            # A section's last point is a map gate, not marking authority.
            if active is None:
                samples.append((point,s,None))
                continue
            observed = self.hdmap.getRoadMark(self.hdmap.pySimPoint3D(*point),native_lane)
            mark = getattr(observed,self.native_side,None)
            offset = getattr(mark,"sOffset",None)
            matched = (getattr(observed,"exists",False) is True and mark is not None and number(offset)
                and _sdk_text(getattr(mark,"type","unknown"))==active["type"]
                and abs(offset-(active["road_s_start_m"]-self.section["start_m"]))<.001
                and active["semantic_verified"] is True)
            if not matched:
                samples.append((point,s,None))
                yield
                continue
            samples.append((point,s,active))
            yield
        deltas = [b[1]-a[1] for a,b in zip(samples,samples[1:])]
        if not deltas or any(abs(v)<=1e-8 for v in deltas) or not (all(v>0 for v in deltas) or all(v<0 for v in deltas)):
            raise ValueError("BOUNDARY_ROAD_S_NONMONOTONIC")
        member = self.section["lanes"][int(self.identity.split("_")[-1])]
        known_direction = member["permission_scope_verified"] and self.road["rule"] in ("RHT","LHT")
        expected_sign = (1 if int(self.identity.split("_")[-1])<0 else -1)*(1 if self.road["rule"]=="RHT" else -1)
        matches_direction = bool(known_direction and deltas[0]*expected_sign>0)
        ranges,run = [],None
        for a,b in zip(samples,samples[1:]):
            active = a[2]
            # Whole nominal border edges are included only within one matched
            # record, inset from transitions. No road-s interpolation or
            # fabricated transition point extends permission across a solid.
            permitted = (matches_direction and active is not None and active==b[2] and active["crossing_permitted"]
                and min(a[1],b[1])>=active["road_s_start_m"]+ROAD_S_GUARD_M
                and max(a[1],b[1])<=active["road_s_end_m"]-ROAD_S_GUARD_M)
            if permitted:
                if run is None:
                    run = dict(shared_boundary_world=[a[0]],road_s_samples_m=[a[1]],marking=copy.deepcopy(active))
                    ranges.append(run)
                run["shared_boundary_world"].append(b[0]); run["road_s_samples_m"].append(b[1])
            else:
                run = None
        return dict(ranges=ranges,shared_boundary_world=list(self.local),
                    road_s_samples_m=[v[1] for v in samples],boundary_arc_start_m=self.window_start,
                    boundary_arc_end_m=self.window_end,verification_model="sdk_piecewise_linear_boundary_v1",
                    complete_marking_coverage=all(v[2] is not None for v in samples[:-1]),
                    travel_direction_verified=known_direction,travel_matches_declared_direction=matches_direction,
                    declared_road_s_direction=expected_sign if known_direction else None,
                    sample_spacing_m=SAMPLE_SPACING_M,road_s_transition_guard_m=ROAD_S_GUARD_M,
                    xy_alignment_tolerance_m=XY_TOLERANCE_M,z_alignment_tolerance_m=Z_TOLERANCE_M)

    def advance(self):
        try:
            next(self.iterator)
        except StopIteration as completed:
            self.result,self.reason = completed.value,"LOCAL_BOUNDARY_RANGE_VERIFIED"
        except Exception as error:
            self.reason = _reason(error)
            self.iterator,self.retry_after = None,self.clock()+.2


class CrossingRanges(object):
    def __init__(self,clock=None):
        self.clock = clock or time.monotonic
        self.jobs = OrderedDict()
        self.cursor = 0

    def clear(self):
        self.jobs.clear()

    def observe(self,ego,lane,items,document,digest,hdmap):
        deadline,steps,pending = self.clock()+MAX_FRAME_SECONDS,0,[]
        for item in items:
            item.update(crossing_ranges_world=[],crossing_boundary_window={},crossing_range_verified=False,
                        crossing_geometry_bound=False)
            if (item.get("geometry_valid") is not True or item.get("shared_boundary_verified") is not True
                    or item.get("same_direction") is not True or item.get("marking_semantics_verified") is not True
                    or str(item.get("lane_type","unknown")).lower().split(".")[-1]!="driving"):
                continue
            try:
                side = item["side"]
                current = points3(lane.left_boundary if side=="left" else lane.right_boundary)
                neighbor = points3(item["right_boundary"] if side=="left" else item["left_boundary"])
                native_side = item.get("marking_observation",{}).get("native_side")
                if native_side not in ("left","right"):
                    raise ValueError("NATIVE_SHARED_MARKING_SIDE_UNAVAILABLE")
                key = (id(hdmap),digest,lane.lane_id,item["lane_id"],native_side,current,neighbor)
                job = self.jobs.get(key)
                at = project_polyline(current,ego.x,ego.y)
                if not projection_within_polyline(at,len(current)):
                    raise ValueError("EGO_OUTSIDE_SHARED_BOUNDARY")
                total = length3(current)
                replace = (job is None or (job.iterator is None and job.result is None and self.clock()>=job.retry_after)
                    or (job is not None and (at["s"]<job.window_start or at["s"]>job.window_end
                        or (job.result is not None and at["s"]>job.window_end-40. and job.window_end<total-1e-6))))
                if replace:
                    previous = job.result if job is not None and job.result is not None else (job.previous if job is not None else None)
                    road,section,unused = document.lane(lane.lane_id)
                    job = _Job(hdmap,lane.lane_id,road,section,current,neighbor,native_side,
                               item["marking_intervals_road_s"],ego,self.clock)
                    job.previous = previous
                    self.jobs[key] = job
                self.jobs.move_to_end(key)
                while len(self.jobs)>8:
                    self.jobs.popitem(last=False)
                pending.append((item,job,at["s"]))
            except (ValueError,TypeError,KeyError,IndexError,AttributeError,OverflowError) as error:
                item["crossing_scope_reason"] = _reason(error)
        # Rotate between both neighbors. One expensive/retrying side cannot
        # monopolize the whole frame or prevent the other side from progressing.
        while pending and steps<MAX_ATOMIC_STEPS and self.clock()<deadline:
            available = [v for v in pending if v[1].result is None and v[1].iterator is not None]
            if not available:
                break
            item,job,at = available[self.cursor%len(available)]
            job.advance(); steps += 1; self.cursor += 1
        for item,job,at in pending:
            item["crossing_scope_reason"] = job.reason
            ready = job.result if job.result is not None else job.previous
            if ready is not None and ready["boundary_arc_start_m"]<=at<=ready["boundary_arc_end_m"]:
                result = copy.deepcopy(ready)
                item.update(crossing_ranges_world=result.pop("ranges"),crossing_boundary_window=result,
                            crossing_range_verified=True,crossing_geometry_bound=True,
                            travel_direction_verified=result["travel_direction_verified"],
                            travel_matches_declared_direction=result["travel_matches_declared_direction"],
                            crossing_allowed_at_ego=bool(item["crossing_allowed_at_ego"] and result["travel_matches_declared_direction"]),
                            marking_scope="verified_local_boundary_fragments")
        return dict(atomic_steps=steps,max_atomic_steps=MAX_ATOMIC_STEPS,time_budget_s=MAX_FRAME_SECONDS)
