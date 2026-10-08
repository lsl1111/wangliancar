"""Normalize existing map/Sensor facts for maneuver consumers, not behaviors.

No SDK calls, guessed lane/parking targets, right-of-way or free-space hulls.
The legacy perception and safety contracts remain independent of this channel.
"""
import copy
import math
import time

from core.geometry import project_polyline, swept_path_distance
from core.interfaces import ManeuverEnvironment
from core.region_geometry import convex_polygon, inside, intersects, hull, rectangle, circle_intersects
from core.validation import number
from perception.sensor_visibility import SensorVisibility
from perception.traffic_geometry import signal_reference, stop_line_position


def _observed_objects(perception):
    meta = perception.source_status.get("targets",{})
    source = perception.target_source
    if (not perception.targets_valid or not isinstance(meta,dict) or meta.get("usable") is not True
            or not isinstance(source,str) or not source.startswith("sensor:") or not source[7:]):
        return [],False,"formal_targets_unavailable"
    objects,ids,complete = [],set(),True
    for target in perception.targets:
        fields = (target.x,target.y,target.z,target.vx,target.vy,target.length,target.width)
        if (target.valid is not True or type(target.id) is not int or target.id<0 or target.id in ids
                or target.source != source or not number(target.probability) or not .05<=target.probability<=1
                or not all(number(v) for v in fields) or min(target.length,target.width)<=0):
            complete = False
            continue
        ids.add(target.id)
        objects.append(dict(id=target.id,type=target.type,x=target.x,y=target.y,z=target.z,
            vx=target.vx,vy=target.vy,length=target.length,width=target.width,
            heading=target.heading if number(target.heading) else None,
            source=source,source_frame_id=perception.targets_frame_id,
            probability=target.probability))
    return objects,complete,"observed" if complete else "object_geometry_or_identity_unknown"


def _touches(obj, region, horizon=0.0):
    end = (obj["x"]+obj["vx"]*horizon,obj["y"]+obj["vy"]*horizon)
    if obj["heading"] is None:
        radius = .5*math.hypot(obj["length"],obj["width"])
        return (circle_intersects(region,obj["x"],obj["y"],radius)
            or circle_intersects(region,end[0],end[1],radius)
            or swept_path_distance(region+region[:1],(obj["x"],obj["y"]),end)<=radius)
    body = rectangle(obj["x"],obj["y"],obj["heading"],obj["length"]/2,obj["length"]/2,obj["width"]/2)
    if horizon:
        body = hull(body+rectangle(end[0],end[1],obj["heading"],obj["length"]/2,obj["length"]/2,obj["width"]/2))
    return intersects(body,region)


def _covered(region, coverage):
    # One matched source must cover the full region; do not invent a union or
    # fuse incompatible packets simply because several sensors are configured.
    return any(all(inside(p,record["polygon"]) for p in region) for record in coverage)


def _same_plane(raw, z):
    # Current visibility certificate is planar. Never project another level
    # of a multi-layer map into an empty region on the observed level.
    return bool(raw and all(isinstance(v,(tuple,list)) and len(v)>=3 and number(v[2])
                           and abs(v[2]-z)<=.1 for v in raw))


def _evidence(environment, source, source_kind, verified=False):
    dynamic = source_kind in ("sensor","verified_fusion")
    return dict(frame_id=environment.frame_id,
        observed_at_s=environment.dynamic_observed_at_s if dynamic else environment.observed_at_s,
        valid_until_s=environment.dynamic_valid_until if dynamic else environment.valid_until,
        source=source,source_kind=source_kind,
        coverage_verified=verified,clock_id="process_monotonic")


class ManeuverEnvironmentBuilder(object):
    def __init__(self, route_manager, config=None, clock=None):
        self.route_manager,self.clock = route_manager,clock or time.monotonic
        self.visibility = SensorVisibility(getattr(config,"sensor_visibility_file",""))
        self.vehicle_id = getattr(config,"vehicle_id","0")
        self.sensor_timeout_ms = getattr(config,"sensor_timeout_ms",500)

    def build(self, perception):
        env = ManeuverEnvironment().bind(perception)
        env.case_id,env.task_id,env.scene_id = perception.case_id,perception.task_id,perception.scene_id
        now = self.clock()
        gps = perception.source_status.get("gps",{})
        gps_age = gps.get("age_ms",-1) if isinstance(gps,dict) else -1
        if (perception.valid is not True or not perception.ego.valid
                or type(perception.frame_id) is not int or perception.frame_id<0
                or not all(number(getattr(perception.ego,k)) for k in ("x","y","z","heading"))
                or not isinstance(gps,dict) or gps.get("usable") is not True
                or not number(gps_age) or gps_age<0 or now>=env.valid_until):
            env.status["frame"] = "gps_or_frame_unusable"
            return env
        env.observed_at_s = now-gps_age/1000.
        env.valid = True
        env.objects,complete,reason = _observed_objects(perception)
        target_meta = perception.source_status.get("targets",{})
        age = target_meta.get("age_ms",-1) if isinstance(target_meta,dict) else -1
        if (not number(age) or not 0<=age<self.sensor_timeout_ms
                or type(perception.targets_frame_id) is not int or perception.targets_frame_id<0
                or perception.targets_frame_id>perception.frame_id):
            complete = False
            env.objects = []
            reason = "target_source_time_unknown_or_invalid"
        else:
            env.dynamic_observed_at_s = now-max(gps_age,age)/1000.
            env.dynamic_valid_until = min(env.valid_until,now+(self.sensor_timeout_ms-age)/1000.)
        env.status["objects"] = dict(usable=complete,reason=reason,source=perception.target_source)
        for obj in env.objects:
            obj["evidence"] = _evidence(env,perception.target_source,"sensor",False)
        coverage,coverage_reason = self.visibility.regions(perception,self.vehicle_id) if complete else ([],reason)
        if perception.targets_frame_id != perception.frame_id:
            # A current GPS pose cannot locate a body-frame visibility model
            # observed on an older Sensor frame. No pose-history producer yet.
            coverage,coverage_reason = [],"sensor_pose_history_unavailable"
        if coverage:
            # Retain original source observation time, never rejuvenate it with
            # the new GPS frame. A regressed/unsynchronized stream is rejected
            # upstream and cannot create a fresh visibility certificate here.
            observed = now-max(gps_age,age)/1000.
            for record in coverage:
                record.update(_evidence(env,perception.target_source,"sensor",True))
                record["observed_at_s"] = observed
                record["source_frame_id"] = perception.targets_frame_id
            env.coverage_regions = coverage
        env.status["coverage"] = dict(verified=bool(coverage),reason=coverage_reason)
        try:
            if hasattr(self.route_manager,"read_neighbor_lanes"):
                env.neighbor_lanes = self.route_manager.read_neighbor_lanes(perception.ego,perception.lane)
            for lane in env.neighbor_lanes:
                outline = lane["left_boundary"]+list(reversed(lane["right_boundary"]))
                verified = bool(lane["geometry_valid"] and outline and complete and _covered(outline,coverage)
                    and _same_plane(lane["center_line"]+lane["left_boundary"]+lane["right_boundary"],perception.ego.z))
                lane["dynamic_coverage_verified"] = verified
                lane["objects"] = copy.deepcopy(env.objects)
                lane["evidence"] = _evidence(env,perception.target_source if verified else "hdmap",
                                              "verified_fusion" if verified else "map",verified)
            env.status["neighbor_lanes"] = "observed" if env.neighbor_lanes else "no_neighbor_geometry"
        except Exception as exc:
            env.neighbor_lanes = []
            env.status["neighbor_lanes"] = "query_failed:"+type(exc).__name__
        if hasattr(self.route_manager,"read_maneuver_map"):
            try:
                env.map_semantics,env.junctions = self.route_manager.read_maneuver_map(
                    perception.ego,perception.lane,env.neighbor_lanes)
                map_source = "sdk_matched_opendrive" if env.map_semantics.get("semantic_verified") is True else "unavailable"
                env.map_semantics["evidence"] = _evidence(env,map_source,"map",False)
                for neighbor in env.neighbor_lanes:
                    if neighbor.get("marking_map_digest"):
                        neighbor["marking_evidence"] = _evidence(env,"sdk_matched_opendrive","map",False)
                for junction in env.junctions:
                    junction["evidence"] = _evidence(env,"sdk_matched_opendrive","map",False)
            except Exception as exc:
                env.map_semantics = dict(verified=False,reason="MAP_SEMANTICS_QUERY:"+type(exc).__name__)
                env.junctions = []
        self._parking(perception,env,complete,coverage)
        self._crossings(perception,env,complete,coverage)
        return env

    def _parking(self,p,env,complete,coverage):
        env.status["parking_spaces"] = "observed" if p.parking_spaces_valid else "map_geometry_unusable"
        if not p.parking_spaces_valid:
            return
        for record in p.parking_spaces:
            item = copy.deepcopy(record)
            item.update(occupancy="unknown",occupancy_verified=False,blocking_object_ids=[],
                        geometry_valid=False,reason="geometry_unavailable")
            try:
                region = convex_polygon([tuple(v[:2]) for v in record["boundary_knots"]])
                item.update(boundary=region,geometry_valid=True)
                hits = [obj["id"] for obj in env.objects if _touches(obj,region)]
                verified = complete and _covered(region,coverage) and _same_plane(record["boundary_knots"],p.ego.z)
                # Observing an obstacle proves occupancy without claiming the
                # remainder of the space was observed. Empty needs full coverage.
                if hits:
                    item.update(occupancy="occupied",occupancy_verified=True,
                        blocking_object_ids=hits,reason="observed_object_intersection")
                elif verified:
                    item.update(occupancy="empty",occupancy_verified=True,reason="verified_region_clear")
                    env.free_regions.append(dict(kind="parking",id=item["id"],polygon=region,
                        evidence=_evidence(env,p.target_source,"verified_fusion",True)))
                else:
                    item["reason"] = "visibility_or_object_geometry_unknown"
                item["coverage_verified"] = verified
                item["evidence"] = _evidence(env,p.target_source,"verified_fusion",verified)
            except (TypeError,ValueError,KeyError,IndexError) as exc:
                item["reason"] = "parking_geometry_invalid:"+type(exc).__name__
            env.parking_spaces.append(item)

    def _crossings(self,p,env,complete,coverage):
        env.status["crossing_regions"] = dict(geometry_scope="signal_associated_lanes",
            coverage_complete=False,right_of_way="UNKNOWN",exit_geometry="unavailable")
        if not p.map_crosswalks_valid:
            return
        reference,identities = signal_reference(p.lane)
        origin = project_polyline(reference,p.ego.x,p.ego.y) if reference else None
        for record in p.map_crosswalks:
            item = dict(id="crosswalk:"+str(record.get("id")),kind="crosswalk",polygon=[],
                geometry_valid=False,occupancy="unknown",coverage_verified=False,
                object_ids=[],predicted_object_ids=[],prediction_horizon_s=3.,
                entry_stop_line=None,entry_distance_m=None,right_of_way="UNKNOWN",
                rule_verified=False,exit_region=None,exit_coverage_verified=False)
            try:
                region = convex_polygon([tuple(v[:2]) for v in record["boundary_knots"]])
                lanes = record.get("lane_ids",[])
                if not any(identity in lanes for identity in identities):
                    raise ValueError("crosswalk outside selected route")
                item.update(polygon=region,geometry_valid=True)
                item["object_ids"] = [o["id"] for o in env.objects if _touches(o,region)]
                item["predicted_object_ids"] = [o["id"] for o in env.objects if _touches(o,region,3.)]
                verified = complete and _covered(region,coverage) and _same_plane(record["boundary_knots"],p.ego.z)
                item["coverage_verified"] = verified
                item["occupancy"] = ("occupied" if item["object_ids"] else "clear" if verified else "unknown")
                item["evidence"] = _evidence(env,p.target_source,"verified_fusion",verified)
                # Match association identity and full line/route intersection;
                # a nearby line for another lamp is never silently selected.
                association = set(record.get("association_signal_ids",[]))
                stops = []
                near = min((project_polyline(reference,*point)["s"] for point in region),default=None) if reference else None
                if origin and p.map_stop_lines_valid:
                    for stop in p.map_stop_lines:
                        if (not association.intersection(stop.get("association_signal_ids",[]))
                                or not any(identity in stop.get("lane_ids",[]) for identity in lanes)):
                            continue
                        s,_ = stop_line_position(reference,{"stop_line_boundary":stop["boundary_knots"]},p.lane.lane_width)
                        if s is not None and near is not None and origin["s"]-1e-6<=s<=near+1e-6:
                            stops.append((s,stop))
                if stops:
                    s,stop = max(stops,key=lambda pair:pair[0])
                    tied = [v for v in stops if abs(v[0]-s)<.01]
                    if all(v[1]["boundary_knots"]==stop["boundary_knots"] for v in tied):
                        item["entry_stop_line"] = copy.deepcopy(stop)
                        item["entry_distance_m"] = s-origin["s"]
            except (ValueError,TypeError,KeyError,IndexError) as exc:
                item["reason"] = "crossing_geometry_invalid:"+type(exc).__name__
            env.crossing_regions.append(item)
