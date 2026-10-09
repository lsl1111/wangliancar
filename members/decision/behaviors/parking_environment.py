"""R08 formal-source adapter for the existing D04 policy, SDK-free.

Task poses/access must come from an explicit caller-verified mission. Neither
SDK bay front geometry nor Sensor probability establishes an entry mission,
vehicle heading, legal access or physical prediction error model.
"""
import copy
import math

from core.behavior_contract import GoalPose,require,finite
from core.maneuver_facts import ManeuverFacts
from members.decision.behaviors.observations import Evidence,MotionObject,vehicle_footprint
from members.decision.behaviors.parking import ParkingArea,ParkingFacts,ParkingSpace


class ParkingMission(object):
    """Explicit verified-task snapshot, not inferred from bay/case numbers."""
    def __init__(self,evidence,mission_id,map_digest,space_id,road_lane_ids,access_edge_index,
                 body_heading_rad,approach_pose,position_pose,exit_goal):
        require(isinstance(evidence,Evidence) and isinstance(mission_id,str) and bool(mission_id)
                and isinstance(map_digest,str) and type(space_id) is int and space_id>=0,
                'PARKING_MISSION_IDENTITY_INVALID')
        require(isinstance(road_lane_ids,(tuple,list)) and 1<=len(road_lane_ids)<=3
                and all(isinstance(v,str) and v for v in road_lane_ids)
                and len(set(road_lane_ids))==len(road_lane_ids), 'PARKING_MISSION_ROAD_SET_INVALID')
        require(type(access_edge_index) is int and 0<=access_edge_index<=3
                and finite(body_heading_rad) and all(isinstance(v,GoalPose) for v in
                    (approach_pose,position_pose,exit_goal)), 'PARKING_MISSION_POSES_OR_ACCESS_UNAVAILABLE')
        self.evidence,self.mission_id,self.map_digest=evidence,mission_id,map_digest
        self.space_id,self.road_lane_ids,self.access_edge_index=space_id,tuple(road_lane_ids),access_edge_index
        self.body_heading_rad=body_heading_rad
        self.approach_pose,self.position_pose,self.exit_goal=copy.deepcopy((approach_pose,position_pose,exit_goal))

    def binding(self,bay):
        poses=tuple((v.x,v.y,v.body_heading_rad) for v in (self.approach_pose,self.position_pose,self.exit_goal))
        return (self.mission_id,self.map_digest,self.space_id,self.road_lane_ids,self.access_edge_index,
                self.body_heading_rad,poses,tuple(bay['boundary_knots']),tuple(bay['entrance_edge']),
                self.evidence.source_kind,self.evidence.source,self.evidence.clock_id)


def parking_facts(perception,frame,mission,vehicle_extents,position_uncertainty_m,
                  velocity_uncertainty_mps,geometry_max_checks,clock=None):
    """Preserve original map/task/Sensor ages and independent deadlines.

    vehicle_extents is explicit (front,rear,half_width), metres from rear axle.
    The policy checks that it matches its own car. Error bounds are caller
    assertions which require actual source calibration before production use.
    Only body containment is checked here; P02 still owns future path safety.
    """
    require(frame.context.key()[:3]==(perception.case_id,perception.task_id,perception.scene_id)
            and frame.frame_id==perception.frame_id,'PARKING_DECISION_CONTEXT_MISMATCH')
    facts=ManeuverFacts(perception,clock)
    require(frame.current(facts.check_current()) and not frame.paused,'PARKING_DECISION_FRAME_UNUSABLE')
    require(isinstance(mission,ParkingMission) and mission.evidence.verified(frame,
            allowed_source_kinds=('task','verified_fusion')),'PARKING_TASK_MISSION_UNVERIFIED')
    # Revalidate/detach mutable caller input at every read; an old instance
    # cannot bypass the constructor by replacing a pose, access or road set.
    mission=ParkingMission(mission.evidence,mission.mission_id,mission.map_digest,mission.space_id,
        mission.road_lane_ids,mission.access_edge_index,mission.body_heading_rad,
        mission.approach_pose,mission.position_pose,mission.exit_goal)
    require(mission.map_digest==facts.map_digest(),'PARKING_MISSION_MAP_MISMATCH')
    require(isinstance(vehicle_extents,(tuple,list)) and len(vehicle_extents)==3
            and all(finite(v) and v>0 for v in vehicle_extents),'PARKING_VEHICLE_EXTENTS_UNAVAILABLE')
    require(all(finite(v) and v>=0 for v in (position_uncertainty_m,velocity_uncertainty_mps)),
            'PARKING_OBJECT_ERROR_MODEL_UNAVAILABLE')
    require(all(finite(v) for v in (perception.ego.vx,perception.ego.vy)),'PARKING_ACTUAL_VELOCITY_UNAVAILABLE')
    allowed=[perception.lane.lane_id]+facts.neighbors()
    require(all(v in allowed for v in mission.road_lane_ids),'PARKING_MISSION_ROAD_UNAVAILABLE')
    bay=facts.parking(mission.space_id)
    roads=[facts.road(v,False) for v in mission.road_lane_ids]
    raw=bay['boundary_knots']+[p for road in roads for key in
        ('center_line','left_boundary','right_boundary') for p in road[key]]
    views=facts.coverage(raw)
    require(bool(views),'PARKING_FORMAL_VISIBILITY_UNAVAILABLE')
    deadline=min([frame.valid_until_s,facts.env.valid_until,facts.env.dynamic_valid_until,
        mission.evidence.valid_until_s,bay['evidence']['valid_until_s'],
        facts.env.map_semantics['evidence']['valid_until_s']]
        +[v['geometry_evidence']['valid_until_s'] for v in roads]
        +[v['valid_until_s'] for v in views]+[v['evidence']['valid_until_s'] for v in bay['objects']])
    area=ParkingArea([v['polygon'] for v in roads]+[bay['boundary']],
                     [v['polygon'] for v in views],geometry_max_checks,deadline,clock)
    actual=GoalPose(perception.ego.x,perception.ego.y,perception.ego.heading)
    surrounded=area.contains(vehicle_footprint(actual,*vehicle_extents))
    evidence=Evidence(frame.context,frame.frame_id,facts.env.dynamic_observed_at_s,deadline,
        usable=True,coverage_verified=True,source=perception.target_source,source_kind='verified_fusion')
    space=ParkingSpace(mission.space_id,bay['boundary'],mission.body_heading_rad,'empty',evidence,
                       mission.approach_pose,mission.position_pose)
    objects=[MotionObject(v['id'],v['x'],v['y'],v['vx'],v['vy'],v['length'],v['width'],v['heading'])
             for v in bay['objects']]
    signed=perception.ego.vx*math.cos(perception.ego.heading)+perception.ego.vy*math.sin(perception.ego.heading)
    result=ParkingFacts(evidence,[space],area,objects,surrounded,surrounded,mission.exit_goal,signed,
        frame.observed_at_s-facts.env.dynamic_observed_at_s,position_uncertainty_m,velocity_uncertainty_mps,
        mission.binding(bay)+(position_uncertainty_m,velocity_uncertainty_mps),tuple(vehicle_extents))
    facts.check_current(True)
    require(facts.map_digest()==mission.map_digest,'PARKING_MAP_CHANGED_DURING_READ')
    return result
