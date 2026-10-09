"""P03 fixed-entry consumer with an explicitly supplied frame/model snapshot.

The captain supplies a provider after checking its physical sources. This
module cannot certify calibration, steering units or prediction error bounds.
No provider or runtime capability is installed by default. REQUEST_PATH only
acknowledges a nominal path; EXECUTE/SETTLE independently revalidate it.
"""
import copy
import time

from core.behavior_channel import read_channel,capabilities_dict,feedback_dict
from core.behavior_contract import TaskContext,ExecutionFeedback,require,finite
from core.interfaces import Trajectory,DecisionTarget,DecisionMode
from core.validation import validate_output
from core.traffic_quality import signal_stop_requirement
from core.maneuver_facts import ManeuverFacts
from core.geometry import project_polyline,projection_within_polyline,normalize_angle
import math
from members.planning.behavior_contract import assess_behavior_request,LANE_CHANGE_ACTIONS
from members.planning.lane_change_generator import plan_lane_change_candidate
from members.planning.lane_change_generator import _source_predictions
from members.planning.lane_planner import build_trajectory
from members.planning.maneuver_scene import read_maneuver_scene
from members.planning.candidate_validation import validate_candidate,ValidationBudget


class LaneChangePlanningInputs(object):
    """Caller-owned model assertion and current curvature, never inferred.

    All times are process_monotonic. Curvature is signed body curvature in
    inverse metres; positive turns left. The caller must establish its actual
    observation, reference point and error model, including at standstill.
    model_id names that caller's model revision, not proof of calibration.
    Existing VehicleGeometry/MotionLimits/Search/Budget/PredictionEnvelope
    values are reused and independently checked by the existing generator.
    """
    def __init__(self,context,frame_id,produced_at_s,valid_until_s,model_id,
                 initial_curvature_m_inv,vehicle,limits,search,budget,
                 prediction_envelopes,geometry_cache=None,clock_id='process_monotonic'):
        self.context,self.frame_id=context,frame_id
        self.produced_at_s,self.valid_until_s=produced_at_s,valid_until_s
        self.model_id,self.clock_id=model_id,clock_id
        self.initial_curvature_m_inv=initial_curvature_m_inv
        self.vehicle,self.limits,self.search=copy.deepcopy((vehicle,limits,search))
        self.budget,self.prediction_envelopes=copy.deepcopy((budget,prediction_envelopes))
        self.geometry_cache=geometry_cache


class LaneChangeBehaviorPlanner(object):
    def __init__(self,inputs_provider=None,clock=None):
        require(inputs_provider is None or callable(inputs_provider),'invalid lane-change input provider')
        self.provider,self.clock=inputs_provider,clock or time.monotonic
        self.context,self.bindings,self.revisions=None,{},{}

    @staticmethod
    def _revision(request):
        return tuple(request[k] for k in ('revision','stage','source_frame_id','issued_at_s'))

    @staticmethod
    def _binding(request,model,digest):
        pose=request['goal_pose']; lights=request['light_intent']
        return (request['maneuver'],request['source_lane_id'],request['target_lane_id'],
                tuple(pose[k] for k in ('x','y','body_heading_rad','reference_point')),
                tuple(sorted(lights.items())),digest,model.model_id,
                tuple(sorted(vars(model.vehicle).items())),tuple(sorted(vars(model.limits).items())))

    def _inputs(self,p,context):
        require(self.provider is not None,'LANE_CHANGE_MODEL_INPUTS_UNAVAILABLE')
        value=self.provider(p,context)
        require(isinstance(value,LaneChangePlanningInputs),'LANE_CHANGE_MODEL_INPUTS_INVALID')
        now=self.clock()
        require(isinstance(value.context,TaskContext) and value.context.key()==context.key()
                and type(value.frame_id) is int and value.frame_id==p.frame_id
                and value.clock_id=='process_monotonic'
                and finite(value.produced_at_s) and finite(value.valid_until_s)
                and value.produced_at_s<=now<value.valid_until_s<=p.valid_until,
                'LANE_CHANGE_MODEL_SOURCE_MISMATCH_OR_EXPIRED')
        require(isinstance(value.model_id,str) and bool(value.model_id)
                and finite(value.initial_curvature_m_inv),'LANE_CHANGE_MOTION_MODEL_UNAVAILABLE')
        # Detach mutable caller parameters; reuse only the explicit static cache.
        cache=value.geometry_cache
        result=copy.copy(value)
        for name in ('vehicle','limits','search','budget','prediction_envelopes'):
            setattr(result,name,copy.deepcopy(getattr(value,name)))
        result.geometry_cache=cache
        return result

    @staticmethod
    def _reply(context,request,p,now,deadline,status,reason):
        return feedback_dict(ExecutionFeedback(context,request['intent_id'],request['stage'],request['revision'],
            'planning',p.frame_id,p.frame_id,now,deadline,status,usable=now<deadline,
            reason_code=reason,retryable=status=='REJECTED'))

    def _staging(self,p,decision,settings,model,checks,deadline,speed_cap):
        # Stay in the actual current lane until the decision consumes PLANNED.
        # The existing ordinary planner supplies the path, then the same P02
        # body/object/visibility model independently checks it. A straddling
        # recovery cannot silently snap back into either lane through staging.
        if model.geometry_cache is not None:
            require(model.geometry_cache.max_entries>=6,'LANE_CHANGE_STAGING_CACHE_NEEDS_SIX_ENTRIES')
        scene=read_maneuver_scene(p,(p.lane.lane_id,),self.clock,model.geometry_cache)
        deadline=min(deadline,scene['source_valid_until_s'])
        compiled=copy.deepcopy(decision)
        compiled.behavior_request=compiled.behavior_active_identity=None
        compiled.target_lane_id=p.lane.lane_id
        compiled.target_speed=speed_cap
        # The full lane reference can extend outside this observed local road.
        # Stop the staging path while its front still fits the current region;
        # P02 separately checks every body corner and intermediate motion.
        road=scene['roads'][0]; center=road['center_line']
        projection=project_polyline(center,p.ego.x,p.ego.y)
        require(projection_within_polyline(projection,len(center)),
                'LANE_CHANGE_STAGING_SOURCE_PROJECTION_UNAVAILABLE')
        length=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(center,center[1:]))
        compiled.stop_distance=max(0.,length-projection['s']-model.vehicle.front_offset_m
                                   -max(settings.traffic_stop_margin,model.limits.distance_tolerance_m))
        output=build_trajectory(p,compiled,settings)
        require(output.valid and not output.emergency_stop,'LANE_CHANGE_STAGING_PATH_UNAVAILABLE')
        a,b=output.points[:2]
        require(math.hypot(a.x-p.ego.x,a.y-p.ego.y)<=1e-9
                and abs(normalize_angle(a.heading-p.ego.heading))<=1e-9
                and abs(a.speed-p.ego.speed)<=1e-9,'LANE_CHANGE_STAGING_START_MOTION_CHANGED')
        distance=math.hypot(b.x-a.x,b.y-a.y)
        if distance>1e-9:
            first_steer=math.atan(model.vehicle.wheelbase_m*normalize_angle(b.heading-a.heading)/distance)
            current_steer=math.atan(model.vehicle.wheelbase_m*model.initial_curvature_m_inv)
            midpoint=.5*(b.relative_time-a.relative_time)
            require(midpoint>0 and abs(first_steer-current_steer)/midpoint
                    <=model.limits.max_steer_rate_rad_s+1e-9,'LANE_CHANGE_STAGING_INITIAL_STEERING_RATE_LIMIT')
        objects=_source_predictions(scene['objects'],model.prediction_envelopes,self.clock())
        for region,predictions in ((scene['corridor'],objects),(scene['coverage'],[])):
            remaining=model.budget.max_checks-checks
            require(remaining>0,'BUDGET_EXHAUSTED')
            budget=ValidationBudget(remaining,model.budget.max_depth,model.budget.min_interval_s,
                min(model.budget.deadline_monotonic_s,time.monotonic()+deadline-self.clock()))
            report=validate_candidate(output.points,1,model.vehicle,model.limits,region,predictions,budget)
            checks+=report['checks']
            require(report['status']=='safe','LANE_CHANGE_STAGING_'+report['reason_code'])
        require(self.clock()<deadline,'LANE_CHANGE_STAGING_SOURCE_EXPIRED')
        output.valid_until=deadline
        return output,deadline

    def plan(self,p,decision,settings):
        output=Trajectory().bind(decision)
        request=decision.behavior_request
        context=None
        deadline=min(p.valid_until,decision.valid_until)
        try:
            validate_output(decision,DecisionTarget,p)
            now=self.clock()
            context,caps,unused=read_channel(p,now)
            assessment=assess_behavior_request(request,context.to_dict(),p.frame_id,now,
                min(p.valid_until,decision.valid_until),'process_monotonic',decision.behavior_active_identity,
                capabilities=capabilities_dict(caps),supported_actions=LANE_CHANGE_ACTIONS,
                current_lane_id=p.lane.lane_id,frame_usable=p.valid,frame_paused=False,runtime_lane_change=True)
            require(assessment['eligible'],assessment['reason_code'])
            request=assessment['request']
            deadline=min(deadline,request['valid_until_s'],caps.valid_until_s)
            require(decision.mode in (DecisionMode.KEEP_LANE,DecisionMode.FOLLOW)
                    and decision.stop_distance==-1 and not decision.precision_stop
                    and decision.target_speed>0,'LANE_CHANGE_OVERRIDDEN_BY_STOP')
            require(decision.target_lane_id==request['target_lane_id'],'LANE_CHANGE_DECISION_TARGET_MISMATCH')
            require(signal_stop_requirement(p) is None,'LANE_CHANGE_SIGNAL_STOP_REQUIRED')
            if self.context!=context.key():
                self.context,self.bindings,self.revisions=context.key(),{},{}
            intent=request['intent_id']; revision=self._revision(request)
            previous=self.revisions.get(intent)
            require(previous is None or (revision[0]>previous[0] and revision[2]>=previous[2]
                    and revision[3]>=previous[3]) or revision==previous,
                    'REQUEST_REVISION_REGRESSED_OR_CHANGED')
            require(intent in self.revisions or len(self.revisions)<128,'LANE_CHANGE_SESSION_CAPACITY')
            self.revisions[intent]=revision
            model=self._inputs(p,context)
            deadline=min(deadline,model.valid_until_s)
            facts=ManeuverFacts(p,self.clock)
            digest=facts.map_digest()
            binding=self._binding(request,model,digest)
            previous_binding=self.bindings.get(intent)
            require(previous_binding is None or binding==previous_binding,'LANE_CHANGE_ACTIVE_BINDING_CHANGED')
            require(previous_binding is not None or request['stage']=='REQUEST_PATH',
                    'LANE_CHANGE_REQUEST_PATH_REQUIRED')
            side=facts.crossing(request['target_lane_id'],request['source_lane_id'])['side']
            require(request['light_intent']['left_signal']==(side=='left')
                    and request['light_intent']['right_signal']==(side=='right'),
                    'LANE_CHANGE_INDICATOR_SIDE_MISMATCH')
            pose=request['goal_pose']
            model.budget.deadline_monotonic_s=min(model.budget.deadline_monotonic_s,
                time.monotonic()+deadline-self.clock())
            result=plan_lane_change_candidate(p,request['target_lane_id'],model.initial_curvature_m_inv,
                min(decision.target_speed,request['speed_cap_mps']),model.vehicle,model.limits,
                model.search,model.budget,model.prediction_envelopes,self.clock,model.geometry_cache,
                source_lane_id=request['source_lane_id'],
                fixed_goal_pose=tuple(pose[k] for k in ('x','y','body_heading_rad')))
            require(result['status']=='safe',result['reason_code'])
            require(self.clock()<deadline,'LANE_CHANGE_SOURCE_EXPIRED_DURING_DISPATCH')
            validate_output(decision,DecisionTarget,p)
            require(result['frame_id']==p.frame_id and result['map_digest']==digest,
                    'LANE_CHANGE_BINDING_CHANGED_DURING_DISPATCH')
            deadline=min(deadline,result['source_valid_until_s'])
            now=self.clock()
            require(now<deadline,'LANE_CHANGE_SOURCE_EXPIRED_DURING_DISPATCH')
            # Geometry in shared settings, if supplied, must describe this car.
            for name,actual in (('front_offset_m',model.vehicle.front_offset_m),
                ('rear_offset_m',model.vehicle.rear_offset_m),('half_width_m',model.vehicle.half_width_m),
                ('wheelbase_m',model.vehicle.wheelbase_m),('front_steer_max_rad',model.limits.max_front_steer_rad)):
                supplied=getattr(settings,name)
                require(supplied is None or abs(supplied-actual)<=1e-9,'LANE_CHANGE_CONTROL_GEOMETRY_MISMATCH')
            if request['stage']=='REQUEST_PATH':
                output,deadline=self._staging(p,decision,settings,model,result['checks'],deadline,
                    min(decision.target_speed,request['speed_cap_mps']))
                facts.check_current(True)
                validate_output(decision,DecisionTarget,p)
                self.bindings[intent]=binding
                output.left_signal=request['light_intent']['left_signal']
                output.right_signal=request['light_intent']['right_signal']
                output.behavior_feedback=self._reply(context,request,p,self.clock(),deadline,'PLANNED',
                                                     'NOMINAL_LANE_CHANGE_PATH_READY')
                output.reason='LANE_CHANGE_PATH_PREPARED_WAIT_EXECUTE'
                return validate_output(output,Trajectory,decision)
            self.bindings[intent]=binding
            output.valid_until=deadline
            output.behavior_feedback=self._reply(context,request,p,now,deadline,'PLANNED',
                                                 'NOMINAL_LANE_CHANGE_PATH_READY')
            output.points=result['points']
            output.target_speed=min(decision.target_speed,request['speed_cap_mps'])
            output.target_lane_id=request['target_lane_id']
            output.left_signal=request['light_intent']['left_signal']
            output.right_signal=request['light_intent']['right_signal']
            output.behavior_identity={key:request[key] for key in ('intent_id','stage','revision')}
            output.reason='SOURCE_BOUND_NOMINAL_LANE_CHANGE_PATH'
            output.valid=True
            return validate_output(output,Trajectory,decision)
        except (ValueError,TypeError,AttributeError,KeyError,OverflowError) as error:
            output=Trajectory().bind(decision)
            output.reason='BEHAVIOR_REQUEST:'+str(error)
            output.errors.append(output.reason)
            if context is not None and isinstance(request,dict):
                try:
                    output.behavior_feedback=self._reply(context,request,p,self.clock(),deadline,'REJECTED',str(error))
                except (ValueError,TypeError,KeyError):
                    pass
            return output
