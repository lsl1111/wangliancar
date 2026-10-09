"""Captain-owned transport and previous-frame feedback; no member internals."""
import copy
import math
import time
import uuid

from core.behavior_contract import (CONTRACT_VERSION, TaskContext, Capabilities,
                                    ExecutionFeedback, GoalPose, finite, require)

CHANNEL_VERSION = "behavior-channel-v1"


def context_from_dict(value):
    require(isinstance(value,dict) and set(value)==set(TaskContext.__slots__), "invalid channel context")
    return TaskContext(**value)


def capabilities_dict(value):
    result = {key:getattr(value,key) for key in Capabilities.__slots__}
    result["context"],result["actions"] = value.context.to_dict(),sorted(value.actions)
    return result


def feedback_dict(value):
    result = {key:getattr(value,key) for key in ExecutionFeedback.__slots__}
    result["context"] = value.context.to_dict()
    result["actual_pose"] = None if value.actual_pose is None else value.actual_pose.to_dict()
    return result


def feedback_from_dict(value):
    require(isinstance(value,dict) and set(value)==set(ExecutionFeedback.__slots__), "invalid channel feedback")
    data = copy.deepcopy(value)
    data["context"] = context_from_dict(data["context"])
    pose = data["actual_pose"]
    if pose is not None:
        require(isinstance(pose,dict) and set(pose)=={"x","y","body_heading_rad","reference_point"}
                and pose["reference_point"]=="ego_rear_axle", "invalid actual reference")
        data["actual_pose"] = GoalPose(pose["x"],pose["y"],pose["body_heading_rad"])
    return ExecutionFeedback(**data)


def read_channel(perception, now_s):
    """Validate current transport before the existing session checks feedback."""
    channel = perception.behavior_channel
    require(isinstance(channel,dict) and set(channel)=={
        "contract_version","context","frame_id","produced_at_s","valid_until_s","capabilities","feedbacks"},
        "behavior channel unavailable")
    require(channel["contract_version"]==CHANNEL_VERSION and channel["frame_id"]==perception.frame_id
            and finite(channel["produced_at_s"]) and finite(channel["valid_until_s"])
            and channel["produced_at_s"]<=now_s<channel["valid_until_s"]<=perception.valid_until,
            "behavior channel expired or mismatched")
    context = context_from_dict(channel["context"])
    require(context.key()[:3]==(perception.case_id,perception.task_id,perception.scene_id), "behavior context mismatch")
    cap = copy.deepcopy(channel["capabilities"])
    require(isinstance(cap,dict) and set(cap)==set(Capabilities.__slots__)
            and cap["context"]==channel["context"] and isinstance(cap["actions"],list), "invalid channel capability")
    cap["context"] = context
    capabilities = Capabilities(**cap)
    require(isinstance(channel["feedbacks"],list) and len(channel["feedbacks"])<=4, "invalid channel feedback list")
    feedbacks = [feedback_from_dict(v) for v in channel["feedbacks"]]
    return context,capabilities,feedbacks


class BehaviorTransport(object):
    """One-process handoff. Capabilities reflect implemented consumers only.

    Planning acceptance and observed execution are separate records. Reusing
    a previous frame never refreshes its observation/deadline. A send receipt
    can invalidate execution, but can never mark arrival or completion.
    """
    def __init__(self, actions=(), clock=None):
        self.actions,self.clock = tuple(actions),clock or time.monotonic
        self.scope,self.context,self.frame,self.feedbacks = None,None,None,[]
        self._progress = {}
        self._motion_progress = {}

    def prepare(self,p):
        now = self.clock()
        gps = p.source_status.get("gps",{}) if isinstance(p.source_status,dict) else {}
        if (p.valid is not True or p.ego.valid is not True or type(p.frame_id) is not int or p.frame_id<0
                or not isinstance(gps,dict) or gps.get("usable") is not True
                or not finite(p.valid_until) or now>=p.valid_until):
            # An untrusted new task label must not discard an accepted duty.
            p.behavior_channel = {}
            self.feedbacks = []
            return
        scope = (p.case_id,p.task_id,p.scene_id)
        if (scope!=self.scope or (self.frame is not None and p.frame_id<self.frame)):
            self.scope,self.context = scope,TaskContext(*(scope+("runtime:"+uuid.uuid4().hex,)))
            self.feedbacks,self._progress = [],{}
            self._motion_progress = {}
        self.frame = p.frame_id
        caps = Capabilities(self.context,self.actions,now,p.valid_until,usable=bool(self.actions))
        p.behavior_channel = dict(contract_version=CHANNEL_VERSION,context=self.context.to_dict(),
            frame_id=p.frame_id,produced_at_s=now,valid_until_s=p.valid_until,
            capabilities=capabilities_dict(caps),feedbacks=copy.deepcopy(self.feedbacks))

    def observe(self,p,decision,trajectory,control,safety_active=False,receipt=None):
        now = self.clock()
        records = []
        request = decision.behavior_request
        if not isinstance(request,dict) or self.context is None:
            self.feedbacks = []
            return
        if request.get("task_context")!=self.context.to_dict() or request.get("produced_frame_id")!=p.frame_id:
            self.feedbacks = []
            return
        if (not all(isinstance(request.get(k),str) and request[k] for k in ("intent_id","stage"))
                or type(request.get("revision")) is not int or request["revision"]<1
                or not finite(request.get("stop_distance_m")) or not finite(request.get("valid_until_s"))):
            self.feedbacks = []
            return
        if isinstance(trajectory.behavior_feedback,dict):
            records.append(copy.deepcopy(trajectory.behavior_feedback))
        identity = {key:request[key] for key in ("intent_id","stage","revision")}
        usable = bool(p.valid and control.valid and trajectory.valid and now<min(p.valid_until,control.valid_until))
        if usable:
            observation = control.execution_observation
            actual = isinstance(observation,dict) and observation.get("identity")==identity
            failed = safety_active or (isinstance(receipt,dict) and receipt.get("attempted") is True
                                      and receipt.get("ok") is not True)
            if actual or failed:
                key = (request["intent_id"],request["revision"])
                initial = self._progress.setdefault(key,max(0.,request["stop_distance_m"]))
                progress = max(0.,min(1.,1.-max(0.,request["stop_distance_m"])/initial)) if initial>0 else 0.
                if (actual and ((request.get('maneuver') in ('LANE_CHANGE','AVOID','OVERTAKE','MERGE')
                        and request['stage'] in ('EXECUTE','SETTLE')) or
                        (request.get('maneuver')=='PARK' and request['stage'] in
                         ('APPROACH','POSITION','REVERSE_ENTRY','ALIGN','PARKED_DWELL','EXIT_PREPARE','EXIT')))):
                    pose=request.get('goal_pose')
                    if (isinstance(pose,dict) and pose.get('reference_point')=='ego_rear_axle'
                            and all(finite(pose.get(k)) for k in ('x','y','body_heading_rad'))):
                        target=tuple(pose[k] for k in ('x','y','body_heading_rad'))
                        remaining=math.hypot(p.ego.x-target[0],p.ego.y-target[1])
                        previous=self._motion_progress.setdefault(key,(target,remaining,0.))
                        if previous[0]!=target:
                            failed=True
                        else:
                            # Actual GPS distance reduction, not path time or
                            # planner acceptance. Even progress=1 cannot mark
                            # arrival, settled lane, lights or completed dwell.
                            progress=max(previous[2],max(0.,min(1.,1.-remaining/previous[1]))) if previous[1]>0 else 0.
                            self._motion_progress[key]=(target,previous[1],progress)
                        if len(self._motion_progress)>128:
                            self._motion_progress={key:self._motion_progress[key]}
                arrived = bool(actual and observation.get("goal_pose_arrived") is True)
                elapsed = observation.get("standstill_duration_s",0.) if actual else 0.
                completed = bool(actual and observation.get("hold_completed") is True)
                status = ("FAILED" if failed else "COMPLETED" if completed else "DWELLING" if arrived else "EXECUTING")
                reply = ExecutionFeedback(self.context,request["intent_id"],request["stage"],request["revision"],
                    "runtime",p.frame_id,p.frame_id,now,min(p.valid_until,control.valid_until),status,
                    usable=True,reason_code="EXECUTION_INTERRUPTED" if failed else "GPS_CONTROLLER_OBSERVED",
                    progress=progress,actual_standstill_confirmed=bool(arrived and p.ego.speed<=.05 and not failed),
                    goal_pose_arrived=arrived and not failed,standstill_duration_s=elapsed if not failed else 0.,
                    hold_completed=completed and not failed,motion_direction=trajectory.motion_direction,
                    actual_pose=GoalPose(p.ego.x,p.ego.y,p.ego.heading),actual_lane_id=p.lane.lane_id)
                records.append(feedback_dict(reply))
                if len(self._progress)>128:
                    self._progress = {key:initial}
        self.feedbacks = records
