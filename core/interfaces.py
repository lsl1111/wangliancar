"""Four-member data contracts.

Keep this module independent from SimOne SDK so every member can import and test it.
Python 3.6 compatible by design.
"""


class DecisionMode(object):
    KEEP_LANE = "KEEP_LANE"
    FOLLOW = "FOLLOW"
    STOP = "STOP"
    EMERGENCY_BRAKE = "EMERGENCY_BRAKE"


class EgoState(object):
    def __init__(self):
        self.frame_id = 0
        self.timestamp = 0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.heading = 0.0
        self.speed = 0.0
        self.acceleration = 0.0
        self.throttle = 0.0
        self.brake = 0.0
        self.steering = 0.0
        self.gear = 0
        self.valid = False
        self.age_ms = 0


class Target(object):
    def __init__(self):
        self.id = -1
        self.type = 0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.length = 0.0
        self.width = 0.0
        self.height = 0.0
        self.distance = -1.0
        self.longitudinal_distance = 0.0
        self.lateral_distance = 0.0
        self.relative_speed = 0.0
        self.same_lane = False
        self.lane_id = ""
        self.ttc = -1.0
        self.valid = False


class LaneContext(object):
    def __init__(self):
        self.lane_id = ""
        self.center_line = []
        self.left_lane_id = ""
        self.right_lane_id = ""
        self.left_mark_type = "UNKNOWN"
        self.right_mark_type = "UNKNOWN"
        self.lane_width = 3.5
        self.heading_error = 0.0
        self.lateral_offset = 0.0
        self.valid = False


class TrafficControl(object):
    def __init__(self):
        self.signal_state = "UNKNOWN"
        self.signal_distance = -1.0
        self.stop_line_distance = -1.0
        self.speed_limit = -1.0
        self.valid = False


class Perception(object):
    """Captain -> decision member."""

    def __init__(self):
        self.ego = EgoState()
        self.targets = []
        self.lane = LaneContext()
        self.traffic = TrafficControl()
        self.scene_id = 0
        self.case_name = ""
        self.frame_id = 0
        self.timestamp = 0
        self.valid = False
        self.errors = []


class DecisionTarget(object):
    """Decision member -> planning member."""

    def __init__(self):
        self.mode = DecisionMode.KEEP_LANE
        self.target_speed = 0.0
        self.target_lane_id = ""
        self.stop_distance = -1.0
        self.reason = ""
        self.valid = False


class TrajectoryPoint(object):
    def __init__(self, x=0.0, y=0.0, speed=0.0, heading=0.0, relative_time=0.0):
        self.x = float(x)
        self.y = float(y)
        self.speed = float(speed)
        self.heading = float(heading)
        self.relative_time = float(relative_time)


class Trajectory(object):
    """Planning member -> control member."""

    def __init__(self):
        self.points = []
        self.target_speed = 0.0
        self.emergency_stop = False
        self.valid = False


class ControlOut(object):
    """Control member -> captain/runtime -> SimOne API."""

    def __init__(self):
        self.throttle = 0.0
        self.brake = 0.0
        self.steering = 0.0
        self.gear = 1
        self.handbrake = False
        self.left_signal = False
        self.right_signal = False
        self.hazard_signal = False
        self.valid = False
        self.source = ""

    def clamp(self):
        self.throttle = max(0.0, min(1.0, float(self.throttle)))
        self.brake = max(0.0, min(1.0, float(self.brake)))
        self.steering = max(-1.0, min(1.0, float(self.steering)))
        return self

