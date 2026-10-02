"""Offline decision invariants, using synthetic Perception objects.

No SimOne connection and no captain code path is required: the decision member
is a pure function of the public contract, so every rule can be checked here.
Passing these tests is not evidence of scenario performance.
"""

import copy
import json
import os
import time
import unittest

from core.interfaces import DecisionMode, Perception, Target, TrafficControl
from core.serialization import to_dict
from core.validation import current, validate_output
from members.decision.engine import DecisionEngine
from members.decision.settings import DecisionSettings
from members.decision_stub import decide, reset_decision

FRAME_ID = 7
TIMESTAMP = 4242


def observation(frame_id, x, y, vx, vy, lane_id="lane-1", same_lane_valid=True,
                length=4.0, targets=True):
    """Raw captain snapshot, mirroring the fields the builder publishes."""
    return {
        "gps": {"frame": frame_id, "timestamp": TIMESTAMP, "x": x, "y": y,
                "heading": 0.0, "vx": 0.0, "vy": 0.0, "gear": 1, "age_ms": 0},
        "targets": [{"id": 1, "x": x + 20.0, "y": y, "vx": vx, "vy": vy,
                     "length": length, "probability": 1.0}] if targets else [],
        "targets_valid": True,
        "targets_frame": frame_id, "targets_timestamp": TIMESTAMP, "targets_age_ms": 0,
        "source_status": {"gps": {"read_ok": True, "frame_id": frame_id,
                                  "timestamp": TIMESTAMP, "age_ms": 0,
                                  "quality": "ok", "usable": True},
                          "targets": {"read_ok": True, "frame_id": frame_id,
                                      "timestamp": TIMESTAMP, "age_ms": 0,
                                      "quality": "ok", "usable": True}},
    }


def perception(speed=0.0, targets_valid=True, frame_id=FRAME_ID, ttl=10.0):
    """Minimal usable Perception for behaviour tests."""
    p = Perception()
    p.valid = p.ego.valid = True
    p.frame_id, p.timestamp = frame_id, TIMESTAMP
    p.valid_until = time.monotonic() + ttl
    p.ego.x, p.ego.y, p.ego.heading = 0.0, 0.0, 0.0
    p.ego.speed = p.ego.vx = speed
    p.ego.gear = 1
    p.lane.valid = True
    p.lane.lane_id = "lane-1"
    p.lane.center_line = [(float(x), 0.0) for x in range(0, 401)]
    p.lane.speed_limit = -1.0
    p.lane.lane_width = 3.5
    p.lane.lane_width_valid = True
    p.targets_valid = targets_valid
    p.target_source = "sensor:test"
    p.targets = []
    p.traffic.speed_limit = -1.0
    p.source_status = {
        "gps": {"read_ok": True, "frame_id": frame_id, "timestamp": TIMESTAMP,
                "age_ms": 0, "quality": "ok", "usable": True},
        "targets": {"read_ok": targets_valid, "frame_id": frame_id,
                    "timestamp": TIMESTAMP, "age_ms": 0,
                    "quality": "ok" if targets_valid else "unavailable",
                    "usable": targets_valid},
    }
    return p


def add_target(p, longitudinal=20.0, speed=0.0, length=4.0, in_lane=True,
               same_lane_valid=True, lane_id=None):
    """Attach one lead vehicle ahead on the +x axis (ego heading is 0)."""
    target = Target()
    target.id = 1
    target.valid = True
    target.x = p.ego.x + longitudinal
    target.y = p.ego.y
    target.vx = speed
    target.vy = 0.0
    target.heading = 0.0
    target.length = length
    target.width = 1.8
    target.longitudinal_distance = longitudinal
    target.lateral_distance = 0.0
    target.distance = longitudinal
    target.relative_speed = speed - p.ego.speed
    target.same_lane_valid = same_lane_valid
    target.lane_id = p.lane.lane_id if lane_id is None else lane_id
    target.same_lane = same_lane_valid and target.lane_id == p.lane.lane_id
    p.targets.append(target)
    return target


def add_red_light(p, stop_line_distance, ambiguous=False):
    p.traffic.observed = True
    p.traffic.ambiguous = ambiguous
    p.traffic.signal_state = "RED" if not ambiguous else "UNKNOWN"
    p.traffic.stop_line_distance = stop_line_distance
    p.traffic.valid = not ambiguous
    return p


class DecisionTests(unittest.TestCase):
    def setUp(self):
        reset_decision()
        self.engine = DecisionEngine(DecisionSettings(front_offset_m=3.5))

    def run_engine(self, value, settings=None, require_current=True):
        engine = DecisionEngine(settings) if settings is not None else self.engine
        result = engine.run(value)
        if require_current:
            validate_output(result, type(result), value)
        return result

    # -- degraded perception ---------------------------------------------

    def test_unusable_ego_is_latched_emergency_without_driving_on(self):
        p = perception(speed=5.0)
        p.ego.valid = False
        result = self.run_engine(p)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, result.mode)
        self.assertEqual(0.0, result.target_speed)

    def test_expired_perception_is_emergency(self):
        # The frame link is deliberately inherited from an expired source, so
        # strict source validation is expected to reject the output; the
        # behaviour itself must still be an emergency stop.
        p = perception(speed=5.0, ttl=-1.0)
        result = self.run_engine(p, require_current=False)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, result.mode)
        self.assertEqual(0.0, result.target_speed)
        self.assertFalse(current(result))

    def test_nonfinite_ego_values_are_rejected(self):
        for field in ("x", "y", "heading", "speed"):
            p = perception(speed=5.0)
            setattr(p.ego, field, float("nan"))
            self.assertEqual(DecisionMode.EMERGENCY_BRAKE, self.run_engine(p).mode)

    def test_missing_lane_stops_moving_and_holds_at_rest(self):
        p = perception(speed=4.0)
        p.lane.valid = False
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, self.run_engine(p).mode)
        p = perception(speed=0.0)
        p.lane.valid = False
        held = self.run_engine(p)
        self.assertEqual(DecisionMode.STOP, held.mode)
        self.assertEqual(0.0, held.stop_distance)

    def test_standstill_launches_instead_of_holding_measured_speed(self):
        p = perception(speed=0.0)
        settings = DecisionSettings(cruise_speed=8.0)
        result = self.run_engine(p, settings)
        self.assertEqual(DecisionMode.KEEP_LANE, result.mode)
        self.assertEqual(8.0, result.target_speed)
        self.assertEqual(-1.0, result.stop_distance)
        self.assertEqual("lane-1", result.target_lane_id)
        self.assertTrue(result.reason)

    def test_published_speed_limit_caps_cruise(self):
        p = perception(speed=2.0)
        p.lane.speed_limit = 3.0
        result = self.run_engine(p, DecisionSettings(cruise_speed=8.0))
        self.assertEqual(DecisionMode.KEEP_LANE, result.mode)
        self.assertEqual(3.0, result.target_speed)

    def test_moving_vehicle_is_not_throttled_to_its_measured_speed(self):
        p = perception(speed=2.0)
        result = self.run_engine(p, DecisionSettings(cruise_speed=8.0))
        self.assertEqual(8.0, result.target_speed)

    # -- traffic control -------------------------------------------------

    def test_red_light_stops_before_stop_line(self):
        p = perception(speed=5.0)
        add_red_light(p, 30.0)
        result = self.run_engine(p, DecisionSettings(
            traffic_stop_margin=3.0, front_offset_m=3.5))
        self.assertEqual(DecisionMode.KEEP_LANE, result.mode)
        self.assertGreater(result.target_speed, 0.0)
        self.assertAlmostEqual(23.5, result.stop_distance)

    def test_ambiguous_signal_group_is_not_treated_as_permission(self):
        p = perception(speed=5.0)
        add_red_light(p, 30.0, ambiguous=True)
        result = self.run_engine(p)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, result.mode)
        self.assertEqual(0.0, result.target_speed)
        self.assertIn("TRAFFIC_SOURCE", result.reason)

    def test_unknown_stop_line_position_still_stops(self):
        p = perception(speed=5.0)
        add_red_light(p, -1.0)
        moving = self.run_engine(p)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, moving.mode)
        self.assertEqual(-1.0, moving.stop_distance)
        p = perception(speed=0.0)
        add_red_light(p, -1.0)
        held = self.run_engine(p)
        self.assertEqual(DecisionMode.STOP, held.mode)
        self.assertEqual(0.0, held.stop_distance)

    def test_red_light_uses_nearer_obstacle_stop(self):
        p = perception(speed=2.0)
        add_red_light(p, 30.0)
        add_target(p, longitudinal=15.0, speed=0.0, length=4.0)
        result = self.run_engine(p)
        self.assertEqual(DecisionMode.KEEP_LANE, result.mode)
        self.assertLess(result.stop_distance, 23.5)
        self.assertIn("STOP_TARGET", result.reason)
        self.assertIn("STOP:traffic", result.reason)

    def test_red_light_does_not_hide_emergency_obstacle(self):
        p = perception(speed=6.0)
        add_red_light(p, 30.0)
        add_target(p, longitudinal=5.0, speed=0.0, length=4.0)
        result = self.run_engine(p)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, result.mode)
        self.assertEqual(0.0, result.stop_distance)

    def test_green_signal_does_not_stop(self):
        p = perception(speed=5.0)
        p.traffic.observed = True
        p.traffic.signal_state = "GREEN"
        p.traffic.valid = True
        result = self.run_engine(p)
        self.assertEqual(DecisionMode.KEEP_LANE, result.mode)

    # -- obstacle handling -----------------------------------------------

    def test_close_obstacle_is_emergency_but_a_safe_gap_is_not(self):
        p = perception(speed=5.0)
        add_target(p, longitudinal=5.0, speed=0.0, length=4.0)
        result = self.run_engine(p)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, result.mode)
        self.assertEqual(0.0, result.stop_distance)

        # 7m of clearance with no closing speed is outside the emergency
        # envelope: the behaviour is to follow with a bounded speed demand.
        p = perception(speed=5.0)
        add_target(p, longitudinal=9.0, speed=5.0, length=4.0)
        safe = self.run_engine(p)
        self.assertEqual(DecisionMode.FOLLOW, safe.mode)
        self.assertLessEqual(safe.target_speed, 5.0)

    def test_time_to_collision_emergency_applies_once_moving(self):
        p = perception(speed=12.0)
        add_target(p, longitudinal=24.0, speed=0.0, length=4.0)
        result = self.run_engine(p)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, result.mode)

    def test_time_to_collision_rule_does_not_fire_at_a_standstill(self):
        p = perception(speed=0.0)
        add_target(p, longitudinal=6.0, speed=0.0, length=4.0)
        held = self.run_engine(p)
        self.assertEqual(DecisionMode.STOP, held.mode)
        self.assertEqual(0.0, held.stop_distance)
        self.assertNotEqual(DecisionMode.EMERGENCY_BRAKE, held.mode)

    def test_time_to_collision_emergency_needs_real_closing_speed(self):
        # Same 4.5m clearance, but now closing at speed: this is an emergency.
        p = perception(speed=6.0)
        add_target(p, longitudinal=6.5, speed=0.0, length=4.0)
        result = self.run_engine(p)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, result.mode)

    def test_distant_lead_is_followed_rather_than_stopped(self):
        p = perception(speed=6.0)
        add_target(p, longitudinal=25.0, speed=6.0, length=4.0)
        result = self.run_engine(p)
        self.assertEqual(DecisionMode.FOLLOW, result.mode)
        self.assertGreater(result.target_speed, 0.0)
        self.assertEqual("lane-1", result.target_lane_id)

    def test_following_speed_approaches_distant_lead_and_respects_cruise(self):
        settings = DecisionSettings(cruise_speed=8.0, front_offset_m=3.5)
        p = perception(speed=6.0)
        add_target(p, longitudinal=40.0, speed=6.0)
        far = self.run_engine(p, settings)
        self.assertEqual(DecisionMode.FOLLOW, far.mode)
        self.assertEqual(8.0, far.target_speed)
        p = perception(speed=6.0)
        add_target(p, longitudinal=10.0, speed=6.0)
        near = self.run_engine(p, settings)
        self.assertEqual(DecisionMode.FOLLOW, near.mode)
        self.assertLess(near.target_speed, 6.0)
        p = perception(speed=0.0)
        p.lane.speed_limit = 3.0
        add_target(p, longitudinal=40.0, speed=6.0)
        self.assertEqual(3.0, self.run_engine(p, settings).target_speed)

    def test_lead_ahead_at_standstill_does_not_block_the_launch(self):
        p = perception(speed=0.0)
        add_target(p, longitudinal=30.0, speed=0.0)
        approach = self.run_engine(p)
        self.assertEqual(DecisionMode.KEEP_LANE, approach.mode)
        self.assertGreater(approach.target_speed, 0.0)
        self.assertGreater(approach.stop_distance, 0.0)
        p = perception(speed=0.0, frame_id=FRAME_ID + 1)
        add_target(p, longitudinal=30.0, speed=5.0)
        moving = self.run_engine(p)
        self.assertEqual(DecisionMode.KEEP_LANE, moving.mode)
        self.assertGreater(moving.target_speed, 0.0)

    def test_follow_speed_falls_with_gap_and_obeys_braking_envelope(self):
        settings = DecisionSettings(cruise_speed=8.0, front_offset_m=3.5)
        results = []
        for distance in (30.0, 20.0, 6.0):
            p = perception(speed=0.0)
            add_target(p, longitudinal=distance, speed=0.0)
            results.append(self.run_engine(p, settings))
        self.assertEqual(DecisionMode.KEEP_LANE, results[0].mode)
        self.assertGreater(results[0].target_speed, results[1].target_speed)
        self.assertGreater(results[1].target_speed, 0.0)
        self.assertEqual(DecisionMode.STOP, results[2].mode)
        self.assertEqual(0.0, results[2].target_speed)

    def test_consecutive_frames_reduce_speed_as_stationary_gap_closes(self):
        engine = DecisionEngine(DecisionSettings(cruise_speed=8.0,
                                                 front_offset_m=3.5))
        speeds = []
        modes = []
        stops = []
        for frame, distance in enumerate((30.0, 25.0, 20.0, 15.0, 6.0)):
            p = perception(speed=0.0, frame_id=frame)
            add_target(p, longitudinal=distance, speed=0.0)
            result = engine.run(p)
            speeds.append(result.target_speed)
            modes.append(result.mode)
            stops.append(result.stop_distance)
        self.assertEqual(DecisionMode.KEEP_LANE, modes[0])
        self.assertEqual(DecisionMode.STOP, modes[-1])
        self.assertEqual(sorted(speeds, reverse=True), speeds)
        self.assertEqual(sorted(stops, reverse=True), stops)

    def test_obstacle_without_extent_stops_without_guessing_clearance(self):
        p = perception(speed=0.0)
        add_target(p, longitudinal=30.0, speed=0.0, length=0.0)
        result = self.run_engine(p)
        self.assertEqual(DecisionMode.STOP, result.mode)
        self.assertEqual(0.0, result.stop_distance)
        self.assertIn("TARGET_UNKNOWN", result.reason)

    def test_unknown_extent_never_invents_a_remote_stop_point(self):
        p = perception(speed=3.0)
        add_target(p, longitudinal=30.0, length=0.0)
        moving = self.run_engine(p)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, moving.mode)
        self.assertEqual(-1.0, moving.stop_distance)
        p = perception(speed=0.0)
        add_target(p, longitudinal=2.0, length=0.0,
                   same_lane_valid=False, lane_id="")
        held = self.run_engine(p)
        self.assertEqual(DecisionMode.STOP, held.mode)
        self.assertEqual(0.0, held.stop_distance)

    def test_measured_front_offset_reduces_gap_and_ttc(self):
        p = perception(speed=0.0)
        add_target(p, longitudinal=20.0, speed=0.0)
        short = self.run_engine(p, DecisionSettings(front_offset_m=3.0))
        long = self.run_engine(p, DecisionSettings(front_offset_m=5.0))
        self.assertEqual(DecisionMode.KEEP_LANE, short.mode)
        self.assertEqual(DecisionMode.KEEP_LANE, long.mode)
        self.assertGreater(short.stop_distance, long.stop_distance)
        self.assertGreaterEqual(short.target_speed, long.target_speed)

    def test_lead_selection_uses_clearance_and_never_hides_unknown_extent(self):
        p = perception(speed=0.0)
        small = add_target(p, longitudinal=10.0, length=2.0)
        small.id = 1
        long = add_target(p, longitudinal=12.0, length=10.0)
        long.id = 2
        result = self.run_engine(p)
        self.assertEqual(DecisionMode.KEEP_LANE, result.mode)
        self.assertAlmostEqual(result.stop_distance, 3.0)
        self.assertIn("STOP_TARGET:id=2", result.reason)
        unknown = add_target(p, longitudinal=30.0, length=0.0)
        unknown.id = 3
        held = self.run_engine(p)
        self.assertEqual(DecisionMode.STOP, held.mode)
        self.assertIn("TARGET_UNKNOWN:id=3", held.reason)

    def test_target_behind_the_reference_point_is_not_a_lead(self):
        p = perception(speed=3.0)
        add_target(p, longitudinal=-10.0, speed=3.0, length=4.0)
        result = self.run_engine(p)
        self.assertEqual(DecisionMode.KEEP_LANE, result.mode)

    def test_invalid_target_is_ignored_rather_than_obeyed(self):
        p = perception(speed=3.0)
        target = add_target(p, longitudinal=6.0, speed=0.0)
        target.valid = False
        result = self.run_engine(p)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, result.mode)
        self.assertIn("target record invalid", result.reason)

    def test_unverified_lane_target_alone_does_not_stop(self):
        p = perception(speed=0.0)
        add_target(p, longitudinal=30.0, speed=0.0,
                   same_lane_valid=False, lane_id="")
        result = self.run_engine(p)
        self.assertEqual(DecisionMode.STOP, result.mode)
        self.assertIn("TARGET_CONFLICT", result.reason)

    def test_unverified_lane_target_close_by_stops_and_says_why(self):
        p = perception(speed=0.0)
        add_target(p, longitudinal=5.0, speed=0.0,
                   same_lane_valid=False, lane_id="")
        result = self.run_engine(p)
        self.assertEqual(DecisionMode.STOP, result.mode)
        self.assertEqual(0.0, result.stop_distance)
        self.assertIn("TARGET_CONFLICT", result.reason)

    def test_verified_other_lane_target_is_ignored(self):
        p = perception(speed=5.0)
        target = add_target(p, longitudinal=5.0, speed=0.0,
                            lane_id="lane-2")
        target.y = target.lateral_distance = 4.0
        result = self.run_engine(p)
        self.assertEqual(DecisionMode.KEEP_LANE, result.mode)

    def test_single_bad_target_frame_does_not_trigger_immediate_blind_stop(self):
        p = perception(speed=4.0, targets_valid=False)
        result = self.run_engine(p)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, result.mode)
        self.assertFalse(self.engine._blind_stop)
        self.assertIn("FAULT_PENDING", result.reason)

    def test_bad_target_frame_does_not_accelerate_or_use_stale_target(self):
        p = perception(speed=3.0, targets_valid=False)
        add_target(p, longitudinal=30.0, speed=0.0)
        result = self.run_engine(p)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, result.mode)
        self.assertEqual(0.0, result.target_speed)
        self.assertIn("TARGET_SOURCE", result.reason)

    def test_blind_stop_cannot_be_bypassed_by_stale_target(self):
        for frame in range(2):
            p = perception(speed=3.0, targets_valid=False, frame_id=frame)
            add_target(p, longitudinal=30.0, speed=6.0)
            result = self.run_engine(p)
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, result.mode)
        self.assertIn("STOP_LATCHED", result.reason)

    def test_repeated_target_failure_latches_a_blind_stop(self):
        for frame in range(2):
            p = perception(speed=0.0, targets_valid=False, frame_id=frame)
            result = self.run_engine(p)
        self.assertEqual(DecisionMode.STOP, result.mode)
        self.assertEqual(0.0, result.target_speed)
        self.assertIn("STOP_LATCHED", result.reason)

    def test_blind_stop_is_not_released_while_still_moving(self):
        for frame in range(2):
            self.run_engine(perception(speed=4.0, targets_valid=False, frame_id=frame))
        still_moving = self.run_engine(perception(speed=2.0, frame_id=2))
        self.assertEqual(DecisionMode.EMERGENCY_BRAKE, still_moving.mode)

    def test_blind_stop_releases_after_standstill_and_repeated_good_evidence(self):
        for frame in range(2):
            self.run_engine(perception(speed=4.0, targets_valid=False,
                                       frame_id=frame))
        for frame in (2, 3):
            held = self.run_engine(perception(speed=0.0, frame_id=frame))
            self.assertEqual(DecisionMode.STOP, held.mode)
        released = self.run_engine(perception(speed=0.0, frame_id=4))
        self.assertEqual(DecisionMode.KEEP_LANE, released.mode)

    def test_empty_target_list_is_not_read_as_a_clear_road(self):
        first = perception(speed=0.0, targets_valid=False, frame_id=1)
        self.run_engine(first)
        second = perception(speed=0.0, targets_valid=False, frame_id=2)
        result = self.run_engine(second)
        self.assertEqual(DecisionMode.STOP, result.mode)

    # -- contract hygiene ------------------------------------------------

    def test_inputs_are_not_modified_and_output_is_repeatable(self):
        p = perception(speed=5.0)
        add_target(p, longitudinal=25.0, speed=5.0, length=4.0)
        before = copy.deepcopy(p.__dict__)
        first = self.run_engine(p)
        second = self.run_engine(p)
        self.assertEqual(sorted(before), sorted(p.__dict__))
        self.assertEqual(first.mode, second.mode)
        self.assertEqual(first.target_speed, second.target_speed)
        self.assertEqual(first.stop_distance, second.stop_distance)

    def test_output_is_json_serializable_and_frame_linked(self):
        p = perception(speed=5.0)
        result = self.run_engine(p)
        self.assertEqual(p.frame_id, result.frame_id)
        self.assertEqual(p.timestamp, result.timestamp)
        self.assertEqual(p.valid_until, result.valid_until)
        json.dumps(to_dict(result), allow_nan=False)

    def test_target_speed_is_never_negative_or_nonfinite(self):
        cases = []
        p = perception(speed=0.0)
        cases.append(p)
        p = perception(speed=5.0)
        add_target(p, longitudinal=25.0, speed=0.0, length=4.0)
        cases.append(p)
        p = perception(speed=5.0)
        p.lane.speed_limit = -1.0
        cases.append(p)
        for value in cases:
            result = self.run_engine(value)
            self.assertGreaterEqual(result.target_speed, 0.0)
            self.assertNotEqual(result.target_speed, float("inf"))

    def test_settings_validation_rejects_bad_parameters(self):
        for bad in (DecisionSettings(cruise_speed=0.0),
                    DecisionSettings(min_gap=-1.0),
                    DecisionSettings(time_headway=float("nan")),
                    DecisionSettings(emergency_ttc=0.0),
                    DecisionSettings(resume_margin=0.0)):
            with self.assertRaises(ValueError):
                bad.validate()

    def test_settings_follow_environment_overrides(self):
        environ = {"NEVC_DECISION_CRUISE_SPEED": "11.1",
                   "NEVC_DECISION_TIME_HEADWAY": "1.5"}
        settings = DecisionSettings.from_environment(environ=environ)
        self.assertEqual(11.1, settings.cruise_speed)
        self.assertEqual(1.5, settings.time_headway)
        # Untouched parameters keep their built-in values.
        self.assertEqual(10.5, settings.min_gap)

    def test_shared_vehicle_front_offset_environment_is_used(self):
        settings = DecisionSettings.from_environment(
            environ={"NEVC_VEHICLE_FRONT_OFFSET_M": "3.5"})
        self.assertEqual(3.5, settings.front_offset_m)
        with self.assertRaises(ValueError):
            DecisionSettings.from_environment(
                environ={"NEVC_VEHICLE_FRONT_OFFSET_M": "0"})

    def test_explicit_override_beats_the_environment(self):
        settings = DecisionSettings.from_environment(
            environ={"NEVC_DECISION_CRUISE_SPEED": "11.1"}, cruise_speed=9.0)
        self.assertEqual(9.0, settings.cruise_speed)

    def test_environment_overrides_are_validated_not_ignored(self):
        for environ in ({"NEVC_DECISION_CRUISE_SPEED": "abc"},
                        {"NEVC_DECISION_CRUISE_SPEED": ""},
                        {"NEVC_DECISION_MIN_GAP": "nan"},
                        {"NEVC_DECISION_CRUISE_SPEED": "0"},
                        {"NEVC_DECISION_MIN_GAP": "-2"}):
            with self.assertRaises(ValueError):
                DecisionSettings.from_environment(environ=environ)

    def test_unknown_parameter_name_is_rejected(self):
        with self.assertRaises(ValueError):
            DecisionSettings().replace(no_such_parameter=1.0)

    def test_engine_uses_environment_settings_by_default(self):
        saved = os.environ.get("NEVC_DECISION_CRUISE_SPEED")
        os.environ["NEVC_DECISION_CRUISE_SPEED"] = "3.5"
        try:
            engine = DecisionEngine()
            self.assertEqual(3.5, engine.settings.cruise_speed)
            p = perception(speed=0.0)
            self.assertEqual(3.5, engine.run(p).target_speed)
        finally:
            if saved is None:
                del os.environ["NEVC_DECISION_CRUISE_SPEED"]
            else:
                os.environ["NEVC_DECISION_CRUISE_SPEED"] = saved

    def test_engine_never_raises_on_malformed_perception(self):
        engine = DecisionEngine()
        for value in (None, 42, "text", object()):
            result = engine.run(value)
            self.assertFalse(result.valid)
            self.assertEqual(0.0, result.target_speed)

    # -- fixed entry and planner interop ---------------------------------

    def test_fixed_entry_returns_a_valid_decision(self):
        result = decide(perception(speed=0.0))
        self.assertTrue(result.valid)
        self.assertEqual(DecisionMode.KEEP_LANE, result.mode)
        validate_output(result, type(result), perception(speed=0.0))

    def test_launch_decision_lets_the_planner_build_a_moving_trajectory(self):
        from members.planning_stub import plan
        p = perception(speed=0.0)
        decision = self.run_engine(p, DecisionSettings(cruise_speed=8.0))
        trajectory = plan(p, decision)
        self.assertTrue(trajectory.valid, trajectory.reason)
        self.assertGreater(trajectory.points[1].speed, 0.0)

    def test_stop_decision_is_understood_by_the_planner(self):
        from members.planning_stub import plan
        p = perception(speed=5.0)
        add_red_light(p, 30.0)
        decision = self.run_engine(p)
        trajectory = plan(p, decision)
        self.assertTrue(trajectory.valid, trajectory.reason)
        self.assertTrue(trajectory.stop_required)


if __name__ == "__main__":
    unittest.main()
