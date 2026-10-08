"""Synthetic P01 draft contract tests; no decision-private imports or SDK."""

import copy
import json
import os
import unittest

from members.planning.behavior_contract import CONTRACT_VERSION, assess_behavior_request


class BehaviorContractTests(unittest.TestCase):
    def setUp(self):
        self.context = dict(case_id="synthetic", task_id="trial", scene_id=0, session_id="session-a")
        self.identity = dict(intent_id="intent-a", stage="APPROACH", revision=2,
                             source_frame_id=90, issued_at_s=9.0)
        self.request = dict(
            contract_version=CONTRACT_VERSION, interface_status="proposal_not_runtime_connected",
            task_context=copy.deepcopy(self.context), intent_id="intent-a", maneuver="KEEP_LANE",
            stage="APPROACH", revision=2, source_frame_id=90, produced_frame_id=100,
            produced_at_s=10.0, issued_at_s=9.0, valid_until_s=10.2, clock_id="trial-clock",
            dispatch_allowed=True, status="REQUESTED", reason_code="INTENT_CREATED", attempt=0,
            target_lane_id="lane-a", parking_space_id=None, goal_pose=None, motion_direction=1,
            speed_cap_mps=5.0, stop_distance_m=-1.0, precision_stop=False,
            minimum_standstill_duration_s=0.0, parking_brake_at_stop=False,
            light_intent=dict(left_signal=False, right_signal=False, hazard_signal=False),
            stop_obligation_id=None)
        self.capabilities = dict(context=copy.deepcopy(self.context),
                                 actions=["PATH", "PATH_STOP"],
                                 produced_at_s=9.9, valid_until_s=10.2, clock_id="trial-clock",
                                 usable=True, version=CONTRACT_VERSION)
        self.arguments = dict(context=self.context, frame_id=100, now_s=10.0, valid_until_s=10.2,
                              clock_id="trial-clock", active_identity=self.identity,
                              capabilities=self.capabilities, supported_actions=("KEEP_LANE", "FOLLOW", "STOP"),
                              current_lane_id="lane-a", frame_usable=True, frame_paused=False)

    def assess(self, **changes):
        arguments = dict(self.arguments)
        arguments.update(changes)
        return assess_behavior_request(self.request, **arguments)

    def rejected(self, reason, **changes):
        result = self.assess(**changes)
        self.assertFalse(result["eligible"])
        self.assertEqual(result["status"], "REJECTED")
        self.assertEqual(result["reason_code"], reason)
        self.assertIsNone(result["request"])
        self.assertFalse(result["production_connected"])
        return result

    def test_refresh_preserves_earlier_creation_frame_and_time(self):
        result = self.assess()
        self.assertTrue(result["eligible"])
        self.assertEqual(result["status"], "READY_FOR_COMPONENT")
        self.assertEqual(result["request"]["source_frame_id"], 90)
        self.assertEqual(result["request"]["issued_at_s"], 9.0)
        self.assertFalse(result["production_connected"])
        self.assertNotIn("actual_pose", result)
        self.assertNotIn("goal_pose_arrived", result)
        result["request"]["light_intent"]["hazard_signal"] = True
        self.assertFalse(self.request["light_intent"]["hazard_signal"])

    def test_explicit_local_enablement_and_current_quality_required(self):
        self.rejected("LOCAL_ACTION_DISABLED", supported_actions=())
        self.rejected("CURRENT_FRAME_UNUSABLE", frame_usable=False)
        self.rejected("CURRENT_FRAME_UNUSABLE", frame_paused=True)
        result = assess_behavior_request(self.request, self.context, 100, 10.0, 10.2,
                                        "trial-clock", self.identity, self.capabilities)
        self.assertFalse(result["eligible"])

    def test_shared_synthetic_example_remains_unconnected_and_disabled(self):
        path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "members", "planning",
                            "examples", "behavior_request.synthetic.json")
        with open(path, encoding="utf-8") as source:
            sample = json.load(source)
        self.assertTrue(sample["synthetic"])
        self.assertFalse(sample["production_connected"])
        request, arguments = sample["request"], sample["call_arguments"]
        result = assess_behavior_request(request, **arguments)
        for key, expected in sample["expected_result"].items():
            self.assertEqual(result[key], expected)
        request["dispatch_allowed"] = True
        self.assertEqual(assess_behavior_request(request, **arguments)["reason_code"], "LOCAL_ACTION_DISABLED")
        arguments["supported_actions"] = ["KEEP_LANE"]
        self.assertEqual(assess_behavior_request(request, **arguments)["reason_code"], "CAPABILITY_NOT_CURRENT")
        arguments["capabilities"]["usable"] = True
        self.assertEqual(assess_behavior_request(request, **arguments)["reason_code"], "CAPABILITY_ACTIONS_MISSING")
        arguments["capabilities"]["actions"] = ["PATH"]
        result = assess_behavior_request(request, **arguments)
        self.assertEqual(result["status"], "READY_FOR_COMPONENT")
        self.assertFalse(result["production_connected"])

    def test_context_and_identity_must_match_explicit_current_values(self):
        for key, value in (("case_id", "other"), ("task_id", "other"),
                           ("session_id", "other"), ("scene_id", 1)):
            with self.subTest(key=key):
                self.request["task_context"][key] = value
                self.rejected("REQUEST_CONTEXT_MISMATCH")
                self.request["task_context"] = copy.deepcopy(self.context)
        for key in ("intent_id", "stage"):
            with self.subTest(key=key):
                original = self.request[key]
                self.request[key] = "other"
                self.rejected("REQUEST_IDENTITY_MISMATCH")
                self.request[key] = original

    def test_old_and_future_revision_rejected(self):
        for value in (1, 3):
            self.request["revision"] = value
            self.rejected("REQUEST_REVISION_MISMATCH")
        self.request["revision"] = True
        self.rejected("REQUEST_REVISION_INVALID")

    def test_created_source_and_produced_frame_have_distinct_meanings(self):
        for field, value in (("produced_frame_id", 99), ("produced_frame_id", 101),
                             ("source_frame_id", 101), ("source_frame_id", 89),
                             ("source_frame_id", True)):
            with self.subTest(field=field, value=value):
                original = self.request[field]
                self.request[field] = value
                self.rejected("REQUEST_FRAME")
                self.request[field] = original

    def test_clock_and_remaining_lifetime_cannot_be_rebound(self):
        for field, value in (("clock_id", "other-clock"), ("issued_at_s", 9.1),
                             ("produced_at_s", 10.1), ("produced_at_s", 8.9),
                             ("valid_until_s", 10.0), ("valid_until_s", 10.3)):
            with self.subTest(field=field, value=value):
                original = self.request[field]
                self.request[field] = value
                self.rejected("REQUEST_TIME")
                self.request[field] = original
        self.rejected("CURRENT_FRAME_INVALID", valid_until_s=10.0)

    def test_actual_check_time_includes_processing_delay(self):
        self.request["produced_at_s"] = 9.99
        self.assertTrue(self.assess()["eligible"])
        self.request["valid_until_s"] = 10.1
        self.rejected("REQUEST_TIME", now_s=10.1)
        self.rejected("CURRENT_FRAME_INVALID", now_s=10.2)
        self.request["valid_until_s"] = 10.2
        self.capabilities["valid_until_s"] = 10.1
        self.rejected("CAPABILITY_NOT_CURRENT", now_s=10.1)

    def test_flat_complete_versioned_schema_required(self):
        for key in ("speed_cap_mps", "light_intent", "stop_distance_m", "produced_frame_id"):
            original = self.request.pop(key)
            self.rejected("REQUEST_SCHEMA")
            self.request[key] = original
        self.request["goal"] = {}
        self.rejected("REQUEST_SCHEMA")
        del self.request["goal"]
        self.request["contract_version"] = "unapproved-v2"
        self.rejected("REQUEST_VERSION")

    def test_non_dispatchable_or_terminal_requests_never_accepted(self):
        for status in ("REQUESTED", "ACCEPTED", "EXECUTING"):
            self.request["status"] = status
            self.assertTrue(self.assess()["eligible"])
        self.request["dispatch_allowed"] = False
        self.rejected("REQUEST_NOT_DISPATCHABLE")
        self.request["dispatch_allowed"] = True
        for status in ("PROPOSED", "BLOCKED", "SUSPENDED", "RETRY_WAIT", "COMPLETED", "CANCELLED", "unknown"):
            self.request["status"] = status
            self.rejected("REQUEST_NOT_DISPATCHABLE")

    def test_capability_identity_validity_and_requirements(self):
        self.rejected("CAPABILITIES_UNAVAILABLE", capabilities=None)
        self.rejected("CAPABILITY_SCHEMA", capabilities={})
        for field, value, reason in (
                ("version", "other", "CAPABILITY_IDENTITY"),
                ("clock_id", "other", "CAPABILITY_IDENTITY"),
                ("usable", False, "CAPABILITY_NOT_CURRENT"),
                ("produced_at_s", 10.1, "CAPABILITY_NOT_CURRENT"),
                ("valid_until_s", 10.0, "CAPABILITY_NOT_CURRENT")):
            with self.subTest(field=field):
                original = self.capabilities[field]
                self.capabilities[field] = value
                self.rejected(reason)
                self.capabilities[field] = original
        self.capabilities["context"]["session_id"] = "other"
        self.rejected("CAPABILITY_IDENTITY")
        self.capabilities["context"] = copy.deepcopy(self.context)
        self.capabilities["actions"] = ["PATH_STOP"]
        result = self.rejected("CAPABILITY_ACTIONS_MISSING")
        self.assertEqual(result["missing_capabilities"], ["PATH"])
        self.request.update(maneuver="STOP", stop_distance_m=0.0)
        self.capabilities["actions"] = ["PATH"]
        self.assertEqual(self.rejected("CAPABILITY_ACTIONS_MISSING")["missing_capabilities"], ["PATH_STOP"])

    def test_new_maneuvers_cannot_be_enabled_by_capability_claim(self):
        self.request["maneuver"] = "LANE_CHANGE"
        self.capabilities["actions"].append("LANE_CHANGE")
        self.rejected("UNSUPPORTED_MANEUVER")
        self.rejected("LOCAL_ACTIONS_UNSUPPORTED", supported_actions=("LANE_CHANGE",))

    def test_direction_and_noncurrent_targets_are_not_silently_changed(self):
        for field, value, reason in (
                ("motion_direction", -1, "UNSUPPORTED_DIRECTION"),
                ("target_lane_id", "lane-b", "UNSUPPORTED_TARGET_LANE"),
                ("parking_space_id", 4, "UNSUPPORTED_POSE_OR_PARKING_TARGET"),
                ("goal_pose", dict(x=1.0, y=2.0, body_heading_rad=0.0,
                                  reference_point="ego_rear_axle"), "UNSUPPORTED_POSE_OR_PARKING_TARGET")):
            with self.subTest(field=field):
                original = self.request[field]
                self.request[field] = value
                self.rejected(reason)
                self.assertEqual(self.request[field], value)
                self.request[field] = original
        self.rejected("CURRENT_LANE_UNKNOWN", current_lane_id="")

    def test_unknown_stop_is_not_defaulted_to_current_position(self):
        self.request["maneuver"] = "STOP"
        self.rejected("STOP_DISTANCE_UNKNOWN")
        self.request["stop_distance_m"] = 8.0
        self.request["speed_cap_mps"] = 0.0
        self.assertTrue(self.assess()["eligible"])
        self.request["precision_stop"] = True
        self.rejected("PRECISION_APPROACH_SPEED_UNAVAILABLE")
        self.request["speed_cap_mps"] = 1.0
        self.assertTrue(self.assess()["eligible"])

    def test_unconnected_dwell_brake_and_lights_remain_unsupported(self):
        self.request["stop_distance_m"] = 0.0
        self.request["stop_obligation_id"] = "stop-identity-not-transmitted"
        self.rejected("UNSUPPORTED_STOP_OBLIGATION")
        self.request["stop_obligation_id"] = None
        for field, value in (("minimum_standstill_duration_s", 3.0), ("parking_brake_at_stop", True)):
            original = self.request[field]
            self.request[field] = value
            self.rejected("UNSUPPORTED_STOP_EXTENSIONS")
            self.request[field] = original
        self.request["light_intent"]["hazard_signal"] = True
        self.rejected("UNSUPPORTED_STOP_EXTENSIONS")
        self.request["light_intent"]["left_signal"] = True
        self.request["light_intent"]["right_signal"] = True
        self.rejected("LIGHT_INTENT_INVALID")

    def test_numbers_and_flags_are_not_coerced(self):
        for field, value, reason in (("speed_cap_mps", True, "GOAL_NUMBERS_INVALID"),
                                    ("speed_cap_mps", float("nan"), "GOAL_NUMBERS_INVALID"),
                                    ("speed_cap_mps", 10 ** 1000, "GOAL_NUMBERS_INVALID"),
                                    ("stop_distance_m", -0.1, "GOAL_NUMBERS_INVALID"),
                                    ("precision_stop", 1, "STOP_FLAGS_INVALID"),
                                    ("motion_direction", True, "DIRECTION_INVALID")):
            with self.subTest(field=field):
                original = self.request[field]
                self.request[field] = value
                self.rejected(reason)
                self.request[field] = original

    def test_per_frame_speed_distance_refresh_does_not_need_new_revision(self):
        self.request.update(maneuver="FOLLOW", speed_cap_mps=3.0, stop_distance_m=12.0)
        self.assertTrue(self.assess()["eligible"])
        self.request.update(speed_cap_mps=2.0, stop_distance_m=8.0)
        self.assertTrue(self.assess()["eligible"])
        self.assertEqual(self.request["revision"], 2)


if __name__ == "__main__":
    unittest.main()
