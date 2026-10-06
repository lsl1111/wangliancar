"""Shared startup capabilities must match the actual controller configuration."""

import unittest
from types import SimpleNamespace
from unittest.mock import patch

from members.planning_stub import configure_planning, plan
from members.decision.settings import DecisionSettings
from runtime import _validate_motion_contract
from tests.test_planning import scene


class PlanningConfigurationTests(unittest.TestCase):
    def tearDown(self):
        with patch.dict('os.environ', {}, clear=True):
            configure_planning()

    def config(self, **changes):
        values = dict(control_calibrated=True, control_wheelbase_m=2.9187,
                      control_front_steer_max_rad=.5)
        values.update(changes)
        return SimpleNamespace(**values)

    def test_configured_geometry_reaches_planning_without_algorithm_defaults(self):
        with patch.dict('os.environ', {'NEVC_VEHICLE_FRONT_OFFSET_M': '3.9187',
                                      'NEVC_VEHICLE_HALF_WIDTH_M': '.9',
                                      'NEVC_VEHICLE_REAR_OFFSET_M': '.88'}, clear=True):
            settings = configure_planning(self.config())
            self.assertEqual(2.9187, settings.wheelbase_m)
            self.assertEqual(.5, settings.front_steer_max_rad)
            self.assertEqual(.88, settings.rear_offset_m)
            self.assertEqual(3.9187, settings.front_offset_m)
            self.assertEqual(.9, settings.half_width_m)

    def test_other_configured_car_does_not_reuse_the_preset_steering_limit(self):
        with patch.dict('os.environ', {}, clear=True):
            settings = configure_planning(self.config(control_wheelbase_m=3.2,
                control_front_steer_max_rad=.4))
            self.assertEqual(3.2, settings.wheelbase_m)
            self.assertEqual(.4, settings.front_steer_max_rad)
            self.assertIsNone(settings.rear_offset_m)

    def test_unconfigured_control_does_not_supply_made_up_geometry(self):
        with patch.dict('os.environ', {}, clear=True):
            settings = configure_planning(self.config(control_calibrated=False))
            self.assertIsNone(settings.wheelbase_m)
            self.assertIsNone(settings.front_steer_max_rad)

    def test_conflicting_environment_and_control_config_is_rejected(self):
        with patch.dict('os.environ', {'NEVC_VEHICLE_WHEELBASE_M': '3.2'}, clear=True):
            with self.assertRaises(ValueError):
                configure_planning(self.config())

    def test_invalid_live_override_returns_an_invalid_bound_trajectory(self):
        with patch.dict('os.environ', {}, clear=True):
            configure_planning(self.config())
            p, d = scene()
            with patch.dict('os.environ', {'NEVC_VEHICLE_FRONT_STEER_MAX_RAD': '.4'}):
                t = plan(p, d)
            self.assertFalse(t.valid)
            self.assertEqual(d.frame_id, t.frame_id)
            self.assertIn('planning/control', t.reason)

    def test_common_motion_overrides_are_shared_with_decision(self):
        overrides = {'NEVC_VEHICLE_FRONT_OFFSET_M': '3.9187',
                     'NEVC_VEHICLE_HALF_WIDTH_M': '.9',
                     'NEVC_PLANNING_HORIZON_M': '75',
                     'NEVC_PLANNING_DECELERATION_MPS2': '1.7',
                     'NEVC_PLANNING_LATERAL_GUARD_TIME_S': '4',
                     'NEVC_PLANNING_LATERAL_MARGIN_M': '.1'}
        with patch.dict('os.environ', overrides, clear=True):
            planning = configure_planning(self.config())
            decision = DecisionSettings.from_environment()
            _validate_motion_contract(planning, decision)
            self.assertEqual(75, decision.motion_horizon_m)

    def test_member_specific_conflicting_width_is_rejected(self):
        with patch.dict('os.environ', {'NEVC_VEHICLE_FRONT_OFFSET_M': '3.9187',
                'NEVC_VEHICLE_HALF_WIDTH_M': '.9',
                'NEVC_DECISION_HALF_WIDTH_M': '.8'}, clear=True):
            planning = configure_planning(self.config())
            decision = DecisionSettings.from_environment()
            with self.assertRaises(ValueError):
                _validate_motion_contract(planning, decision)
