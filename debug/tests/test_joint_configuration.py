from __future__ import annotations

import logging
import unittest
from pathlib import Path

from lcaf.utils.joint_configuration import (
    JointConfiguration,
    MachineConfiguration,
    generate_hal,
    generate_ini,
    load_machine_configuration,
)

REPO_ROOT = Path(__file__).resolve().parents[2]


def make_joint(**overrides) -> JointConfiguration:
    defaults = dict(
        joint=0,
        axis="X",
        motor_steps_per_revolution=200,
        microsteps=16,
        travel_per_motor_rev=0.2,
        max_velocity=1.0,
        max_acceleration=5.0,
        mesa_stepgen="hm2_7i76e.0.stepgen.00",
        negative_limit_input="hm2_7i76e.0.7i76.0.0.input-00",
        positive_limit_input="hm2_7i76e.0.7i76.0.0.input-01",
        has_limit_switches=True,
        retracted_distance=0.0,
        extended_distance=10.0,
    )
    defaults.update(overrides)
    return JointConfiguration(**defaults)


class LimitSwitchPinValidationTests(unittest.TestCase):
    """
    Negative and positive limit switches must be wired to different physical
    inputs -- otherwise homing/limit-switch reads can't tell which end of
    travel was actually reached. See JointConfiguration.__post_init__.
    """

    def test_same_pin_for_both_limits_is_rejected(self):
        with self.assertRaises(ValueError):
            make_joint(
                negative_limit_input="hm2_7i76e.0.7i76.0.0.input-00",
                positive_limit_input="hm2_7i76e.0.7i76.0.0.input-00",
            )

    def test_different_pins_are_accepted(self):
        joint = make_joint(
            negative_limit_input="hm2_7i76e.0.7i76.0.0.input-00",
            positive_limit_input="hm2_7i76e.0.7i76.0.0.input-01",
        )
        self.assertNotEqual(joint.negative_limit_input, joint.positive_limit_input)

    def test_switchless_joint_with_no_limit_inputs_is_unaffected(self):
        joint = make_joint(
            joint=3,
            axis="A",
            is_angular=True,
            has_limit_switches=False,
            negative_limit_input=None,
            positive_limit_input=None,
            retracted_distance=100000.0,
            extended_distance=100000.0,
        )
        self.assertIsNone(joint.negative_limit_input)
        self.assertIsNone(joint.positive_limit_input)


class DualLimitSwitchesValidationTests(unittest.TestCase):
    """
    dual_limit_switches (default True) selects whether a switched joint's
    homing measures travel with a second switch or trusts the configured
    extended_distance from just the one negative switch -- see
    JointConfiguration.dual_limit_switches.
    """

    def test_dual_mode_requires_positive_limit_input(self):
        with self.assertRaises(ValueError):
            make_joint(
                dual_limit_switches=True,
                positive_limit_input=None,
            )

    def test_single_switch_mode_forbids_positive_limit_input(self):
        with self.assertRaises(ValueError):
            make_joint(
                dual_limit_switches=False,
                positive_limit_input="hm2_7i76e.0.7i76.0.0.input-01",
            )

    def test_single_switch_mode_accepted_with_only_negative_input(self):
        joint = make_joint(
            dual_limit_switches=False,
            positive_limit_input=None,
        )
        self.assertFalse(joint.dual_limit_switches)
        self.assertIsNone(joint.positive_limit_input)
        self.assertIsNotNone(joint.negative_limit_input)

    def test_default_is_dual_limit_switches_true(self):
        joint = make_joint()
        self.assertTrue(joint.dual_limit_switches)


class GeneratedHalLimitSwitchWiringTests(unittest.TestCase):
    def test_negative_limit_wires_to_its_hal_net(self):
        machine = load_machine_configuration(REPO_ROOT / "configs" / "machine.json")
        hal_text = generate_hal(machine)

        for joint in machine.joints:
            if not joint.has_limit_switches:
                continue

            neg_line = f"net {joint.axis.lower()}-neg-lim {joint.negative_limit_input} => {joint.negative_limit_hal_pin}"
            self.assertIn(neg_line, hal_text)

            # This project's real X/Y/Z are single-switch (negative only --
            # see docs/hardware_setup.md section 5): no positive switch is
            # wired, so there is no positive-limit net to assert on, and
            # negative_limit_input/positive_limit_input trivially differ
            # (the latter is None).
            self.assertIsNone(joint.positive_limit_input)
            self.assertNotEqual(joint.negative_limit_input, joint.positive_limit_input)


class GeneratedHalSingleLimitSwitchSimulationTests(unittest.TestCase):
    """
    A dual_limit_switches=False joint has no positive limit switch wired on
    real hardware -- the simulated HAL for it (generate_hal(simulate=True))
    should not fabricate one either, since that would let a test pass
    against simulated hardware that couldn't exist for real.
    """

    def make_single_switch_machine(self) -> MachineConfiguration:
        joint = make_joint(dual_limit_switches=False, positive_limit_input=None)
        return MachineConfiguration(machine_name="TestMachine", joints=[joint])

    def test_no_positive_comp_loaded_or_wired(self):
        machine = self.make_single_switch_machine()
        hal_text = generate_hal(machine, simulate=True)

        self.assertNotIn("complim_x_pos", hal_text)
        self.assertNotIn(f"=> {machine.joints[0].positive_limit_hal_pin}", hal_text)

    def test_negative_comp_still_loaded_and_wired(self):
        machine = self.make_single_switch_machine()
        hal_text = generate_hal(machine, simulate=True)

        self.assertIn("complim_x_neg", hal_text)
        self.assertIn(f"=> {machine.joints[0].negative_limit_hal_pin}", hal_text)


class NullableTravelLimitTests(unittest.TestCase):
    """
    retracted_distance/extended_distance may each be left None to disable
    that end's software soft limit entirely (this project's A axis does
    this for both -- see configs/axis.json) -- __post_init__ must accept
    it (with a logged warning, since it's a real safety-check reduction)
    rather than rejecting it, and generate_ini() must omit the
    corresponding MIN_LIMIT/MAX_LIMIT entry rather than crashing.
    """

    def test_null_retracted_distance_is_accepted_and_warns(self):
        with self.assertLogs("lcaf.utils.joint_configuration", level="WARNING"):
            joint = make_joint(retracted_distance=None)
        self.assertIsNone(joint.retracted_distance)
        self.assertIsNone(joint.min_travel)

    def test_null_extended_distance_is_accepted_and_warns(self):
        with self.assertLogs("lcaf.utils.joint_configuration", level="WARNING"):
            joint = make_joint(extended_distance=None)
        self.assertIsNone(joint.extended_distance)

    def test_both_null_is_accepted_with_no_positive_limit_switch(self):
        joint = make_joint(
            joint=3,
            axis="A",
            is_angular=True,
            has_limit_switches=False,
            negative_limit_input=None,
            positive_limit_input=None,
            retracted_distance=None,
            extended_distance=None,
        )
        self.assertIsNone(joint.retracted_distance)
        self.assertIsNone(joint.extended_distance)
        self.assertIsNone(joint.min_travel)

    def test_generate_ini_omits_min_and_max_limit_when_null(self):
        joint = make_joint(
            joint=3,
            axis="A",
            is_angular=True,
            has_limit_switches=False,
            negative_limit_input=None,
            positive_limit_input=None,
            retracted_distance=None,
            extended_distance=None,
        )
        machine = MachineConfiguration(machine_name="TestMachine", joints=[joint])
        ini_text = generate_ini(machine)

        joint_section = ini_text.split("[JOINT_3]")[1].split("[AXIS_A]")[0]
        self.assertNotIn("MIN_LIMIT", joint_section)
        self.assertNotIn("MAX_LIMIT", joint_section)

    def test_negative_retracted_distance_still_rejected(self):
        with self.assertRaises(ValueError):
            make_joint(retracted_distance=-1.0)

    def test_non_positive_extended_distance_still_rejected(self):
        with self.assertRaises(ValueError):
            make_joint(extended_distance=0.0)


class RetractToValidationTests(unittest.TestCase):
    """
    JointConfiguration.retract_to (this project's Y axis, set to 1.5 in
    configs/axis.json) is the absolute position a joint moves to on
    retract instead of re-seeking its negative limit switch -- see
    LinuxCNCAxialInterface.start_retract_to_zero(). When both travel
    limits are known it must fall within [-retracted_distance,
    extended_distance], the same soft-limited range every other command is
    held to.
    """

    def test_retract_to_within_bounds_is_accepted(self):
        joint = make_joint(retracted_distance=0.25, extended_distance=2.5, retract_to=1.5)
        self.assertEqual(joint.retract_to, 1.5)

    def test_retract_to_beyond_extended_distance_is_rejected(self):
        with self.assertRaises(ValueError):
            make_joint(retracted_distance=0.25, extended_distance=2.5, retract_to=3.0)

    def test_retract_to_beyond_retracted_distance_is_rejected(self):
        with self.assertRaises(ValueError):
            make_joint(retracted_distance=0.25, extended_distance=2.5, retract_to=-1.0)

    def test_retract_to_defaults_to_none(self):
        joint = make_joint()
        self.assertIsNone(joint.retract_to)

    def test_retract_to_unbounded_when_distances_are_null(self):
        # No range check possible without configured bounds -- any value is
        # accepted (with the usual retracted_distance/extended_distance-null
        # warnings, not a retract_to-specific one).
        with self.assertLogs("lcaf.utils.joint_configuration", level="WARNING"):
            joint = make_joint(retracted_distance=None, extended_distance=None, retract_to=1000.0)
        self.assertEqual(joint.retract_to, 1000.0)


if __name__ == "__main__":
    unittest.main()
