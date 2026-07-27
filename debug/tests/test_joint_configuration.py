from __future__ import annotations

import unittest
from pathlib import Path

from lcaf.utils.joint_configuration import (
    JointConfiguration,
    generate_hal,
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
        max_travel=10.0,
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
            max_travel=100000.0,
        )
        self.assertIsNone(joint.negative_limit_input)
        self.assertIsNone(joint.positive_limit_input)


class GeneratedHalLimitSwitchWiringTests(unittest.TestCase):
    def test_negative_and_positive_limits_wire_to_distinct_hal_nets(self):
        machine = load_machine_configuration(REPO_ROOT / "configs" / "machine.json")
        hal_text = generate_hal(machine)

        for joint in machine.joints:
            if not joint.has_limit_switches:
                continue

            neg_line = f"net {joint.axis.lower()}-neg-lim {joint.negative_limit_input} => {joint.negative_limit_hal_pin}"
            pos_line = f"net {joint.axis.lower()}-pos-lim {joint.positive_limit_input} => {joint.positive_limit_hal_pin}"

            self.assertIn(neg_line, hal_text)
            self.assertIn(pos_line, hal_text)
            self.assertNotEqual(joint.negative_limit_input, joint.positive_limit_input)


if __name__ == "__main__":
    unittest.main()
