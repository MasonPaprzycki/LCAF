from __future__ import annotations

import sys
import unittest

from lcaf.control.linuxcnc_interface import LinuxCNCAxialInterface, LinuxCNCMachineInterface
from lcaf.utils.joint_configuration import JointConfiguration


def make_switched_joint(joint: int = 0, axis: str = "X") -> JointConfiguration:
    return JointConfiguration(
        joint=joint,
        axis=axis,
        motor_steps_per_revolution=200,
        microsteps=16,
        travel_per_motor_rev=0.2,
        max_velocity=1.0,
        max_acceleration=5.0,
        mesa_stepgen=f"hm2_7i76e.0.stepgen.0{joint}",
        negative_limit_input=f"hm2_7i76e.0.7i76.0.0.input-{2 * joint:02d}",
        positive_limit_input=f"hm2_7i76e.0.7i76.0.0.input-{2 * joint + 1:02d}",
        has_limit_switches=True,
        max_travel=10.0,
    )


class SoftwareHomingLimitSwitchTests(unittest.TestCase):
    """
    Software homing (MachineConfiguration.use_linuxcnc_native_processes=False,
    this project's default) deliberately jogs a joint into its own limit
    switch to find it. The switch is also wired straight into LinuxCNC's own
    hard-limit fault pins (generate_hal()), which -- unlike native homing --
    software homing never tells LinuxCNC to ignore. These tests pin down
    LinuxCNCAxialInterface's fix for that: command.override_limits() must be
    called before every jog that risks touching a switch, so the deliberate
    contact never trips LinuxCNC's "joint N on limit switch error" (which
    would otherwise disable every joint's enable output machine-wide -- see
    linuxcnc_interface.py's _jog_toward_limit_switch docstring and
    docs/hardware_setup.md).
    """

    def setUp(self):
        # Installed once by debug/tests/conftest.py before this module (and
        # lcaf.control.linuxcnc_interface) was ever imported -- re-fetch the
        # same fake module objects rather than reinstalling new ones, since
        # linuxcnc_interface's own `import linuxcnc` / `import hal` already
        # bound those names to whatever was in sys.modules at that time.
        self.fake_linuxcnc = sys.modules["linuxcnc"]
        self.fake_hal = sys.modules["hal"]
        self.fake_hal.pins.clear()

        self.joint = make_switched_joint()
        self.machine = LinuxCNCMachineInterface(use_native_homing=False)
        self.command = self.fake_linuxcnc._last_command
        self.stat = self.fake_linuxcnc._last_stat
        self.axial = LinuxCNCAxialInterface(self.joint, self.machine)

    def neg_pin(self) -> str:
        return self.joint.negative_limit_hal_pin

    def pos_pin(self) -> str:
        return self.joint.positive_limit_hal_pin

    def test_negative_and_positive_limits_are_different_hal_pins(self):
        self.assertNotEqual(self.neg_pin(), self.pos_pin())

    def test_seek_min_overrides_limits_before_jogging_toward_negative_switch(self):
        self.axial.start_homing(speed=1.0, timeout=5.0)

        self.assertEqual(self.axial._homing_phase, "software_seek_min")
        self.assertGreaterEqual(self.command.override_limits_calls, 1)

        call_names = [c[0] for c in self.command.calls]
        first_jog = call_names.index("jog")
        self.assertLess(call_names.index("override_limits"), first_jog)

    def test_hitting_negative_switch_re_overrides_before_seeking_positive_switch(self):
        self.axial.start_homing(speed=1.0, timeout=5.0)
        overrides_before = self.command.override_limits_calls

        self.fake_hal.pins[self.neg_pin()] = True
        done = self.axial.poll_homing()

        self.assertFalse(done)
        self.assertEqual(self.axial._homing_phase, "software_seek_max")
        self.assertGreater(self.command.override_limits_calls, overrides_before)

    def test_full_homing_sequence_completes_without_raising(self):
        self.axial.start_homing(speed=1.0, timeout=5.0)
        self.assertEqual(self.axial._homing_phase, "software_seek_min")

        self.fake_hal.pins[self.neg_pin()] = True
        self.axial.poll_homing()
        self.assertEqual(self.axial._homing_phase, "software_seek_max")

        self.stat.joint_actual_position[self.joint.joint] = 8.5
        self.fake_hal.pins[self.pos_pin()] = True
        done = self.axial.poll_homing()

        self.assertTrue(done)
        self.assertTrue(self.axial.has_axis_been_homed())
        # Both the negative-seek and positive-seek jogs must each have been
        # bracketed with their own override -- neither switch is exempt.
        self.assertGreaterEqual(self.command.override_limits_calls, 2)

    def test_starting_already_on_negative_switch_still_overrides_before_seeking_positive(self):
        self.fake_hal.pins[self.neg_pin()] = True

        self.axial.start_homing(speed=1.0, timeout=5.0)

        self.assertEqual(self.axial._homing_phase, "software_seek_max")
        self.assertGreaterEqual(self.command.override_limits_calls, 1)

    def test_override_limits_is_re_armed_every_heartbeat_during_a_seek(self):
        self.axial.start_homing(speed=1.0, timeout=5.0)
        overrides_after_start = self.command.override_limits_calls
        self.assertGreaterEqual(overrides_after_start, 1)

        # Neither switch trips yet -- still mid-seek on every one of these.
        for _ in range(3):
            done = self.axial.poll_homing()
            self.assertFalse(done)

        self.assertGreater(self.command.override_limits_calls, overrides_after_start)

    def test_is_on_hard_limit_reflects_status_not_faulted(self):
        self.stat.joint[self.joint.joint]["min_hard_limit"] = False
        self.stat.joint[self.joint.joint]["max_hard_limit"] = False
        self.assertFalse(self.axial.is_on_hard_limit())

        self.stat.joint[self.joint.joint]["min_hard_limit"] = True
        self.assertTrue(self.axial.is_on_hard_limit())
        # A hard-limit trip is a distinct signal from the amp/following-error
        # fault flag -- is_faulted() must not be affected by it.
        self.assertFalse(self.axial.is_faulted())

    def test_native_homing_never_calls_override_limits(self):
        native_machine = LinuxCNCMachineInterface(use_native_homing=True)
        native_command = self.fake_linuxcnc._last_command
        native_axial = LinuxCNCAxialInterface(self.joint, native_machine)

        native_axial.start_homing(speed=1.0, timeout=5.0)

        self.assertEqual(native_axial._homing_phase, "native_wait")
        self.assertEqual(native_command.override_limits_calls, 0)


if __name__ == "__main__":
    unittest.main()
