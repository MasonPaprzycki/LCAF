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
        retracted_distance=0.0,
        extended_distance=10.0,
    )


def make_single_switch_joint(joint: int = 0, axis: str = "X") -> JointConfiguration:
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
        positive_limit_input=None,
        has_limit_switches=True,
        dual_limit_switches=False,
        retracted_distance=0.0,
        extended_distance=10.0,
    )


def make_retract_to_joint(joint: int = 1, axis: str = "Y", retract_to: float = 6.0) -> JointConfiguration:
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
        positive_limit_input=None,
        has_limit_switches=True,
        dual_limit_switches=False,
        retract_to=retract_to,
        retracted_distance=0.0,
        extended_distance=10.0,
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


class SingleLimitSwitchHomingTests(unittest.TestCase):
    """
    A dual_limit_switches=False joint (JointConfiguration) has only a
    negative limit switch -- homing should seek it to establish zero, then
    stop and trust the configured extended_distance, without ever jogging
    toward a positive switch that doesn't exist.
    """

    def setUp(self):
        self.fake_linuxcnc = sys.modules["linuxcnc"]
        self.fake_hal = sys.modules["hal"]
        self.fake_hal.pins.clear()

        self.joint = make_single_switch_joint()
        self.machine = LinuxCNCMachineInterface(use_native_homing=False)
        self.command = self.fake_linuxcnc._last_command
        self.stat = self.fake_linuxcnc._last_stat
        self.axial = LinuxCNCAxialInterface(self.joint, self.machine)

    def neg_pin(self) -> str:
        return self.joint.negative_limit_hal_pin

    def test_homing_completes_after_only_seeking_negative_switch(self):
        self.axial.start_homing(speed=1.0, timeout=5.0)
        self.assertEqual(self.axial._homing_phase, "software_seek_min")

        self.fake_hal.pins[self.neg_pin()] = True
        done = self.axial.poll_homing()

        self.assertTrue(done)
        self.assertTrue(self.axial.has_axis_been_homed())

    def test_max_limit_trusts_configured_extended_distance_not_measured(self):
        self.axial.start_homing(speed=1.0, timeout=5.0)
        self.fake_hal.pins[self.neg_pin()] = True
        self.axial.poll_homing()

        self.assertEqual(self.axial.max_limit["native"], self.joint.extended_distance)

    def test_no_jog_ever_targets_the_positive_direction(self):
        self.axial.start_homing(speed=1.0, timeout=5.0)
        self.fake_hal.pins[self.neg_pin()] = True
        self.axial.poll_homing()

        jog_speeds = [c[4] for c in self.command.calls if c[0] == "jog"]
        # jog_negative() passes a negated speed (see LinuxCNCAxialInterface);
        # a positive-direction seek would show up as a positive speed here.
        self.assertTrue(all(speed <= 0 for speed in jog_speeds if speed is not None))

    def test_starting_already_on_the_only_switch_finishes_immediately(self):
        self.fake_hal.pins[self.neg_pin()] = True

        self.axial.start_homing(speed=1.0, timeout=5.0)

        self.assertIsNone(self.axial._homing_phase)
        self.assertTrue(self.axial.has_axis_been_homed())
        self.assertEqual(self.axial.max_limit["native"], self.joint.extended_distance)


class RetractToZeroTests(unittest.TestCase):
    """
    MotionCoordinator re-seeks a switched joint's negative limit switch
    before every retract (not just once at home_all()) via
    start_retract_to_zero()/poll_retract_to_zero() -- see
    docs/hardware_setup.md section 7 ("Retract-to-zero"). Unlike
    start_homing(), this must never touch min_limit/max_limit (a
    dual_limit_switches joint's measured travel must survive every later
    retract unchanged) and must refuse to run before initial homing.
    """

    def setUp(self):
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

    def complete_initial_homing(self):
        self.axial.start_homing(speed=1.0, timeout=5.0)
        self.fake_hal.pins[self.neg_pin()] = True
        self.axial.poll_homing()
        self.stat.joint_actual_position[self.joint.joint] = 8.5
        self.fake_hal.pins[self.joint.positive_limit_hal_pin] = True
        self.axial.poll_homing()
        self.assertTrue(self.axial.has_axis_been_homed())
        self.fake_hal.pins.clear()

    def test_retract_before_initial_homing_raises(self):
        with self.assertRaises(RuntimeError):
            self.axial.start_retract_to_zero(speed=1.0, timeout=5.0)

    def test_retract_on_switchless_joint_raises(self):
        switchless = JointConfiguration(
            joint=3, axis="A", motor_steps_per_revolution=200, microsteps=16,
            travel_per_motor_rev=360.0, max_velocity=45.0, max_acceleration=50.0,
            mesa_stepgen="hm2_7i76e.0.stepgen.03", is_angular=True,
            has_limit_switches=False, negative_limit_input=None,
            positive_limit_input=None, retracted_distance=100000.0,
            extended_distance=100000.0,
        )
        axial = LinuxCNCAxialInterface(switchless, self.machine)
        axial.axis_homed = True

        with self.assertRaises(RuntimeError):
            axial.start_retract_to_zero(speed=1.0, timeout=5.0)

    def test_retract_seeks_negative_switch_and_completes_without_remeasuring_max(self):
        self.complete_initial_homing()
        measured_max = self.axial.max_limit["native"]
        self.assertEqual(measured_max, 8.5)

        self.axial.start_retract_to_zero(speed=1.0, timeout=5.0)
        self.assertEqual(self.axial._retract_phase, "software_seek_min")

        self.fake_hal.pins[self.neg_pin()] = True
        done = self.axial.poll_retract_to_zero()

        self.assertTrue(done)
        # max_limit must survive the retract unchanged -- retract-to-zero
        # never remeasures travel, even for a dual_limit_switches joint.
        self.assertEqual(self.axial.max_limit["native"], measured_max)

    def test_retract_overrides_limits_before_jogging(self):
        self.complete_initial_homing()
        overrides_before = self.command.override_limits_calls

        self.axial.start_retract_to_zero(speed=1.0, timeout=5.0)

        self.assertGreater(self.command.override_limits_calls, overrides_before)

    def test_retract_can_run_again_after_completing_once(self):
        self.complete_initial_homing()

        self.axial.start_retract_to_zero(speed=1.0, timeout=5.0)
        self.fake_hal.pins[self.neg_pin()] = True
        self.assertTrue(self.axial.poll_retract_to_zero())

        self.fake_hal.pins.clear()
        self.axial.start_retract_to_zero(speed=1.0, timeout=5.0)
        self.assertEqual(self.axial._retract_phase, "software_seek_min")
        self.fake_hal.pins[self.neg_pin()] = True
        self.assertTrue(self.axial.poll_retract_to_zero())

    def test_native_mode_retract_reruns_native_home(self):
        native_machine = LinuxCNCMachineInterface(use_native_homing=True)
        native_command = self.fake_linuxcnc._last_command
        native_axial = LinuxCNCAxialInterface(self.joint, native_machine)
        native_axial.axis_homed = True

        native_axial.start_retract_to_zero(speed=1.0, timeout=5.0)

        self.assertEqual(native_axial._retract_phase, "native_wait")
        home_calls = [c for c in native_command.calls if c[0] == "home"]
        self.assertEqual(len(home_calls), 1)


class RetractToTests(unittest.TestCase):
    """
    JointConfiguration.retract_to (this project's Y axis -- see
    axis.json, set to 1.5) changes what "retract" means for that one
    joint: since it has no positive limit switch to re-seek against
    (dual_limit_switches is False machine-wide -- see
    docs/hardware_setup.md section 5), retracting it means commanding a
    plain MDI move to its configured retract_to position instead of
    jogging toward the negative switch. This must never jog, never touch
    position_offset_to_native, and must work the same whether or not
    use_linuxcnc_native_processes is set (unlike the negative-switch
    retract, which behaves differently in native mode).
    """

    def setUp(self):
        self.fake_linuxcnc = sys.modules["linuxcnc"]
        self.fake_hal = sys.modules["hal"]
        self.fake_hal.pins.clear()

        self.joint = make_retract_to_joint()
        self.machine = LinuxCNCMachineInterface(use_native_homing=False)
        self.command = self.fake_linuxcnc._last_command
        self.stat = self.fake_linuxcnc._last_stat
        self.axial = LinuxCNCAxialInterface(self.joint, self.machine)
        self.axial.axis_homed = True

    def test_start_retract_commands_an_mdi_move_to_retract_to_position(self):
        self.axial.start_retract_to_zero(speed=1.0, timeout=5.0)

        self.assertEqual(self.axial._retract_phase, "move_to_retract_position")

        mdi_calls = [c for c in self.command.calls if c[0] == "mdi"]
        self.assertEqual(len(mdi_calls), 1)
        # retract_to=6.0 in -> 152.4 mm (to_machine_units) -- deliberately
        # distinct from extended_distance=10.0, confirming retract targets
        # retract_to and not extended_distance.
        self.assertIn("Y152.4000", mdi_calls[0][1])

        call_names = [c[0] for c in self.command.calls]
        self.assertNotIn("jog", call_names)

    def test_retract_completes_once_joint_reports_in_position(self):
        self.stat.joint[self.joint.joint]["inpos"] = False
        self.axial.start_retract_to_zero(speed=1.0, timeout=5.0)

        self.assertFalse(self.axial.poll_retract_to_zero())

        self.stat.joint[self.joint.joint]["inpos"] = True
        self.assertTrue(self.axial.poll_retract_to_zero())
        self.assertIsNone(self.axial._retract_phase)

    def test_retract_never_touches_position_offset_or_measured_limits(self):
        self.axial.position_offset_to_native = 3.25
        self.axial.max_limit["native"] = 8.5

        self.axial.start_retract_to_zero(speed=1.0, timeout=5.0)
        self.assertTrue(self.axial.poll_retract_to_zero())

        self.assertEqual(self.axial.position_offset_to_native, 3.25)
        self.assertEqual(self.axial.max_limit["native"], 8.5)

    def test_never_overrides_limits_or_seeks_a_switch(self):
        self.axial.start_retract_to_zero(speed=1.0, timeout=5.0)
        self.axial.poll_retract_to_zero()

        self.assertEqual(self.command.override_limits_calls, 0)

    def test_native_homing_mode_still_moves_instead_of_native_homing(self):
        native_machine = LinuxCNCMachineInterface(use_native_homing=True)
        native_command = self.fake_linuxcnc._last_command
        native_axial = LinuxCNCAxialInterface(self.joint, native_machine)
        native_axial.axis_homed = True

        native_axial.start_retract_to_zero(speed=1.0, timeout=5.0)

        self.assertEqual(native_axial._retract_phase, "move_to_retract_position")
        call_names = [c[0] for c in native_command.calls]
        self.assertNotIn("home", call_names)
        self.assertIn("mdi", call_names)

    def test_timeout_raises_if_never_reaches_position(self):
        import time as time_module

        self.stat.joint[self.joint.joint]["inpos"] = False
        self.axial.start_retract_to_zero(speed=1.0, timeout=0.0)
        self.axial._retract_start_time = time_module.monotonic() - 1.0

        with self.assertRaises(TimeoutError):
            self.axial.poll_retract_to_zero()


if __name__ == "__main__":
    unittest.main()
