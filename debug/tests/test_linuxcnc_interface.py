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


class HomeAllCommandTests(unittest.TestCase):
    """
    MotionCoordinator.home_all() homes every joint through a single native
    "Home All" (LinuxCNCMachineInterface.home_all_command()) -- the exact
    same command.home(-1) the Axis GUI's own "Home All" button sends --
    rather than a separate command.home(joint) per joint. This project
    previously did the latter; removed, since it was an unnecessary
    reimplementation of exactly what "Home All" already does natively (see
    docs/hardware_setup.md section 7).
    """

    def setUp(self):
        self.fake_linuxcnc = sys.modules["linuxcnc"]
        self.machine = LinuxCNCMachineInterface()
        self.command = self.fake_linuxcnc._last_command

    def test_home_all_command_issues_a_single_home_minus_one(self):
        self.machine.home_all_command()

        home_calls = [c for c in self.command.calls if c[0] == "home"]
        self.assertEqual(len(home_calls), 1)
        self.assertEqual(home_calls[0][1], -1)

        mode_calls = [c for c in self.command.calls if c[0] == "mode"]
        self.assertEqual(len(mode_calls), 1)

        # LinuxCNC rejects a directly-numbered command.home(joint) with
        # "must be in joint mode to home" unless teleop_enable(0) has been
        # called -- MANUAL task mode alone is not sufficient (see
        # ensure_manual_mode()'s docstring). Home All needs the machine in
        # the same state.
        teleop_calls = [c for c in self.command.calls if c[0] == "teleop_enable"]
        self.assertEqual(len(teleop_calls), 1)
        self.assertEqual(teleop_calls[0][1], 0)


class NativeHomingTests(unittest.TestCase):
    """
    This project always homes via LinuxCNC's own native homing sequence --
    a single machine-wide "Home All" (LinuxCNCMachineInterface.
    home_all_command(), see HomeAllCommandTests above) that every joint
    then independently waits out via
    LinuxCNCAxialInterface.begin_homing_wait()/poll_homing() (without
    issuing any command of its own) for status.joint[n]['homed']. There is
    no software-jog-to-switch fallback (removed: LinuxCNC's own realtime
    motion loop re-checks each joint's hard-limit HAL pin every servo cycle
    independently of any Python process, and a Python client reactively
    calling command.override_limits() once per its own control-loop
    heartbeat cannot reliably win that race -- see
    JointConfiguration/MachineConfiguration's homing note and
    docs/potential_issues.md). generate_ini()'s HOME_IGNORE_LIMITS=YES is
    what keeps LinuxCNC's own homing state machine from faulting on the
    deliberate switch contact instead.
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
        self.machine = LinuxCNCMachineInterface()
        self.command = self.fake_linuxcnc._last_command
        self.stat = self.fake_linuxcnc._last_stat
        self.axial = LinuxCNCAxialInterface(self.joint, self.machine)

    def neg_pin(self) -> str:
        return self.joint.negative_limit_hal_pin

    def pos_pin(self) -> str:
        return self.joint.positive_limit_hal_pin

    def test_negative_and_positive_limits_are_different_hal_pins(self):
        self.assertNotEqual(self.neg_pin(), self.pos_pin())

    def test_begin_homing_wait_issues_no_command_of_its_own(self):
        self.axial.begin_homing_wait(timeout=5.0)

        self.assertEqual(self.axial._homing_phase, "native_wait")
        home_calls = [c for c in self.command.calls if c[0] == "home"]
        self.assertEqual(len(home_calls), 0)

    def test_poll_homing_waits_for_homed_status(self):
        self.axial.begin_homing_wait(timeout=5.0)

        self.assertFalse(self.axial.poll_homing())
        self.assertFalse(self.axial.has_axis_been_homed())

        self.stat.joint[self.joint.joint]["homed"] = True
        self.assertTrue(self.axial.poll_homing())
        self.assertTrue(self.axial.has_axis_been_homed())

    def test_homing_never_remeasures_travel_even_with_dual_limit_switches(self):
        # Native homing never seeks a positive switch or remeasures travel
        # regardless of dual_limit_switches -- MAX_LIMIT is always the
        # static configured extended_distance (see generate_ini() /
        # JointConfiguration.dual_limit_switches).
        self.axial.begin_homing_wait(timeout=5.0)
        self.stat.joint[self.joint.joint]["homed"] = True
        self.axial.poll_homing()

        self.assertEqual(self.axial.min_limit["native"], self.joint.retracted_distance)
        self.assertEqual(self.axial.max_limit["native"], self.joint.extended_distance)

    def test_fault_while_homing_raises(self):
        self.axial.begin_homing_wait(timeout=5.0)
        self.stat.joint[self.joint.joint]["fault"] = True

        with self.assertRaises(RuntimeError):
            self.axial.poll_homing()

    def test_timeout_raises_if_never_reports_homed(self):
        import time as time_module

        self.axial.begin_homing_wait(timeout=0.0)
        self.axial._homing_start_time = time_module.monotonic() - 1.0

        with self.assertRaises(TimeoutError):
            self.axial.poll_homing()

    def test_is_on_hard_limit_reflects_status_not_faulted(self):
        self.stat.joint[self.joint.joint]["min_hard_limit"] = False
        self.stat.joint[self.joint.joint]["max_hard_limit"] = False
        self.assertFalse(self.axial.is_on_hard_limit())

        self.stat.joint[self.joint.joint]["min_hard_limit"] = True
        self.assertTrue(self.axial.is_on_hard_limit())
        # A hard-limit trip is a distinct signal from the amp/following-error
        # fault flag -- is_faulted() must not be affected by it.
        self.assertFalse(self.axial.is_faulted())


class SingleLimitSwitchHomingTests(unittest.TestCase):
    """
    A dual_limit_switches=False joint (JointConfiguration) has only a
    negative limit switch wired -- native homing still just trusts the
    configured extended_distance as MAX_LIMIT (it never seeks a positive
    switch regardless of dual_limit_switches), so this must behave
    identically to the dual-switch case above.
    """

    def setUp(self):
        self.fake_linuxcnc = sys.modules["linuxcnc"]
        self.fake_hal = sys.modules["hal"]
        self.fake_hal.pins.clear()

        self.joint = make_single_switch_joint()
        self.machine = LinuxCNCMachineInterface()
        self.stat = self.fake_linuxcnc._last_stat
        self.axial = LinuxCNCAxialInterface(self.joint, self.machine)

    def test_homing_completes_and_trusts_configured_extended_distance(self):
        self.axial.begin_homing_wait(timeout=5.0)
        self.stat.joint[self.joint.joint]["homed"] = True

        self.assertTrue(self.axial.poll_homing())
        self.assertTrue(self.axial.has_axis_been_homed())
        self.assertEqual(self.axial.max_limit["native"], self.joint.extended_distance)


if __name__ == "__main__":
    unittest.main()
