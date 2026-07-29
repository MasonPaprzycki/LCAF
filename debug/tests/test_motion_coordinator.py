from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

from lcaf.control.axis import AxisState
from lcaf.control.motion_coordinator import MotionCoordinator, MotionCoordinatorState
from lcaf.utils.joint_configuration import load_machine_configuration
from lcaf.utils.toolpath import OperationType, ToolpathOperation

REPO_ROOT = Path(__file__).resolve().parents[2]


class RetractStatesUseRetractToZeroTests(unittest.TestCase):
    """
    MotionCoordinator's RETRACT_Z/RETRACT_Y/RETRACT_X states must re-seek
    each axis's own negative limit switch (Axis.retract_to_zero()) instead
    of commanding a plain move to a remembered "0.0" -- see
    docs/hardware_setup.md section 7 ("Retract-to-zero") and
    docs/state_machine.md's RETRACT_Z/RETRACT_XY definitions.
    """

    def setUp(self):
        self.fake_linuxcnc = sys.modules["linuxcnc"]

        self.machine_config = load_machine_configuration(REPO_ROOT / "configs" / "machine.json")
        self._tmpdir = tempfile.TemporaryDirectory()
        self.motion = MotionCoordinator(
            machine_config=self.machine_config,
            generated_config_dir=self._tmpdir.name,
        )
        self.stat = self.fake_linuxcnc._last_stat

        # Simulate every switched axis having already completed initial
        # homing this session -- retract_to_zero() only requires that flag,
        # not a full re-derivation of the native-homing sequence (already
        # covered by test_linuxcnc_interface.py).
        for axis in self.motion.axes.values():
            axis.axial_interface.axis_homed = True

    def tearDown(self):
        self._tmpdir.cleanup()

    def advance_retract(self, axis_name: str):
        """Drive one axis's RETRACTING state to completion via its own
        poll() (native retract-to-zero re-runs command.home() -- see
        LinuxCNCAxialInterface.start_retract_to_zero()). LinuxCNC's own
        final move-to-HOME (generate_ini() sets HOME=retracted_distance)
        already backs the joint off its switch before ever reporting
        homed=True, so setting the fake stat's 'homed' flag alone is
        enough -- no separate backoff step to drive here."""
        joint_num = self.motion.axes[axis_name].joint
        self.motion.axes[axis_name].poll()  # issues command.home() for retract
        self.stat.joint[joint_num]["homed"] = True
        self.motion.axes[axis_name].poll()  # observes homed -> retract complete

    def start_operation(self) -> ToolpathOperation:
        operation = ToolpathOperation(
            step=1,
            operation=OperationType.STRIKE,
            x=1.0,
            y=1.0,
            die_gap=0.5,
            rotation=10.0,
            target_temperature=0.0,
        )
        self.motion.start(operation)
        return operation

    def test_retract_z_puts_axis_into_retracting_not_moving(self):
        self.start_operation()

        self.motion.update()

        self.assertEqual(self.motion.axes["z"].status.state, AxisState.RETRACTING)
        self.assertEqual(self.motion.state, MotionCoordinatorState.VERIFY_Z_RETRACTED)

    def test_verify_retracted_waits_for_the_switch_before_advancing(self):
        self.start_operation()
        self.motion.update()

        # Not retracted yet -- must not advance early.
        self.motion.update()
        self.assertEqual(self.motion.state, MotionCoordinatorState.VERIFY_Z_RETRACTED)

        self.advance_retract("z")
        self.motion.update()
        self.assertEqual(self.motion.state, MotionCoordinatorState.RETRACT_Y)

    def test_full_retract_sequence_advances_through_z_y_x_to_rotate(self):
        self.start_operation()

        for axis_name, next_state in (
            ("z", MotionCoordinatorState.RETRACT_Y),
            ("y", MotionCoordinatorState.RETRACT_X),
            ("x", MotionCoordinatorState.ROTATE_A),
        ):
            self.motion.update()  # enters RETRACT_<axis>
            self.advance_retract(axis_name)
            self.motion.update()  # verifies + transitions
            self.assertEqual(self.motion.state, next_state)

    def test_retract_before_initial_homing_faults_instead_of_moving(self):
        # Undo the setUp shortcut -- a fresh joint that never homed this
        # session must refuse retract-to-zero (LinuxCNCAxialInterface.
        # start_retract_to_zero()), surfacing as a motion fault rather than
        # silently moving.
        self.motion.axes["z"].axial_interface.axis_homed = False
        self.start_operation()

        self.motion.update()
        self.motion.axes["z"].poll()

        self.assertEqual(self.motion.axes["z"].status.state, AxisState.FAULT)


class ConcurrentHomingTests(unittest.TestCase):
    """
    home_all() must home X, Y, and Z (and A) all at the same time, not one
    at a time -- LinuxCNC's own native homing per joint is independent and
    safe to run concurrently (HOME_IGNORE_LIMITS is evaluated inside
    LinuxCNC's own compiled homing state machine, per joint, not a shared
    reactive mechanism -- see MotionCoordinator.home_all()'s docstring).
    Each axis then backs off to its own configured retracted_distance as
    part of LinuxCNC's own native homing sequence (generate_ini() sets
    [JOINT_n]HOME=retracted_distance, so LinuxCNC's own final move-to-HOME
    step does this natively -- status.joint[n]['homed'] only becomes true
    once that move completes), fully independently of the other axes -- not
    gated on any of them.
    """

    _MAX_TICKS = 8

    def setUp(self):
        self.fake_linuxcnc = sys.modules["linuxcnc"]

        self.machine_config = load_machine_configuration(REPO_ROOT / "configs" / "machine.json")
        self._tmpdir = tempfile.TemporaryDirectory()
        self.motion = MotionCoordinator(
            machine_config=self.machine_config,
            generated_config_dir=self._tmpdir.name,
        )
        self.stat = self.fake_linuxcnc._last_stat
        self.command = self.fake_linuxcnc._last_command

    def tearDown(self):
        self._tmpdir.cleanup()

    def run_all_axes_to_completion(self):
        """
        Poll every axis's native homing to completion in lockstep --
        feeding each joint's own fake 'homed' flag whenever its own
        axial_interface reports a fresh command.home() in flight
        (_homing_phase == "native_wait"), so all four progress together
        rather than one waiting on another.
        """
        for _ in range(self._MAX_TICKS):
            for axis in self.motion.axes.values():
                joint_num = axis.joint
                if axis.axial_interface._homing_phase == "native_wait":
                    self.stat.joint[joint_num]["homed"] = True
                else:
                    self.stat.joint[joint_num]["homed"] = False

            self.motion.poll()

            if all(axis.is_homed() for axis in self.motion.axes.values()):
                # One more poll() so each Axis's own status.state (checked
                # at the top of Axis.poll(), before poll_homing() updates
                # axis_homed) has a chance to catch up to HOMING -> READY.
                self.motion.poll()
                return

        self.fail(f"not every axis finished homing within {self._MAX_TICKS} ticks")

    def test_home_all_issues_command_home_for_every_axis_in_one_poll(self):
        self.motion.home_all()

        for axis_name in ("x", "y", "z", "a"):
            self.assertEqual(self.motion.axes[axis_name].status.state, AxisState.HOMING)

        # A single poll() must start every axis's own command.home() at
        # once -- none deferred until another axis finishes.
        self.motion.poll()

        home_calls = [c for c in self.command.calls if c[0] == "home"]
        homed_joints = {c[1] for c in home_calls}
        expected_joints = {self.motion.axes[name].joint for name in ("x", "y", "z", "a")}
        self.assertEqual(homed_joints, expected_joints)

        for axis_name in ("x", "y", "z", "a"):
            self.assertEqual(self.motion.axes[axis_name].axial_interface._homing_phase, "native_wait")

    def test_all_axes_home_and_back_off_concurrently(self):
        self.motion.home_all()
        self.run_all_axes_to_completion()

        for axis_name in ("x", "y", "z"):
            self.assertTrue(self.motion.axes[axis_name].is_homed())
            self.assertEqual(self.motion.axes[axis_name].status.state, AxisState.READY)

        # A is switchless -- homes instantly, no backoff attempted or needed.
        self.assertTrue(self.motion.axes["a"].is_homed())
        self.assertFalse(self.motion.axes["a"].has_fault())

        self.assertTrue(self.motion.all_homed())


if __name__ == "__main__":
    unittest.main()
