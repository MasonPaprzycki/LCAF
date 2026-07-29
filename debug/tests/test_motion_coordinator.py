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
        """Drive one axis's RETRACTING state to completion via its own poll()
        (native retract-to-zero re-runs command.home() -- see
        LinuxCNCAxialInterface.start_retract_to_zero())."""
        joint_num = self.motion.axes[axis_name].joint
        self.motion.axes[axis_name].poll()
        self.stat.joint[joint_num]["homed"] = True
        self.motion.axes[axis_name].poll()

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


class SequentialHomingTests(unittest.TestCase):
    """
    home_all() must home and retract exactly one axis at a time (joint 0,
    1, 2, 3 in order -- here x, y, z, a), immediately retracting each axis
    once it homes rather than deferring every retract until the whole
    machine has homed -- see MotionCoordinator.home_all()'s docstring. A's
    has_limit_switches=False (configs/axis.json) means it must be skipped
    straight to "done" with no retract attempt (start_retract_to_zero()
    refuses a switchless joint outright).
    """

    _MAX_TICKS_PER_AXIS = 8

    def setUp(self):
        self.fake_linuxcnc = sys.modules["linuxcnc"]

        self.machine_config = load_machine_configuration(REPO_ROOT / "configs" / "machine.json")
        self._tmpdir = tempfile.TemporaryDirectory()
        self.motion = MotionCoordinator(
            machine_config=self.machine_config,
            generated_config_dir=self._tmpdir.name,
        )
        self.stat = self.fake_linuxcnc._last_stat

    def tearDown(self):
        self._tmpdir.cleanup()

    def run_axis_to_completion(self, axis_name: str):
        """
        Poll axis_name's native homing (then, for a switched joint, its
        native retract-to-zero) to completion, feeding the fake stat's
        'homed' flag whenever this project has a fresh command.home() in
        flight for it (LinuxCNCAxialInterface's own _homing_phase/
        _retract_phase == "native_wait") and clearing it otherwise, so each
        fresh command.home() is genuinely observed as not-yet-complete
        before completing it -- mirrors what real LinuxCNC reports while a
        home sequence is actually in progress.
        """
        axis = self.motion.axes[axis_name]
        joint_num = axis.joint

        for _ in range(self._MAX_TICKS_PER_AXIS):
            axial = axis.axial_interface
            if axial._homing_phase == "native_wait" or axial._retract_phase == "native_wait":
                self.stat.joint[joint_num]["homed"] = True
            else:
                self.stat.joint[joint_num]["homed"] = False

            self.motion.poll()

            if axis.is_homed() and (not axial.joint.has_limit_switches or axis.is_retracted()):
                return

        self.fail(f"{axis_name} did not finish homing/retracting within {self._MAX_TICKS_PER_AXIS} ticks")

    def test_axes_home_one_at_a_time_in_joint_order(self):
        self.motion.home_all()

        for axis_name in ("x", "y", "z"):
            for other in self.motion.axes:
                if other != axis_name and not self.motion.axes[other].is_homed():
                    self.assertNotEqual(
                        self.motion.axes[other].status.state,
                        AxisState.HOMING,
                        f"{other} should not be homing while {axis_name} is still in progress",
                    )

            self.run_axis_to_completion(axis_name)
            self.assertTrue(self.motion.axes[axis_name].is_homed())
            # Immediately retracted -- not deferred until every axis homes.
            self.assertTrue(self.motion.axes[axis_name].is_retracted())

        # A is switchless -- homes instantly, no retract attempted or needed.
        self.run_axis_to_completion("a")
        self.assertTrue(self.motion.axes["a"].is_homed())
        self.assertFalse(self.motion.axes["a"].has_fault())

        self.assertTrue(self.motion.all_homed())

    def test_all_homed_false_until_last_axis_retracts(self):
        self.motion.home_all()

        for axis_name in ("x", "y", "z"):
            self.run_axis_to_completion(axis_name)
            self.assertFalse(self.motion.all_homed())

        # A reports is_homed() instantly, but all_homed() must not flip True
        # off that raw per-axis flag alone (see MotionCoordinator.all_homed()).
        self.run_axis_to_completion("a")
        self.assertTrue(self.motion.axes["a"].is_homed())
        self.assertTrue(self.motion.all_homed())


if __name__ == "__main__":
    unittest.main()
