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


class MultiAxisMdiWordTests(unittest.TestCase):
    """
    LinuxCNCAxialInterface.move() must spell out every registered axis's
    own word explicitly (at its own real current position), not just the
    one axis actually being commanded. Real-hardware testing found that a
    bare single-axis MDI line (e.g. "G1 Y1.5000") gets its *other* axis
    words filled in by whatever position the RS274NGC interpreter itself
    last tracked -- not reliably synced with the actual machine position
    established by native homing/jogging -- and LinuxCNC rejected the
    entire line ("would exceed joint N's negative limit" / "invalid params
    in linear command") for axes that were never being commanded to move
    at all. See LinuxCNCAxialInterface.move()'s comment.
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
        self.command = self.fake_linuxcnc._last_command

        for axis in self.motion.axes.values():
            axis.axial_interface.axis_homed = True
            # Mirrors what poll_homing() sets on real success -- Axis.move()
            # gates on is_position_in_range(), which needs these populated
            # (they default to 0.0/0.0 otherwise, since the axis_homed
            # shortcut above bypasses the real homing flow that would
            # normally set them).
            axis.axial_interface._set_homed_limits()

        # Distinct native-unit positions per joint so each one's expected
        # machine-units word is unambiguous below. None of these sit exactly
        # on a soft limit -- see RestatedSiblingSafetyMarginTests for that
        # boundary case specifically.
        self.stat.joint_actual_position[self.motion.axes["x"].joint] = 0.5  # in -> 12.7 mm
        self.stat.joint_actual_position[self.motion.axes["y"].joint] = 1.5  # in -> 38.1 mm
        self.stat.joint_actual_position[self.motion.axes["z"].joint] = 0.3  # in -> 7.62 mm
        self.stat.joint_actual_position[self.motion.axes["a"].joint] = 45.0  # deg, unconverted

    def tearDown(self):
        self._tmpdir.cleanup()

    def last_mdi_line(self) -> str:
        mdi_calls = [c for c in self.command.calls if c[0] == "mdi"]
        self.assertTrue(mdi_calls, "expected at least one mdi() call")
        return mdi_calls[-1][1]

    def test_move_axis_includes_every_other_axis_at_its_real_position(self):
        self.motion.move_axis("y", 50.0)  # within Y's [6.35, 63.5] mm range

        mdi = self.last_mdi_line()
        self.assertIn("X12.7000", mdi)
        self.assertIn("Y50.0000", mdi)
        self.assertIn("Z7.6200", mdi)
        self.assertIn("A45.0000", mdi)

    def test_moving_a_different_axis_still_restates_every_sibling(self):
        self.motion.move_axis("z", 50.0)

        mdi = self.last_mdi_line()
        self.assertIn("X12.7000", mdi)
        self.assertIn("Y38.1000", mdi)
        self.assertIn("Z50.0000", mdi)
        self.assertIn("A45.0000", mdi)


class RestatedSiblingSafetyMarginTests(unittest.TestCase):
    """
    A sibling axis's own real position is most often *exactly* its
    retracted_distance right after a home/retract -- which is also exactly
    its own MIN_LIMIT. Real-hardware testing found that restating that
    value verbatim in a G-code word (native inches -> mm string ->
    LinuxCNC's own mm-to-native conversion) can round-trip a hair below
    MIN_LIMIT from ordinary floating-point noise, with no margin to absorb
    it since the value already sits exactly on the limit -- LinuxCNC then
    rejected the entire multi-axis line, not just that one word. See
    LinuxCNCAxialInterface._restated_position_machine_units().
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
        self.command = self.fake_linuxcnc._last_command

        for axis in self.motion.axes.values():
            axis.axial_interface.axis_homed = True
            axis.axial_interface._set_homed_limits()

    def tearDown(self):
        self._tmpdir.cleanup()

    def last_mdi_line(self) -> str:
        mdi_calls = [c for c in self.command.calls if c[0] == "mdi"]
        self.assertTrue(mdi_calls, "expected at least one mdi() call")
        return mdi_calls[-1][1]

    def test_sibling_exactly_at_retracted_distance_is_nudged_off_the_boundary(self):
        z_axis = self.motion.axes["z"].axial_interface
        # Exactly retracted_distance (0.25 in for Z, see axis.json) -- the
        # parked position every switched joint sits at right after a
        # home/retract, and also exactly Z's own MIN_LIMIT.
        self.stat.joint_actual_position[z_axis.joint.joint] = z_axis.joint.retracted_distance

        self.motion.move_axis("y", 50.0)

        mdi = self.last_mdi_line()
        self.assertNotIn("Z6.3500", mdi)
        self.assertIn("Z6.3600", mdi)

    def test_sibling_exactly_at_extended_distance_is_nudged_off_the_boundary(self):
        y_axis = self.motion.axes["y"].axial_interface
        # Exactly extended_distance (2.5 in for Y, see axis.json) -- Y's own
        # MAX_LIMIT.
        self.stat.joint_actual_position[y_axis.joint.joint] = y_axis.joint.extended_distance

        self.motion.move_axis("z", 50.0)

        mdi = self.last_mdi_line()
        self.assertNotIn("Y63.5000", mdi)
        self.assertIn("Y63.4900", mdi)


class ConcurrentHomingTests(unittest.TestCase):
    """
    home_all() homes X, Y, Z, and A all through a single native "Home All"
    (command.home(-1), issued once machine-wide by
    LinuxCNCMachineInterface.home_all_command() -- the exact same command
    the Axis GUI's own "Home All" button sends -- see
    MotionCoordinator.home_all()'s docstring), not a separate
    command.home(joint) per joint. Each axis then independently backs off
    to its own configured retracted_distance as part of LinuxCNC's own
    native homing sequence (generate_ini() sets
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

    def test_home_all_issues_a_single_native_home_all(self):
        self.motion.home_all()

        for axis_name in ("x", "y", "z", "a"):
            self.assertEqual(self.motion.axes[axis_name].status.state, AxisState.HOMING)

        # home_all() itself issues the one machine-wide command.home(-1) --
        # not deferred to a later poll(), and not one call per joint.
        home_calls = [c for c in self.command.calls if c[0] == "home"]
        self.assertEqual(len(home_calls), 1)
        self.assertEqual(home_calls[0][1], -1)

        # A single poll() must start every axis's own wait on that one
        # shared command -- none deferred until another axis finishes, and
        # none issuing any further command of their own.
        self.motion.poll()

        for axis_name in ("x", "y", "z", "a"):
            self.assertEqual(self.motion.axes[axis_name].axial_interface._homing_phase, "native_wait")

        home_calls = [c for c in self.command.calls if c[0] == "home"]
        self.assertEqual(len(home_calls), 1)

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
