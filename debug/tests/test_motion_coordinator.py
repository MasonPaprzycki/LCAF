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
        self.fake_hal = sys.modules["hal"]
        self.fake_hal.pins.clear()

        self.machine_config = load_machine_configuration(REPO_ROOT / "configs" / "machine.json")
        self._tmpdir = tempfile.TemporaryDirectory()
        self.motion = MotionCoordinator(
            machine_config=self.machine_config,
            generated_config_dir=self._tmpdir.name,
        )

        # Simulate every switched axis having already completed initial
        # homing this session -- retract_to_zero() only requires that flag,
        # not a full re-derivation of the seek-min/seek-max sequence
        # (already covered by test_linuxcnc_interface.py).
        for axis in self.motion.axes.values():
            axis.axial_interface.axis_homed = True

    def tearDown(self):
        self._tmpdir.cleanup()

    def neg_pin(self, axis_name: str) -> str:
        return self.motion.axes[axis_name].axial_interface.joint.negative_limit_hal_pin

    def advance_retract(self, axis_name: str):
        """Drive one axis's RETRACTING state to completion via its own poll()."""
        self.motion.axes[axis_name].poll()
        self.fake_hal.pins[self.neg_pin(axis_name)] = True
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


if __name__ == "__main__":
    unittest.main()
