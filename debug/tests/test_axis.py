from __future__ import annotations

import sys
import unittest

from lcaf.control.axis import Axis, AxisState
from lcaf.control.linuxcnc_interface import LinuxCNCMachineInterface
from lcaf.utils.joint_configuration import JointConfiguration


def make_switched_joint(
    joint: int = 0, axis: str = "X", flip_retraction: bool = False
) -> JointConfiguration:
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
        flip_retraction=flip_retraction,
    )


class HardLimitFaultDetectionTests(unittest.TestCase):
    """
    A hard-limit-switch trip disables every joint's enable output
    machine-wide in real LinuxCNC (see docs/hardware_setup.md), but never
    touches status.joint[n]['fault'] (that's amp/following-error only --
    see LinuxCNCAxialInterface.is_faulted()). Without Axis.poll() checking
    is_on_hard_limit() separately, a real trip outside homing would leave
    the axis silently stuck instead of reporting FAULT.
    """

    def setUp(self):
        self.fake_linuxcnc = sys.modules["linuxcnc"]
        self.fake_hal = sys.modules["hal"]
        self.fake_hal.pins.clear()

        self.joint = make_switched_joint()
        self.machine = LinuxCNCMachineInterface()
        self.stat = self.fake_linuxcnc._last_stat
        self.axis = Axis(self.joint, self.machine)

        # First poll() takes UNINITIALIZED -> READY (stat.joint[n]['enabled']
        # defaults True in the fake).
        self.axis.poll()
        self.assertEqual(self.axis.status.state, AxisState.READY)

    def test_hard_limit_trip_outside_homing_faults_the_axis(self):
        self.stat.joint[self.joint.joint]["min_hard_limit"] = True

        self.axis.poll()

        self.assertEqual(self.axis.status.state, AxisState.FAULT)
        self.assertTrue(self.axis.has_fault())

    def test_hard_limit_true_during_homing_does_not_fault(self):
        self.axis.home()
        self.assertEqual(self.axis.status.state, AxisState.HOMING)

        # The joint is expected to be sitting on (or driving into) its own
        # switch during a deliberate native-homing seek.
        self.stat.joint[self.joint.joint]["min_hard_limit"] = True

        self.axis.poll()

        self.assertEqual(self.axis.status.state, AxisState.HOMING)
        self.assertFalse(self.axis.has_fault())

    def test_following_error_fault_is_unaffected_by_this_change(self):
        self.stat.joint[self.joint.joint]["fault"] = True

        self.axis.poll()

        self.assertEqual(self.axis.status.state, AxisState.FAULT)


class RetractionTests(unittest.TestCase):
    """
    Axis.retract() commands a plain move (through LinuxCNCAxialInterface.
    move(), mechanically identical to MOVING) to this axis's configured
    retract position -- self.retracted_position, set once in __init__ from
    JointConfiguration.retracted_distance/extended_distance/flip_retraction
    -- never a re-home or a limit-switch re-seek. See docs/hardware_setup.md
    section 7 ("Retract"). Requires the axis to have already completed
    HOMING at least once this session, and (unlike HOMING) a hard-limit
    trip while RETRACTING is a genuine fault, the same as MOVING.
    """

    def setUp(self):
        self.fake_linuxcnc = sys.modules["linuxcnc"]
        self.fake_hal = sys.modules["hal"]
        self.fake_hal.pins.clear()

        self.joint = make_switched_joint()
        self.machine = LinuxCNCMachineInterface()
        self.stat = self.fake_linuxcnc._last_stat
        self.axis = Axis(self.joint, self.machine)

        self.axis.poll()
        self.assertEqual(self.axis.status.state, AxisState.READY)

        # Simulate initial homing having already completed once this
        # session, without re-deriving the full seek-min/seek-max sequence
        # (already covered by test_linuxcnc_interface.py) -- retract() only
        # requires axis_homed to be True beforehand.
        self.axis.axial_interface.axis_homed = True

    def test_retract_targets_retracted_distance_by_default(self):
        # retracted_position is in machine units (mm), converted from
        # retracted_distance's native unit (inches) -- see __init__ -- plus
        # a small inward margin since retracted_distance is also exactly
        # this joint's own MIN_LIMIT.
        expected = self.axis.axial_interface.to_machine_units(self.joint.retracted_distance)
        self.assertAlmostEqual(self.axis.retracted_position, expected, delta=0.02)
        self.assertGreater(self.axis.retracted_position, expected)

    def test_retract_completes_once_joint_reports_in_position(self):
        self.stat.joint[self.joint.joint]["inpos"] = False

        self.axis.retract()
        self.assertEqual(self.axis.status.state, AxisState.RETRACTING)
        self.assertFalse(self.axis.is_retracted())

        # First poll() issues the plain commanded move.
        self.axis.poll()
        self.assertEqual(self.axis.status.state, AxisState.RETRACTING)
        self.assertFalse(self.axis.is_retracted())

        self.stat.joint[self.joint.joint]["inpos"] = True
        self.axis.poll()

        self.assertEqual(self.axis.status.state, AxisState.READY)
        self.assertTrue(self.axis.is_retracted())

    def test_hard_limit_true_during_retracting_faults(self):
        self.axis.retract()
        self.axis.poll()
        self.assertEqual(self.axis.status.state, AxisState.RETRACTING)

        self.stat.joint[self.joint.joint]["min_hard_limit"] = True
        self.axis.poll()

        self.assertEqual(self.axis.status.state, AxisState.FAULT)
        self.assertTrue(self.axis.has_fault())

    def test_retract_before_homing_faults_instead_of_moving(self):
        self.axis.axial_interface.axis_homed = False

        self.axis.retract()
        self.axis.poll()

        self.assertEqual(self.axis.status.state, AxisState.FAULT)
        call_names = [c[0] for c in self.fake_linuxcnc._last_command.calls]
        self.assertNotIn("mdi", call_names)

    def test_is_retracted_resets_on_next_retract_call(self):
        self.axis.retract()
        self.axis.poll()  # issues the move
        self.axis.poll()  # observes in-position -> retract complete
        self.assertTrue(self.axis.is_retracted())

        self.axis.retract()

        self.assertFalse(self.axis.is_retracted())
        self.assertEqual(self.axis.status.state, AxisState.RETRACTING)


class FlipRetractionTests(unittest.TestCase):
    """
    JointConfiguration.flip_retraction (this project's Y axis -- see
    configs/axis.json) changes what "retract" means for that one joint:
    it moves to extended_distance instead of retracted_distance. See
    docs/hardware_setup.md section 7 ("Retract").
    """

    def setUp(self):
        self.fake_linuxcnc = sys.modules["linuxcnc"]
        self.fake_hal = sys.modules["hal"]
        self.fake_hal.pins.clear()

        self.joint = make_switched_joint(joint=1, axis="Y", flip_retraction=True)
        self.machine = LinuxCNCMachineInterface()
        self.stat = self.fake_linuxcnc._last_stat
        self.axis = Axis(self.joint, self.machine)

        self.axis.poll()
        self.axis.axial_interface.axis_homed = True

    def test_retract_targets_extended_distance(self):
        # retracted_position is in machine units (mm), converted from
        # extended_distance's native unit (inches) -- see __init__ -- minus
        # a small inward margin since extended_distance is also exactly
        # this joint's own MAX_LIMIT.
        expected = self.axis.axial_interface.to_machine_units(self.joint.extended_distance)
        self.assertAlmostEqual(self.axis.retracted_position, expected, delta=0.02)
        self.assertLess(self.axis.retracted_position, expected)

        not_expected = self.axis.axial_interface.to_machine_units(self.joint.retracted_distance)
        self.assertNotAlmostEqual(self.axis.retracted_position, not_expected, delta=1.0)

    def test_retract_completes_once_joint_reports_in_position(self):
        self.stat.joint[self.joint.joint]["inpos"] = False

        self.axis.retract()
        self.axis.poll()  # issues the move to extended_distance
        self.assertEqual(self.axis.status.state, AxisState.RETRACTING)

        self.stat.joint[self.joint.joint]["inpos"] = True
        self.axis.poll()

        self.assertEqual(self.axis.status.state, AxisState.READY)
        self.assertTrue(self.axis.is_retracted())


if __name__ == "__main__":
    unittest.main()
