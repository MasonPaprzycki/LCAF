from __future__ import annotations

import sys
import unittest

from lcaf.control.axis import Axis, AxisState
from lcaf.control.linuxcnc_interface import LinuxCNCMachineInterface
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


class RetractToZeroTests(unittest.TestCase):
    """
    Axis.retract_to_zero() re-seeks this axis's negative limit switch to
    re-zero it -- used by MotionCoordinator before every retract, not just
    once at home_all() -- see docs/hardware_setup.md section 7
    ("Retract-to-zero"). Requires the axis to have already completed
    initial homing at least once (LinuxCNCAxialInterface.axis_homed), and a
    hard-limit trip while RETRACTING must not fault the axis, the same way
    HOMING is already exempt.
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
        # (already covered by test_linuxcnc_interface.py) -- retract_to_zero
        # only requires axis_homed to be True beforehand.
        self.axis.axial_interface.axis_homed = True

    def test_retract_to_zero_completes_and_reports_retracted(self):
        self.axis.retract()
        self.assertEqual(self.axis.status.state, AxisState.RETRACTING)
        self.assertFalse(self.axis.is_retracted())

        # First poll() issues start_retract_to_zero() (re-runs native home()).
        self.axis.poll()
        self.assertEqual(self.axis.status.state, AxisState.RETRACTING)
        self.assertFalse(self.axis.is_retracted())

        self.stat.joint[self.joint.joint]["homed"] = True
        self.axis.poll()

        self.assertEqual(self.axis.status.state, AxisState.READY)
        self.assertTrue(self.axis.is_retracted())

    def test_hard_limit_true_during_retracting_does_not_fault(self):
        self.axis.retract()
        self.axis.poll()
        self.assertEqual(self.axis.status.state, AxisState.RETRACTING)

        self.stat.joint[self.joint.joint]["min_hard_limit"] = True
        self.axis.poll()

        self.assertEqual(self.axis.status.state, AxisState.RETRACTING)
        self.assertFalse(self.axis.has_fault())

    def test_is_retracted_resets_on_next_retract_call(self):
        self.axis.retract()
        self.axis.poll()
        self.stat.joint[self.joint.joint]["homed"] = True
        self.axis.poll()
        self.assertTrue(self.axis.is_retracted())

        self.stat.joint[self.joint.joint]["homed"] = False
        self.axis.retract()

        self.assertFalse(self.axis.is_retracted())
        self.assertEqual(self.axis.status.state, AxisState.RETRACTING)


if __name__ == "__main__":
    unittest.main()
