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
        max_travel=10.0,
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
        self.machine = LinuxCNCMachineInterface(use_native_homing=False)
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
        # switch during a deliberate software-homing seek.
        self.stat.joint[self.joint.joint]["min_hard_limit"] = True

        self.axis.poll()

        self.assertEqual(self.axis.status.state, AxisState.HOMING)
        self.assertFalse(self.axis.has_fault())

    def test_following_error_fault_is_unaffected_by_this_change(self):
        self.stat.joint[self.joint.joint]["fault"] = True

        self.axis.poll()

        self.assertEqual(self.axis.status.state, AxisState.FAULT)


if __name__ == "__main__":
    unittest.main()
