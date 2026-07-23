from __future__ import annotations
from lcaf.utils.joint_configuration import JointConfiguration

import linuxcnc
import hal
import time


class LinuxCNCMachineInterface:
    """
    Single shared connection to a running LinuxCNC instance.

    LinuxCNC exposes one command channel, one status channel, and one error
    channel for the whole machine (NML), not one per joint. This object owns
    that single connection; every LinuxCNCAxialInterface is handed a
    reference to it instead of opening its own.

    This is also the object MotionCoordinator publishes as `self.interface`
    for ForgeBrain (docs/state_machine.md) to query machine-wide state:
    update(), estop(), machine_on(), machine_on_command(), all_homed().
    """

    def __init__(self):
        self.command = linuxcnc.command()
        self.status = linuxcnc.stat()
        self.error = linuxcnc.error_channel()

        self._axes = []

    def register_axes(self, axes):
        """
        Give the machine interface the set of Axis objects it should
        consult for all_homed(). Homing in this project is performed in
        software (see LinuxCNCAxialInterface.home_axis) rather than via
        LinuxCNC's native homing sequence, so status.homed cannot be used
        as the source of truth.
        """

        self._axes = list(axes)

    # Status update
    def poll(self):
        self.status.poll()

    def update(self):
        self.poll()

    # Machine-wide queries
    def machine_on(self):
        return self.status.task_state == linuxcnc.STATE_ON

    def estop(self):
        return bool(self.status.estop)

    def all_homed(self):
        if not self._axes:
            return False

        return all(axis.is_homed() for axis in self._axes)

    # Machine-wide commands
    def machine_on_command(self):
        self.command.state(linuxcnc.STATE_ON)

    def machine_off(self):
        self.command.state(linuxcnc.STATE_OFF)

    def estop_command(self):
        self.command.state(linuxcnc.STATE_ESTOP)

    def estop_reset(self):
        self.command.state(linuxcnc.STATE_ESTOP_RESET)

    def abort(self):
        self.command.abort()

    def get_errors(self):
        errors = []

        while True:
            error = self.error.poll()

            if error is None:
                break

            errors.append(error)

        return errors


class LinuxCNCAxialInterface:

    def __init__(self, joint: JointConfiguration, machine: LinuxCNCMachineInterface):

        self.joint = joint
        self.machine = machine

        # 1 inch = 25.4 mm
        self.command = machine.command
        self.status = machine.status
        self.error = machine.error

        self.min_limit = {"inches": 0.0, "mm": 0.0}
        self.max_limit = {"inches": 0.0, "mm": 0.0}

        self.position_offset_to_native = 0.0

        self.axis_homed = False
        self.homing_ever_been_intialized = False

        # Homing command needs to check whether the min axis position
        # limit switch is active. If it is active it should not move
        # in the negative direction it would then move in the positive direction
        # until the maximum limit switch is activated.
        # It then sets the min position as zero
        # and the max position as however far the distance between them is.
        # It should pull the hal configuration from linux cnc to know
        # Basically the gear ratio and how many turns or step signals converts into what distance
        # But it should not assume the set distance or length of the axis provided by the user is correct
        # it finds that distance with the limit switches.
        #
        # During homing this interface dynamically updates:
        #
        #     self.min_limit
        #     self.max_limit["mm"]
        #     self.max_limit["inches"]
        #
        # using the measured travel of the axis.

    # Status update
    def poll(self):
        self.status.poll()

    def program_running(self):
        return self.status.interp_state == linuxcnc.INTERP_READING

    # Machine-wide control (estop/on/off/abort) lives on LinuxCNCMachineInterface;
    # use self.machine for that rather than duplicating it per axis.

    def soft_stop(self):
        self.command.jog(linuxcnc.JOG_STOP, True, self.joint.joint)

    # axis status getter functions
    def get_linuxcnc_native_position(self):
        self.poll()
        return self.status.position[self.joint.joint]

    def get_position(self):
        return self.get_linuxcnc_native_position() - self.position_offset_to_native

    def get_velocity(self):
        self.poll()
        return self.status.joint[self.joint.joint]["velocity"]

    def has_axis_been_homed(self):
        self.poll()
        return self.axis_homed

    def has_homing_ever_been_intialized(self):
        self.poll()
        return self.homing_ever_been_intialized

    def is_axis_enabled(self):
        self.poll()
        return bool(self.status.joint[self.joint.joint]["enabled"])

    def is_axis_in_position(self):
        self.poll()
        return self.status.joint[self.joint.joint]["inpos"]

    def is_position_in_range(self):
        return (self.min_limit["mm"] <= self.get_position() <= self.max_limit["mm"])

    def get_targeted_position(self):
        self.poll()
        return self.status.commanded_position[self.joint.joint]

    # homing
    def home_axis(self, speed: float = 1.0, timeout: float = 30.0):
        self.homing_ever_been_intialized = True
        self.poll()

        if not self.joint.has_limit_switches:
            # Continuous/switchless joint (e.g. an unlimited rotary axis):
            # there is nothing to seek. Zero at the current position and
            # trust the configured static soft limits (JointConfiguration
            # guarantees these are set when has_limit_switches is False).
            assert self.joint.soft_min_mm is not None and self.joint.soft_max_mm is not None

            self.position_offset_to_native = self.get_linuxcnc_native_position()
            self.min_limit["mm"] = self.joint.soft_min_mm
            self.max_limit["mm"] = self.joint.soft_max_mm
            self.min_limit["inches"] = self.joint.soft_min_mm / 25.4
            self.max_limit["inches"] = self.joint.soft_max_mm / 25.4
            self.axis_homed = True
            return

        if self.min_limit_active():
            self.position_offset_to_native = self.get_linuxcnc_native_position()

            try:

                self.jog_positive(speed)

                start_time = time.monotonic()

                while not self.max_limit_active():

                    self.poll()

                    if time.monotonic() - start_time > timeout:
                        raise TimeoutError(
                            "Timed out waiting for the maximum limit switch."
                        )
            finally:
                self.soft_stop()

            self.max_limit["mm"] = self.get_position()
            self.max_limit["inches"] = self.max_limit["mm"] / 25.4
            self.min_limit["mm"] = 0.0
            self.min_limit["inches"] = 0.0

        else:

            try:
                self.jog_negative(speed)

                start_time = time.monotonic()

                while not self.min_limit_active():
                    self.poll()

                    if time.monotonic() - start_time > timeout:
                        raise TimeoutError(
                            "Timed out waiting for the minimum limit switch."
                        )
            finally:
                self.soft_stop()

            return self.home_axis(speed=speed, timeout=timeout)

        self.axis_homed = True

    # MDI
    def execute_mdi(self, mdi):
        self.command.mode(linuxcnc.MODE_MDI)
        self.command.wait_complete()
        self.command.mdi(mdi)
        # no wait_complete here

    def move(self, position: float, feed: float = 1000):
        mdi = f"G1 {self.joint.axis}{position:.4f}F{feed}"

        self.execute_mdi(mdi)

    def dwell(self, seconds):
        self.execute_mdi(f"G4 P{seconds}")

    def min_limit_active(self):
        return hal.get_value(self.joint.negative_limit_hal_pin)

    def max_limit_active(self):
        return hal.get_value(self.joint.positive_limit_hal_pin)

    def jog_positive(self, speed):
        self.command.mode(linuxcnc.MODE_MANUAL)
        self.command.wait_complete()
        self.command.jog(linuxcnc.JOG_CONTINUOUS, True, self.joint.joint, speed)

    def jog_negative(self, speed):
        self.command.mode(linuxcnc.MODE_MANUAL)
        self.command.wait_complete()
        self.command.jog(linuxcnc.JOG_CONTINUOUS, True, self.joint.joint, -speed)

    def is_idle(self, velocity_tolerance: float = 1e-6):
        self.poll()

        joint = self.status.joint[self.joint.joint]

        return (
            joint["inpos"] and
            abs(joint["velocity"]) <= velocity_tolerance
        )
