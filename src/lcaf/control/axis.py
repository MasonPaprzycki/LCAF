from __future__ import annotations

import time
import logging

from enum import Enum, auto
from dataclasses import dataclass, field

from lcaf.utils.joint_configuration import JointConfiguration
from lcaf.control.linuxcnc_interface import LinuxCNCAxialInterface, LinuxCNCMachineInterface

class AxisState(Enum):
    UNINITIALIZED = auto()
    READY = auto()
    MOVING = auto()
    HOMING = auto()
    RETRACTING = auto()
    FAULT = auto()
    ESTOP = auto()

# Axis Status
@dataclass
class AxisStatus:

    axis: str
    fault: bool = False
    state: AxisState = AxisState.UNINITIALIZED
    last_update: float = field(default_factory=time.time)

class Axis:

    def __init__(self, joint_config: JointConfiguration, machine: LinuxCNCMachineInterface):
        self.axis = joint_config.axis.lower()
        self.joint = joint_config.joint
        self.axial_interface = LinuxCNCAxialInterface(joint_config, machine)

        self.status = AxisStatus(self.axis)
        self.logger = logging.getLogger(f"Motor-{self.axis}")

        self._retract_command_issued = False
        self._retracted = False

    def poll(self):

        self.axial_interface.poll()

        self.status.last_update = time.time()

        if self.status.state != AxisState.FAULT and self.axial_interface.is_faulted():
            self.status.fault = True
            self.status.state = AxisState.FAULT
            self.logger.error(
                f"{self.axis}: LinuxCNC reports this joint faulted "
                "(following error tripped -- see docs/potential_issues.md)."
            )

        # HOMING and RETRACTING are excluded: LinuxCNC's own native homing
        # and retract-to-zero both deliberately drive this joint onto its
        # own negative limit switch to find it (see
        # LinuxCNCAxialInterface.start_homing/start_retract_to_zero).
        # HOME_IGNORE_LIMITS (generate_ini()) keeps LinuxCNC's homing state
        # machine itself from faulting on that expected trip, but the raw
        # status.joint[n]['min_hard_limit']/['max_hard_limit'] fields
        # is_on_hard_limit() reads still reflect the physical pin regardless
        # -- so is_on_hard_limit() reading True during either phase here is
        # still expected, not a fault. Anywhere else, LinuxCNC reporting this
        # joint on a hard limit means every joint's enable output has just
        # been disabled machine-wide (see docs/hardware_setup.md) -- a different condition
        # from is_faulted() above (that only reflects following error, never
        # a hard-limit trip), so without this check it would go completely
        # unnoticed here.
        if (
            self.status.state not in (AxisState.FAULT, AxisState.HOMING, AxisState.RETRACTING)
            and self.axial_interface.is_on_hard_limit()
        ):
            self.status.fault = True
            self.status.state = AxisState.FAULT
            self.logger.error(
                f"{self.axis}: hard limit switch tripped outside of homing -- LinuxCNC has "
                "disabled every joint's enable output machine-wide and requires "
                "override_limits()+re-enable+re-home to recover (see docs/hardware_setup.md; "
                "no automated recovery path exists yet -- see docs/potential_issues.md)."
            )

        if self.status.state == AxisState.FAULT:
            return # do not continue state machine if fault is detected

        elif self.status.state == AxisState.UNINITIALIZED:
            if self.is_enabled():
                self.status.state = AxisState.READY
            return

        # Homing detection
        elif self.status.state == AxisState.HOMING:
            if self.axial_interface.has_axis_been_homed():
                self.status.state = AxisState.READY
                self.logger.info(f"{self.axis}: Has been homed.")
                return

            if not self.axial_interface.has_homing_ever_been_intialized():
                self.axial_interface.start_homing()
                return

            try:
                self.axial_interface.poll_homing()

            except (RuntimeError, TimeoutError) as e:
                self.status.fault = True
                self.status.state = AxisState.FAULT
                self.logger.error(f"{self.axis}: Homing failed: {e}")

        # Retract-to-zero detection (see retract_to_zero()/is_retracted())
        elif self.status.state == AxisState.RETRACTING:
            try:
                if not self._retract_command_issued:
                    self.axial_interface.start_retract_to_zero(
                        speed=self.axial_interface.joint.max_velocity
                    )
                    self._retract_command_issued = True
                    return

                if self.axial_interface.poll_retract_to_zero():
                    self._retracted = True
                    self.status.state = AxisState.READY
                    self.logger.info(f"{self.axis}: Retract-to-zero complete.")

            except (RuntimeError, TimeoutError) as e:
                self.status.fault = True
                self.status.state = AxisState.FAULT
                self.logger.error(f"{self.axis}: Retract-to-zero failed: {e}")

        # Motion completion
        elif self.status.state == AxisState.MOVING:
            if not self.axial_interface.has_axis_been_homed():
                self.logger.info(f"{self.axis}: Axis has not been homed so will not move.")
                return

            elif self.axial_interface.is_axis_in_position():
                self.status.state = AxisState.READY
                self.logger.info(f"{self.axis}: Motion complete.")
                return

    def home(self):
        self.status.state = AxisState.HOMING
        self.logger.info( f"{self.axis}: Homing")

    def is_homed(self):
        return self.axial_interface.has_axis_been_homed()

    def retract_to_zero(self):
        """
        Begin retracting this axis -- for almost every axis that means
        re-zeroing against its negative limit switch before the move,
        rather than trusting the commanded "0.0" position (see
        LinuxCNCAxialInterface.start_retract_to_zero() for why). This
        project's Y axis is the one exception (JointConfiguration.
        retract_to): mechanically its retracted position is partway into
        its positive-direction travel, so it instead moves to its
        configured retract_to position -- still routed through this same
        method/state, since LinuxCNCAxialInterface handles the distinction
        internally. Mirrors
        home(): MotionCoordinator calls this once per retract, then polls
        is_retracted() every heartbeat while poll() (above) advances it.
        """
        self.status.state = AxisState.RETRACTING
        self._retract_command_issued = False
        self._retracted = False
        if self.axial_interface.joint.retract_to is not None:
            self.logger.info(f"{self.axis}: Retracting (moving to configured retract position)")
        else:
            self.logger.info(f"{self.axis}: Retracting to zero (re-seeking negative limit switch)")

    def is_retracted(self):
        """
        True once the most recent retract_to_zero() call has completed.
        Reset to False by the next retract_to_zero() call.
        """
        return self._retracted

    def move(self, position, feed=1000):
        """
        position is in machine units (millimetres for a linear joint,
        degrees for the angular one -- see
        LinuxCNCAxialInterface.to_machine_units), matching what
        ToolpathOperation.x/y/die_gap/rotation and MotionCoordinator
        already command in.
        """
        if self.is_homed():
            if self.axial_interface.is_position_in_range(position):
                self.status.state = AxisState.MOVING
                self.axial_interface.move(position, feed)

            else:
                self.logger.error(
                    f"{self.axis}: Commanded position {position} is out of this joint's "
                    "homed range -- check forge configuration in toolpath generator."
                )
        else:
            self.logger.error(f"{self.axis}: Axis has not been homed. Home the axis first with the home() method.")

    def soft_stop(self):
        self.axial_interface.soft_stop()
        self.status.state = AxisState.READY
        self.logger.info(f"{self.axis}: Stop")

    def estop(self):
        """
        Mark this axis as ESTOP'd. E-Stop itself is a machine-wide LinuxCNC
        state (linuxcnc.STATE_ESTOP), not a per-joint one -- the actual
        hardware command is issued once via LinuxCNCMachineInterface by
        MotionCoordinator.emergency_stop(). This just updates bookkeeping.
        """
        self.status.state = AxisState.ESTOP
        self.logger.warning( f"{self.axis}: ESTOP")

    def is_estop_active(self):
        return self.status.state == AxisState.ESTOP

    def has_fault(self):
        return self.status.state == AxisState.FAULT

    def is_moving(self):
        return self.status.state == AxisState.MOVING

    def position(self):
        """Current position in machine units -- millimetres for a linear
        joint, degrees for the angular one (see
        LinuxCNCAxialInterface.to_machine_units). This is the same unit
        space MotionCoordinator compares commanded targets in."""
        return self.axial_interface.get_position_machine_units()

    def is_idle(self):
        return self.axial_interface.is_idle()

    def is_enabled(self):
        return self.axial_interface.is_axis_enabled()
