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

        # retracted_distance/extended_distance are native units (inches for
        # a linear joint -- see JointConfiguration's docstring), but
        # retract()/poll() command through axial_interface.move() using the
        # same machine-unit (mm) space as Axis.move()/ToolpathOperation --
        # using the native value directly here previously sent e.g. X's
        # retracted_distance=0.25 (inches) as a G21 target of 0.25mm
        # (=0.0098in), 25x short of the intended 0.25in standoff and well
        # below MIN_LIMIT, which LinuxCNC rejected outright as beyond the
        # negative limit. A non-flip joint's target is also exactly its own
        # MIN_LIMIT (retracted_distance) once converted -- nudging it a hair
        # inside the limit (same margin/reasoning as
        # LinuxCNCAxialInterface._restated_position_machine_units()) avoids
        # an ordinary floating-point round-trip landing a hair outside it,
        # with no room to absorb that since the value already sits exactly
        # on the limit.
        if joint_config.flip_retraction:
            native_retract_target = joint_config.extended_distance
        else:
            native_retract_target = joint_config.retracted_distance

        if native_retract_target is None:
            self.retracted_position = None
        else:
            margin = LinuxCNCAxialInterface._RESTATED_AXIS_SAFETY_MARGIN_MM
            converted = self.axial_interface.to_machine_units(native_retract_target)
            self.retracted_position = (
                converted - margin if joint_config.flip_retraction else converted + margin
            )

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

        # HOMING is excluded: LinuxCNC's own native homing (the only homing
        # this project ever does -- once, at startup) deliberately drives
        # onto this joint's own negative limit switch to find it.
        # RETRACTING is a plain commanded move to a configured soft limit
        # (retracted_position -- see retract() below), never a switch seek,
        # so a hard-limit trip during it is a genuine fault, same as MOVING.
        if (
            self.status.state not in (AxisState.FAULT, AxisState.HOMING)
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
                self.logger.info(
                    f"{self.axis}: Waiting for LinuxCNC's native Home All to home this joint."
                )
                self.axial_interface.begin_homing_wait()
                return

            try:
                self.axial_interface.poll_homing()

            except (RuntimeError, TimeoutError) as e:
                self.status.fault = True
                self.status.state = AxisState.FAULT
                self.logger.error(f"{self.axis}: Homing failed: {e}")

        # Retract completion (see retract()/is_retracted()) -- a plain
        # commanded move to retracted_position, not a re-home: this project
        # only ever homes once, at startup.
        elif self.status.state == AxisState.RETRACTING:
            try:
                if not self._retract_command_issued:
                    if not self.axial_interface.has_axis_been_homed():
                        raise RuntimeError(
                            f"{self.axis}: Axis has not been homed; cannot retract."
                        )
                    if self.retracted_position is None:
                        raise RuntimeError(
                            f"{self.axis}: No retract position configured; cannot move to retract position."
                        )
                    self.axial_interface.move(position=self.retracted_position, feed=1000)
                    self._retract_command_issued = True
                    return

                if self.axial_interface.is_axis_in_position():
                    self._retracted = True
                    self.status.state = AxisState.READY
                    self.logger.info(f"{self.axis}: retraction complete.")
                    return

            except (RuntimeError, TimeoutError) as e:
                self.status.fault = True
                self.status.state = AxisState.FAULT
                self.logger.error(f"{self.axis}: Retract failed: {e}")

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

    def retract(self):
        """
        Command this axis to retract: a plain commanded move to
        self.retracted_position (retracted_distance, or extended_distance
        if this joint's axis.json sets flip_retraction -- see __init__).
        Never re-homes or re-seeks a limit switch -- this project only ever
        homes once, at startup.
        """
        self.status.state = AxisState.RETRACTING
        self._retract_command_issued = False
        self._retracted = False
        if self.axial_interface.joint.flip_retraction:
            self.logger.info(f"{self.axis}: Retracting to extended_distance (flip_retraction)")
        else:
            self.logger.info(f"{self.axis}: Retracting to retracted_distance")

    def is_retracted(self):
        """
        True once the most recent retract() call has completed.
        Reset to False by the next retract() call.
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
