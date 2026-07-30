from __future__ import annotations

from datetime import datetime, timezone
from enum import Enum, auto
import json
import logging
from pathlib import Path
from typing import Optional

from lcaf.utils.toolpath import ToolpathOperation
from lcaf.utils.joint_configuration import (
    MachineConfiguration,
    load_machine_configuration,
    write_config_files,
)
from lcaf.control.linuxcnc_interface import LinuxCNCMachineInterface
from lcaf.control.axis import Axis

# Repo root's configs/ directory: src/lcaf/control/motion_coordinator.py -> configs/
_DEFAULT_CONFIG_DIR = Path(__file__).resolve().parents[3] / "configs"
_DEFAULT_MACHINE_CONFIG = _DEFAULT_CONFIG_DIR / "machine.json"
_DEFAULT_GENERATED_DIR = _DEFAULT_CONFIG_DIR / "generated"

class MotionCoordinatorState(Enum):

    IDLE = auto()

    RETRACT_Z = auto()
    VERIFY_Z_RETRACTED = auto()

    RETRACT_Y = auto()
    VERIFY_Y_RETRACTED = auto()

    RETRACT_X = auto()
    VERIFY_X_RETRACTED = auto()

    ROTATE_A = auto()
    VERIFY_ROTATION = auto()

    MOVE_X = auto()
    VERIFY_X_POSITION = auto()

    MOVE_Y = auto()
    VERIFY_Y_POSITION = auto()

    MOVE_Z = auto()
    VERIFY_Z_POSITION = auto()

    COMPLETE = auto()

    FAULT = auto()

class MotionCoordinator:

    """
    Coordinates all motor movement.

    LinuxCNC executes motion.

    MotionCoordinator only issues commands
    and checks completion.
    """

    def __init__(
        self,
        machine_config: MachineConfiguration | None = None,
        machine_config_path: str | Path = _DEFAULT_MACHINE_CONFIG,
        generated_config_dir: str | Path = _DEFAULT_GENERATED_DIR,
        simulate: bool = False,
    ):
        self.logger = logging.getLogger("ForgeBrain.Motion")

        if machine_config is None:
            machine_config = load_machine_configuration(machine_config_path)

        generated_config_dir = Path(generated_config_dir)

        # Regenerate the .hal/.ini pair LinuxCNC operates on so the joint
        # config below is always what's actually running on the machine.
        # simulate=True generates a software-loopback HAL with no physical
        # hardware (see generate_hal()) -- everything above this line is
        # otherwise identical, so this is a way to run this exact
        # MachineConfiguration against a real LinuxCNC instance for testing.
        write_config_files(machine_config, generated_config_dir, simulate=simulate)

        self.machine_config = machine_config

        self.interface = LinuxCNCMachineInterface()

        self.axes = {
            joint.axis.lower(): Axis(joint, self.interface)
            for joint in machine_config.joints
        }

        self.interface.register_axes(self.axes.values())

        self.state = MotionCoordinatorState.IDLE

        self.active_operation: Optional[ToolpathOperation] = None

        self.command_sent = False

        self.fault_message = ""

        # See _write_homed_properties()/_warn_if_not_homed() below: a
        # diagnostic record of the last successful homing, plus a startup
        # warning that this process instance has not homed yet. This file
        # is never read back to skip homing -- every process start still
        # requires a fresh home_all() (see docs/potential_issues.md on why
        # this machine can't detect being moved while unpowered).
        self.homed_properties_path = generated_config_dir / "homed_properties.json"
        self._homed_properties_written = False
        self._warn_if_not_homed()

    def _warn_if_not_homed(self):
        """
        Prompt a startup warning that this process instance has not
        completed homing yet -- true unconditionally right after
        construction, since home_all() hasn't had a chance to run. Distinct
        message depending on whether a homed_properties.json from some
        earlier session/boot exists, purely so the operator knows whether
        this machine has ever homed successfully at all.
        """
        if self.homed_properties_path.exists():
            self.logger.warning(
                f"{self.homed_properties_path} exists from a previous session, but this "
                "machine has NOT been homed yet in this session -- motion commands will be "
                "rejected until home_all() completes. (Homing is always required fresh on "
                "every boot; this file is a diagnostic record, not a homed-state cache.)"
            )
        else:
            self.logger.warning(
                f"No {self.homed_properties_path.name} found -- this machine has never "
                "completed homing. Motion commands will be rejected until home_all() completes."
            )

    def _write_homed_properties(self):
        """
        Persist a diagnostic record of the homing this process instance just
        completed: per-axis measured/configured travel limits and position
        at completion. Called once, from poll(), the first time
        homing_complete() becomes true. This is an audit/troubleshooting
        record only -- it is never read back to skip homing on a later boot
        (see docs/potential_issues.md).
        """
        properties = {
            "homed_at_unix": datetime.now(timezone.utc).timestamp(),
            "homed_at_iso": datetime.now(timezone.utc).isoformat(),
            "axes": {
                name: {
                    "joint": axis.joint,
                    "is_angular": axis.axial_interface.joint.is_angular,
                    "min_limit_native": axis.axial_interface.min_limit["native"],
                    "max_limit_native": axis.axial_interface.max_limit["native"],
                    "min_limit_mm": axis.axial_interface.min_limit["mm"],
                    "max_limit_mm": axis.axial_interface.max_limit["mm"],
                    "position_native": axis.axial_interface.get_position(),
                }
                for name, axis in self.axes.items()
            },
        }

        self.homed_properties_path.parent.mkdir(parents=True, exist_ok=True)
        self.homed_properties_path.write_text(json.dumps(properties, indent=2))
        self.logger.info(f"Wrote homing record to {self.homed_properties_path}")

    def poll(self):
        """
        Poll all motor states.

        Do not log every poll cycle.
        Motor state changes/errors should be logged
        inside Motor.poll().
        """

        self.interface.poll()

        # LinuxCNC's own NML error channel is where a rejected jog/home
        # command's real reason lands as text (e.g. "joint 0 exceeds
        # positive limit", "Can't jog joint 0, not enabled", "joint N on
        # limit switch error") -- get_errors() drains it, but nothing
        # previously logged what it drained, so a command LinuxCNC silently
        # refused (motors never move, no exception raised here) left no
        # trace in this project's own log. See docs/potential_issues.md.
        for error in self.interface.get_errors():
            self.logger.error(f"LinuxCNC error: {error}")

        for name, axis in self.axes.items():

            try:
                axis.poll()

            except Exception as e:

                self.logger.exception(
                    f"Axis poll failure: axis={name}, error={e}"
                )

                raise

        if not self._homed_properties_written and self.homing_complete():
            self._write_homed_properties()
            self._homed_properties_written = True


    def all_idle(self):
        """
        Check if all axes have completed motion.
        """

        busy_axes = []

        for name, axis in self.axes.items():

            if not axis.is_idle():
                busy_axes.append(name)

        if busy_axes:
            self.logger.debug(f"Motion busy: axes={busy_axes}")

            return False

        self.logger.debug("All axes idle")

        return True

    def _position_tolerance(self, axis: Axis) -> float:
        """
        How close axis.position() has to be to a commanded target to count
        as "arrived," in the same machine units axis.position() itself
        returns (millimetres for a linear joint, degrees for the angular
        one -- see Axis.position()). None of these joints have real
        position feedback (see docs/potential_issues.md) -- LinuxCNC's own
        MIN_FERROR (machine.json's linear_min_ferror_in/angular_min_ferror_deg,
        the same tolerance LinuxCNC itself uses to decide a joint has
        settled) is the most trustworthy number available for this, rather
        than inventing a separate one.
        """
        joint = axis.axial_interface.joint
        if joint.is_angular:
            return self.machine_config.angular_min_ferror_deg
        return self.machine_config.linear_min_ferror_in * 25.4

    def is_axis_in_position(self, axis: str, position: float):
        """
        True once LinuxCNC itself considers the joint settled (Axis.is_idle()
        -- joint.inpos plus a real velocity tolerance) AND axis.position() is
        within this joint's own configured MIN_FERROR of the requested
        target. Exact float equality never actually happens on a real (or
        simulated) trajectory, so this used to hang here forever -- see
        docs/potential_issues.md.
        """
        if axis not in self.axes:
            self.logger.error(f"Invalid axis command: axis={axis}")
            raise ValueError(f"Unknown axis {axis}")

        axis_obj = self.axes[axis]

        if not axis_obj.is_idle():
            return False

        tolerance = self._position_tolerance(axis_obj)
        return abs(axis_obj.position() - position) <= tolerance

    def first_faulted_axis(self) -> Optional[str]:
        """
        Name of the first axis currently in AxisState.FAULT, or None if none
        are faulted. Checked during operation execution (update() below) and
        by ForgeBrain during INITIALIZING (a homing failure also lands here,
        since LinuxCNCAxialInterface.poll_homing() failures are surfaced as
        Axis.FAULT -- see axis.py).
        """
        for name, axis in self.axes.items():
            if axis.has_fault():
                return name
        return None


    def move_axis(self, axis: str, position: float):
        """
        Command a single axis movement.
        """

        if axis not in self.axes:

            self.logger.error(f"Invalid axis command: axis={axis}")

            raise ValueError(f"Unknown axis {axis}")


        self.logger.info(
            f"MOTION COMMAND: "
            f"axis={axis}, "
            f"target={position}"
        )

        try:

            self.axes[axis].move(position)

            self.logger.info(
                f"MOTION ACCEPTED: "
                f"axis={axis}, "
                f"target={position}"
            )


        except Exception as e:

            self.logger.exception(
                f"MOTION COMMAND FAILED: "
                f"axis={axis}, "
                f"target={position}, "
                f"error={e}"
            )

            raise


    def retract_axis(self, axis: str):
        """
        Command a single axis to retract (Axis.retract()) to its configured
        retract position -- retracted_distance, or extended_distance if
        this joint's axis.json sets flip_retraction. Always a plain
        commanded move; this project only ever homes once, at startup.
        """

        if axis not in self.axes:

            self.logger.error(f"Invalid axis command: axis={axis}")

            raise ValueError(f"Unknown axis {axis}")


        self.logger.info(f"RETRACT COMMAND: axis={axis}")

        try:

            self.axes[axis].retract()

            self.logger.info(f"RETRACT ACCEPTED: axis={axis}")


        except Exception as e:

            self.logger.exception(
                f"RETRACT COMMAND FAILED: axis={axis}, error={e}"
            )

            raise

    def is_axis_retracted(self, axis: str) -> bool:
        """
        True once axis's most recent retract_axis() call has completed --
        see Axis.is_retracted().
        """

        if axis not in self.axes:
            self.logger.error(f"Invalid axis command: axis={axis}")
            raise ValueError(f"Unknown axis {axis}")

        return self.axes[axis].is_retracted()

    def home_all(self):
        """
        Home every axis through LinuxCNC's own native "Home All" -- a
        single command.home(-1) issued once, machine-wide
        (LinuxCNCMachineInterface.home_all_command()), the exact same
        command the Axis GUI's own "Home All" button sends. LinuxCNC's own
        task-level homing sequencer then drives every joint's search and
        backoff itself, honoring each joint's own [JOINT_n]HOME_SEQUENCE
        (generate_ini()) exactly the way that button does.

        This project previously re-issued a separate command.home(joint)
        call per joint instead, from each Axis's own lazily-triggered
        start_homing() -- removed: it was an unnecessary reimplementation
        of exactly what "Home All" already does natively, and didn't behave
        identically to the GUI button that operators use to confirm homing
        works at all (see docs/hardware_setup.md section 7).

        Each axis then just watches its own status.joint[n]['homed']
        (Axis.poll() -> LinuxCNCAxialInterface.begin_homing_wait()/
        poll_homing()) until LinuxCNC's own Home All finishes homing it --
        including backing it off to its own configured retracted_distance,
        entirely inside that same native sequence (see
        JointConfiguration.retracted_distance) -- independently and
        asynchronously per axis, with nothing further for this project's
        own code to command. Axis.is_homed() (and therefore all_homed()
        below) only becomes true once that backoff has finished too.
        """

        self.logger.info("HOMING START: Home All (LinuxCNC native)")

        self.interface.home_all_command()

        for name, motor in self.axes.items():
            motor.home()

        self.logger.info("HOMING COMMAND ISSUED: Home All")

        self.logger.info("HOMING COMMANDS ISSUED: all axes")

    def all_homed(self):
        return all(
            motor.is_homed()
            for motor in self.axes.values()
        )

    def homing_complete(self):
        """
        Check if all axes have completed homing.
        """
        for name, axis in self.axes.items():
            if not axis.is_homed():
                return False

        return True

    def start(self, operation: ToolpathOperation):

        if self.state != MotionCoordinatorState.IDLE:
            raise RuntimeError("Motion already active.")

        self.active_operation = operation
        self.command_sent = False
        self.state = MotionCoordinatorState.RETRACT_Z

    def is_complete(self):
        return self.state == MotionCoordinatorState.COMPLETE

    def has_fault(self):
        return self.state == MotionCoordinatorState.FAULT

    def reset(self):
        self.active_operation = None
        self.command_sent = False
        self.state = MotionCoordinatorState.IDLE

    def update(self, telemetry=None):

        if self.active_operation is None:
            return

        faulted_axis = self.first_faulted_axis()
        if faulted_axis is not None:
            self.fault_message = f"Axis '{faulted_axis}' faulted (following error)."
            self.logger.error(self.fault_message)
            self.state = MotionCoordinatorState.FAULT
            return

        operation = self.active_operation

        # Retract Z
        if self.state == MotionCoordinatorState.RETRACT_Z:
            self.retract_axis("z")
            self.state = MotionCoordinatorState.VERIFY_Z_RETRACTED

            return

        # Verify Z retracted
        if self.state == MotionCoordinatorState.VERIFY_Z_RETRACTED:
            if self.is_axis_retracted("z") and self.all_idle():
                self.state = MotionCoordinatorState.RETRACT_Y

            return

        # Retract Y
        if self.state == MotionCoordinatorState.RETRACT_Y:
            self.retract_axis("y")
            self.state = MotionCoordinatorState.VERIFY_Y_RETRACTED

            return

        # Verify Y Retracted
        if self.state == MotionCoordinatorState.VERIFY_Y_RETRACTED:
            if self.is_axis_retracted("y") and self.all_idle():
                self.state = MotionCoordinatorState.RETRACT_X

            return

         # Retract X
        if self.state == MotionCoordinatorState.RETRACT_X:
            self.retract_axis("x")
            self.state = MotionCoordinatorState.VERIFY_X_RETRACTED

            return

        # Verify X Retract
        if self.state == MotionCoordinatorState.VERIFY_X_RETRACTED:
            if self.is_axis_retracted("x") and self.all_idle():
                self.state = MotionCoordinatorState.ROTATE_A

            return

        # Rotate
        if self.state == MotionCoordinatorState.ROTATE_A:
            self.move_axis("a", operation.rotation)
            self.state = MotionCoordinatorState.VERIFY_ROTATION

            return

        # Verify Rotation
        if self.state == MotionCoordinatorState.VERIFY_ROTATION:
            if self.is_axis_in_position("a", operation.rotation) and self.all_idle():
                self.state = MotionCoordinatorState.MOVE_X

            return

        # Move X
        if self.state == MotionCoordinatorState.MOVE_X:
            self.move_axis( "x", operation.x )
            self.state = MotionCoordinatorState.VERIFY_X_POSITION

            return

        # Verify X
        if self.state == MotionCoordinatorState.VERIFY_X_POSITION:
            if self.is_axis_in_position("x", operation.x) and self.all_idle():
                self.state = MotionCoordinatorState.MOVE_Y

            return


        # Move Y
        if self.state == MotionCoordinatorState.MOVE_Y:
            self.move_axis( "y", operation.y )
            self.state = MotionCoordinatorState.VERIFY_Y_POSITION

            return

        # Verify Y
        if self.state == MotionCoordinatorState.VERIFY_Y_POSITION:
            if self.is_axis_in_position("y", operation.y) and self.all_idle():
                self.state = MotionCoordinatorState.MOVE_Z

            return

        # Move Z
        if self.state == MotionCoordinatorState.MOVE_Z:
            self.move_axis( "z", operation.die_gap )
            self.state = MotionCoordinatorState.VERIFY_Z_POSITION

            return

        # Verify Z
        if self.state == MotionCoordinatorState.VERIFY_Z_POSITION:
            if self.is_axis_in_position("z", operation.die_gap) and self.all_idle():
                self.state = MotionCoordinatorState.COMPLETE

            return

    def is_motion_enabled(self):
        for name, axis in self.axes.items():
            if axis.is_enabled():
                return True

        return False

    def emergency_stop(self):
        """
        Machine-wide E-Stop. Issued once through the shared LinuxCNC
        connection, then reflected into each Axis's bookkeeping state.
        """

        self.interface.estop_command()

        for axis in self.axes.values():
            axis.estop()

        self.logger.info("EMERGENCY STOP ACTIVATED")

    def is_estop_active(self):
        return self.interface.estop()
