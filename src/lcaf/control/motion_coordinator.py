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

        # Sequential homing state -- see home_all()/_advance_sequential_homing().
        # _homing_order is empty and _homing_phase is None both before the
        # first home_all() call and after the whole sequence (every axis
        # homed AND retracted, one at a time) has completed.
        self._homing_order: list[str] = []
        self._homing_position = 0
        self._homing_phase: str | None = None

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

        self._advance_sequential_homing()

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
        Command a single axis to retract (Axis.retract_to_zero()) -- for
        almost every axis that means re-seeking its negative limit switch
        rather than trusting a commanded "0.0" position (see
        LinuxCNCAxialInterface.start_retract_to_zero() for why: these are
        open-loop stepper joints with no position feedback, so trusting an
        accumulated commanded position instead of re-referencing to the
        physical switch risks silent drift -- docs/potential_issues.md).
        This project's Y axis is the one exception
        (JointConfiguration.retract_to): it retracts to its configured
        retract_to position instead, since its retracted position is
        mechanically partway into its positive-direction travel -- see
        LinuxCNCAxialInterface.start_retract_to_zero().
        """

        if axis not in self.axes:

            self.logger.error(f"Invalid axis command: axis={axis}")

            raise ValueError(f"Unknown axis {axis}")


        self.logger.info(f"RETRACT COMMAND: axis={axis}")

        try:

            self.axes[axis].retract_to_zero()

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
        Home all machine axes sequentially -- one axis at a time, each
        immediately retracted to its retracted position as soon as it finds
        its home switch, before the next axis begins seeking its own switch.
        See _advance_sequential_homing(), which drives this every poll().

        This project always homes via LinuxCNC's own native homing sequence
        (see JointConfiguration/MachineConfiguration's homing note) --
        HOME_IGNORE_LIMITS makes each joint's own home-seek safe on its own,
        regardless of how many other joints are homing at the same time, so
        this one-at-a-time ordering isn't about avoiding a race. It exists
        because retracting each axis immediately after it homes -- rather
        than waiting for every axis to finish homing first -- is this
        project's own requirement (see _advance_sequential_homing()).
        """

        try:
            self._homing_order = sorted(self.axes, key=lambda name: self.axes[name].joint)
            self._homing_position = 0
            self._homing_phase = "homing"

            self.logger.info("HOMING START: all axes (sequential)")

            first_axis = self._homing_order[0]
            self.logger.info(f"HOMING COMMAND: axis={first_axis}")
            self.axes[first_axis].home()

        except Exception as e:
            self.logger.exception(f"HOMING FAILED: error={e}")
            raise

    def _advance_sequential_homing(self):
        """
        Called every poll() heartbeat while a home_all() sequence is in
        progress. Moves the current axis from "homing" to "retracting" once
        it finds its switch, then advances to the next axis's "homing" phase
        once that retract completes -- see home_all()'s docstring for why
        this must stay strictly one axis at a time. No-op once the sequence
        has completed (or before one has ever been started).
        """
        if self._homing_phase is None:
            return

        current_name = self._homing_order[self._homing_position]
        current_axis = self.axes[current_name]

        if current_axis.has_fault():
            return  # ForgeBrain/first_faulted_axis() handles reporting this.

        if self._homing_phase == "homing":
            if not current_axis.is_homed():
                return

            # A switchless joint (has_limit_switches=False, e.g. this
            # project's A) has no switch to re-seek -- start_retract_to_zero()
            # itself refuses that joint outright (RuntimeError -- see its
            # docstring), and there is nothing meaningful to retract to
            # anyway, since its "home" is just wherever it sat at boot. Skip
            # straight to the next axis instead of retracting.
            if not current_axis.axial_interface.joint.has_limit_switches:
                self.logger.info(
                    f"HOMING COMPLETE (no retract -- switchless joint): axis={current_name}"
                )
                self._advance_to_next_axis()
                return

            self.logger.info(f"HOMING COMPLETE, RETRACTING: axis={current_name}")
            current_axis.retract_to_zero()
            self._homing_phase = "retracting"
            return

        if self._homing_phase == "retracting" and current_axis.is_retracted():
            self._advance_to_next_axis()

    def _advance_to_next_axis(self):
        """
        Move from the current axis to the next one in _homing_order, or
        finish the sequence if this was the last one -- shared by both the
        normal retract-complete path and the switchless-joint skip in
        _advance_sequential_homing().
        """
        self._homing_position += 1

        if self._homing_position >= len(self._homing_order):
            self.logger.info("HOMING COMPLETE: all axes homed and retracted")
            self._homing_phase = None
            return

        next_name = self._homing_order[self._homing_position]
        self.logger.info(f"HOMING COMMAND: axis={next_name}")
        self.axes[next_name].home()
        self._homing_phase = "homing"

    def all_homed(self):
        """
        True once home_all()'s full sequence has completed -- every axis
        homed AND retracted, one at a time, in order. Deliberately not
        "every axis's own is_homed() is currently true": the last axis in
        the sequence reports is_homed() True as soon as its own homing seek
        finishes, before its own retract has even started -- see
        home_all()/_advance_sequential_homing().
        """
        return bool(self._homing_order) and self._homing_phase is None

    def homing_complete(self):
        """Alias for all_homed() -- see its docstring."""
        return self.all_homed()

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
