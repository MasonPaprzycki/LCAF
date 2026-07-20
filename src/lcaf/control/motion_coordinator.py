from enum import Enum, auto
from motor import Motor

from lcaf.toolpath.toolpath import ToolpathOperation
import logging
from typing import Optional
from linuxcnc_interface import LinuxCNCInterface

class MotionCoordinatorState(Enum):

    IDLE = auto()

    RETRACT_Z = auto()
    VERIFY_Z_RETRACTED = auto()

    RETRACT_XY = auto()
    VERIFY_XY_RETRACTED = auto()

    ROTATE_A = auto()
    VERIFY_ROTATION = auto()

    MOVE_XY = auto()
    VERIFY_XY_POSITION = auto()

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

    def __init__(self):
        self.logger = logging.getLogger("ForgeBrain.Motion")

        self.interface = LinuxCNCInterface()
    
        self.axes = {
            "x": Motor("x", 0, self.interface),
            "y": Motor("y", 1, self.interface),
            "z": Motor("z", 2, self.interface),
            "a": Motor("a", 3, self.interface)
        }

        self.state = MotionCoordinatorState.IDLE

        self.active_operation: Optional[ToolpathOperation] = None

        self.command_sent = False

        self.fault_message = ""

    def poll(self):
        """
        Poll all motor states.

        Do not log every poll cycle.
        Motor state changes/errors should be logged
        inside Motor.poll().
        """

        for name, motor in self.axes.items():

            try:
                motor.poll()

            except Exception as e:

                self.logger.exception(
                    f"Motor poll failure: axis={name}, error={e}"
                )

                raise


    def all_idle(self):
        """
        Check if all axes have completed motion.
        """

        busy_axes = []

        for name, motor in self.axes.items():

            if not motor.is_idle():
                busy_axes.append(name)

        if busy_axes:
            self.logger.debug(f"Motion busy: axes={busy_axes}")

            return False

        self.logger.debug("All axes idle")

        return True


    def emergency_stop(self):
        """
        Immediately stop all motors.
        """

        self.logger.warning("EMERGENCY STOP REQUESTED")

        for name, motor in self.axes.items():
            try:
                self.logger.warning(f"Emergency stop axis={name}")
                motor.emergency_stop()

            except Exception as e:
                self.logger.exception(f"Emergency stop failed: axis={name}, error={e}")


        idle = self.all_idle()

        if idle:
            self.logger.warning("Emergency stop complete: all axes idle")

        else:
            self.logger.error("Emergency stop incomplete: motors still active")


        return idle


    def move_axis(self, axis: str, position: float):
        """
        Command a single axis movement.
        """

        if axis not in self.axes:

            self.logger.error(f"Invalid axis command: axis={axis}")

            raise ValueError(f"Unknown axis {axis}")


        motor = self.axes[axis]

        self.logger.info(
            f"MOTION COMMAND: "
            f"axis={axis}, "
            f"target={position}"
        )

        try:

            motor.move_to(position)


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


    def home_all(self):
        """
        Home all machine axes.
        """

        self.logger.info("HOMING START: all axes")

        for name, motor in self.axes.items():
            try:
                self.logger.info(f"HOMING COMMAND: axis={name}")
                motor.home()

            except Exception as e:
                self.logger.exception(f"HOMING FAILED: axis={name}, error={e}")
                raise

        self.interface.home_all()

        self.logger.info( "HOMING COMMANDS ISSUED: all axes")

    def all_homed(self):

        return all(
            motor.is_homed()
            for motor in self.axes.values()
        )

    def homing_complete(self):
        """
        Check if all axes have completed homing.
        """
        return self.interface.all_homed()

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

    def update(self):

        if self.active_operation is None:
            return

        operation = self.active_operation

        # Retract Z
        if self.state == MotionCoordinatorState.RETRACT_Z:

            self.move_axis("z", 0.0)
            self.state = MotionCoordinatorState.VERIFY_Z_RETRACTED

            return

        # Verify Z
        if self.state == MotionCoordinatorState.VERIFY_Z_RETRACTED:
            if self.axes["z"].is_idle():
                self.state = MotionCoordinatorState.RETRACT_XY

            return

        # Retract XY
        if self.state == MotionCoordinatorState.RETRACT_XY:

            self.move_axis("x", 0.0)
            self.move_axis("y", 0.0)

            self.state = MotionCoordinatorState.VERIFY_XY_RETRACTED

            return

        # Verify XY Retract
        if self.state == MotionCoordinatorState.VERIFY_XY_RETRACTED:

            if (self.axes["x"].is_idle() and self.axes["y"].is_idle() ):
                self.state = MotionCoordinatorState.ROTATE_A

            return

        # Rotate
        if self.state == MotionCoordinatorState.ROTATE_A:

            self.move_axis("a", operation.rotation)

            self.state = MotionCoordinatorState.VERIFY_ROTATION

            return

        # Verify Rotation
        if self.state == MotionCoordinatorState.VERIFY_ROTATION:

            if self.axes["a"].is_idle():
                self.state = MotionCoordinatorState.MOVE_XY

            return

        # Move XY
        if self.state == MotionCoordinatorState.MOVE_XY:

            self.move_axis( "x", operation.x )
            self.move_axis( "y", operation.y )

            self.state = MotionCoordinatorState.VERIFY_XY_POSITION

            return

        # Verify XY
        if self.state == MotionCoordinatorState.VERIFY_XY_POSITION:

            if (self.axes["x"].is_idle() and self.axes["y"].is_idle()):
                self.state = MotionCoordinatorState.MOVE_Z

            return

        # Move Z
        if self.state == MotionCoordinatorState.MOVE_Z:
            self.move_axis( "z", operation.die_gap )
            self.state = MotionCoordinatorState.VERIFY_Z_POSITION

            return

        # Verify Z 
        if self.state == MotionCoordinatorState.VERIFY_Z_POSITION:
            if self.axes["z"].is_idle():
                self.state = MotionCoordinatorState.COMPLETE

            return

    
