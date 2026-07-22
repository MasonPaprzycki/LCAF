from enum import Enum, auto
import logging
from typing import Optional

from lcaf.utils.toolpath import ToolpathOperation
from lcaf.control.linuxcnc_interface import LinuxCNCAxialInterface
from lcaf.control.axis import Axis

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

    def __init__(self):
        self.logger = logging.getLogger("ForgeBrain.Motion")
    
        self.axes = {
            "x": Axis("x", 0),
            "y": Axis("y", 1),
            "z": Axis("z", 2),
            "a": Axis("a", 3)
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

        for name, axis in self.axes.items():

            try:
                axis.poll()

            except Exception as e:

                self.logger.exception(
                    f"Axis poll failure: axis={name}, error={e}"
                )

                raise


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
    
    def is_axis_in_position(self, axis: str, position: float):
        if axis not in self.axes:
            self.logger.error(f"Invalid axis command: axis={axis}")
            raise ValueError(f"Unknown axis {axis}")

        if self.axes[axis].position() == position:
            return True
        else:
            return False


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

        for name, axis in self.axes.items():
            axis.home()

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

    def update(self):

        if self.active_operation is None:
            return

        operation = self.active_operation

        # Retract Z
        if self.state == MotionCoordinatorState.RETRACT_Z:
            self.move_axis("z", 0.0)
            self.state = MotionCoordinatorState.VERIFY_Z_RETRACTED

            return

        # Verify Z retracted
        if self.state == MotionCoordinatorState.VERIFY_Z_RETRACTED:
            if self.is_axis_in_position("z", 0.0) and self.all_idle():
                self.state = MotionCoordinatorState.RETRACT_Y

            return

        # Retract Y
        if self.state == MotionCoordinatorState.RETRACT_Y:
            self.move_axis("y", 0.0)
            self.state = MotionCoordinatorState.VERIFY_Y_RETRACTED

            return

        # Verify Y Retracted
        if self.state == MotionCoordinatorState.VERIFY_Y_RETRACTED:
            if self.is_axis_in_position("y", 0.0) and self.all_idle():
                self.state = MotionCoordinatorState.RETRACT_X

            return
        
         # Retract X
        if self.state == MotionCoordinatorState.RETRACT_X:
            self.move_axis("x", 0.0)
            self.state = MotionCoordinatorState.VERIFY_X_RETRACTED

            return

        # Verify X Retract
        if self.state == MotionCoordinatorState.VERIFY_X_RETRACTED:
            if self.is_axis_in_position("x", 0.0) and self.all_idle():
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
    
    def estop(self):
        for name, axis in self.axes.items():
            axis.estop()

        logging.log(logging.INFO, "EMERGENCY STOP ACTIVATED")

    def is_estop_active(self):
        for name, axis in self.axes.items():
            if not axis.is_estop_active():
                return False

        return True