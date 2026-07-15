"""
===============================================================================
ForgeBrain
-------------------------------------------------------------------------------
Highest level abstraction for the forging brain.

Author: Mason Paprzycki 
Version: 0.1
===============================================================================
"""

from __future__ import annotations

import json
import logging
import time
from dataclasses import dataclass, field
from pathlib import Path
from enum import Enum, auto
from typing import Dict
from typing import List
from typing import Optional
from typing import Any

from linuxcnc_interface import LinuxCNCInterface
from motor import Motor

class BrainState(Enum):

    INITIALIZING = auto()
    IDLE = auto()

    LOAD_OPERATION = auto()
    PLAN_OPERATION = auto()
    EXECUTE_OPERATION = auto()

    VERIFY_OPERATION = auto()      # verify completed operation
    COMPLETE_OPERATION = auto()    # bookkeeping

    ADAPT_OPERATION = auto()       # sensor analysis / adaptive logic
    NEXT_OPERATION = auto()

    COMPLETE_TOOLPATH = auto()

    FAULT = auto()
    SHUTDOWN = auto()


class MotionState(Enum):

    UNKNOWN = auto()
    READY = auto()
    MOVING = auto()
    COMPLETE = auto()
    ERROR = auto()

class ForgeMode(Enum):

    STARTUP = auto()
    POSITIONING = auto()
    STRIKING = auto()
    REHEATING = auto()
    INSPECTING = auto()
    WAITING = auto()
    COMPLETE = auto()
    FAULT = auto()


class OperationType(Enum):
    STRIKE = auto()
    REHEAT = auto()
    INSPECT = auto()
    MOVE_ONLY = auto()
    DWELL = auto()
    CUSTOM = auto()


class OperationPhase(Enum):
    START = auto()
    MOVE_XY = auto()
    WAIT_XY = auto()
    VERIFY_XY = auto()

    MOVE_A = auto()
    WAIT_A = auto()
    VERIFY_A = auto()

    MOVE_Z = auto()
    WAIT_Z = auto()
    VERIFY_Z = auto()

    RETRACT_Z = auto()
    WAIT_RETRACT = auto()
    VERIFY_RETRACT = auto()

    COMPLETE = auto()

@dataclass
class ToolpathOperation:

    step: int
    operation: OperationType

    x: float
    y: float
    die_gap: float
    rotation: float

    target_temperature: float

    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class SystemState:

    # Execution
    
    toolpath_loaded: bool = False
    active_operation: Optional[ToolpathOperation] = None
    brain_state: BrainState = BrainState.INITIALIZING
    forge_mode: ForgeMode = ForgeMode.STARTUP
    motion_state: MotionState = MotionState.UNKNOWN
    operation_phase: OperationPhase = OperationPhase.START

    machine_enabled: bool = False
    machine_homed: bool = False
    estop: bool = False

    current_step: int = 0
    completed_steps: int = 0

    last_update: float = 0.0

    # Estimated billet state
    billet_temperature: Optional[float] = None

    # Fault state
    fault_message: str = ""
    fault_active: bool = False

    # Runtime
    runtime_seconds: float = 0.0
    start_time: float = field(default_factory=time.time)

    # Logging
    last_event: str = ""

class ToolpathQueue:

    """
    Sequential JSONL execution queue.
    """

    def __init__(self):
        self.operations: List[ToolpathOperation] = []
        self.index = 0

    @property
    def finished(self):
        return self.index >= len(self.operations)

    def current(self):
        if self.finished:
            return None

        return self.operations[self.index]

    def advance(self):
        self.index += 1

    def reset(self):
        self.operations.clear()
        self.index = 0

class Telemetry:

    def __init__(self):
        self.subscribers = {}

    def subscribe(self, topic, callback):

        if topic not in self.subscribers:
            self.subscribers[topic] = []

        self.subscribers[topic].append(callback)

    def publish(self, topic, data):

        if topic not in self.subscribers:
            return

        for callback in self.subscribers[topic]:
            callback(data)

class SensorManager:

    def __init__(self):
        self.sensors = []

    def register(self, sensor):
        self.sensors.append(sensor)

    def update(self):
        for sensor in self.sensors:
            sensor.poll()

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

            self.logger.debug(
                f"Motion busy: axes={busy_axes}"
            )

            return False


        self.logger.debug(
            "All axes idle"
        )

        return True


    def emergency_stop(self):
        """
        Immediately stop all motors.
        """

        self.logger.warning(
            "EMERGENCY STOP REQUESTED"
        )


        for name, motor in self.axes.items():

            try:

                self.logger.warning(
                    f"Emergency stop axis={name}"
                )

                motor.emergency_stop()


            except Exception as e:

                self.logger.exception(
                    f"Emergency stop failed: axis={name}, error={e}"
                )


        idle = self.all_idle()


        if idle:

            self.logger.warning(
                "Emergency stop complete: all axes idle"
            )

        else:

            self.logger.error(
                "Emergency stop incomplete: motors still active"
            )


        return idle


    def move_axis(self, axis: str, position: float):
        """
        Command a single axis movement.
        """

        if axis not in self.axes:

            self.logger.error(
                f"Invalid axis command: axis={axis}"
            )

            raise ValueError(
                f"Unknown axis {axis}"
            )


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

        self.logger.info(
            "HOMING START: all axes"
        )


        for name, motor in self.axes.items():

            try:

                self.logger.info(
                    f"HOMING COMMAND: axis={name}"
                )

                motor.home()


            except Exception as e:

                self.logger.exception(
                    f"HOMING FAILED: axis={name}, error={e}"
                )

                raise


        self.logger.info(
            "HOMING COMMANDS ISSUED: all axes"
        )

class ForgeBrain:

    """
    High-level autonomous forging controller.
    """

    def __init__(self):

        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s | %(levelname)s | %(name)s | %(message)s"
        )

        self.logger = logging.getLogger("ForgeBrain")

        self.state = SystemState()
        self.state.last_update = time.time()
        self.queue = ToolpathQueue()
        self.telemetry = Telemetry()
        self.motion = MotionCoordinator()
        self.sensors = SensorManager()
        self.running = False
        self.command_history = []


        # Controller update frequency (Hz)
        self.update_rate = 50.0

        # Controller period (seconds)
        self.control_period = 1.0 / self.update_rate

        # Number of update cycles executed
        self.loop_count = 0

    def run(self):

        self.running = True

        self.log_event("ForgeBrain Started")

        while self.running:

            start = time.perf_counter()

            # Poll machine
            self.update()

            # Execute one state
            self.process_state_machine()

            # Publish telemetry
            self.publish()

            # Maintain loop frequency
            elapsed = time.perf_counter() - start

            sleep = self.control_period - elapsed

            if sleep > 0:

                time.sleep(sleep)

        self.log_event("ForgeBrain Shutdown")

    def update(self):

        """
        Poll every subsystem once.

        This should execute every controller cycle.
        """

        self.motion.poll()
        self.sensors.update()

        self.state.runtime_seconds = (
            time.time() - self.state.start_time
        )

        self.state.last_update = time.time()
        self.loop_count += 1

    def publish(self):

        self.telemetry.publish(
            "brain",
            self.state
        )

        self.telemetry.publish(
            "motion",
            self.state.motion_state
        )

        self.telemetry.publish(
            "forge",
            self.state.forge_mode
        )

    def log_event(self, message):
        self.state.last_event = message
        self.logger.info(message)

    def set_fault(self, message):
        self.state.fault_active = True
        self.state.fault_message = message

        self.set_brain_state(BrainState.FAULT, f"Fault message: {message}")


    def process_state_machine(self):
        state = self.state.brain_state

        if state == BrainState.INITIALIZING:
            self.initialize_machine()

        elif state == BrainState.IDLE:
            self.idle()

        elif state == BrainState.LOAD_OPERATION:
            self.load_operation()

        elif state == BrainState.PLAN_OPERATION:
            self.plan_operation()

        elif state == BrainState.EXECUTE_OPERATION:
            self.execute_operation()

        elif state == BrainState.VERIFY_OPERATION:
            self.verify_operation()

        elif state == BrainState.COMPLETE_OPERATION:
            self.complete_operation()

        elif state == BrainState.ADAPT_OPERATION:
            self.adapt_operation()

        elif state == BrainState.NEXT_OPERATION:
            self.next_operation()

        elif state == BrainState.COMPLETE_TOOLPATH:
            self.complete_toolpath()

        elif state == BrainState.FAULT:
            self.handle_fault()

        elif state == BrainState.SHUTDOWN:
            self.shutdown()

    def set_brain_state(self, state: BrainState, message: str = ""):
        if self.state.brain_state == state:
            return

        self.log_event(
            f"Brain state changed from "
            f"{self.state.brain_state.name} "
            f"to {state.name}"
        )

        if message:
            self.log_event(message)

        self.state.brain_state = state

    def set_operation_phase(self,phase: OperationPhase, message: str = ""):
        if self.state.operation_phase == phase:
            return

        operation = self.state.active_operation

        if operation:
            context = f"Step {operation.step}"
        else:
            context = "No active operation"

        self.log_event(
            f"{context}: Operation phase changed from "
            f"{self.state.operation_phase.name} "
            f"to {phase.name}"
        )

        if message:
            self.log_event(message)

        self.state.operation_phase = phase

    def set_motion_state(self, state: MotionState, message: str = ""):
        if self.state.motion_state == state:
            return

        self.log_event(
            f"Motion state changed from "
            f"{self.state.motion_state.name} "
            f"to {state.name}"
        )

        if message:
            self.log_event(message)

        self.state.motion_state = state

    def set_forge_mode(self, mode: ForgeMode, message: str = ""):
        if self.state.forge_mode == mode:
            return

        self.log_event(
            f"Forge mode changed from "
            f"{self.state.forge_mode.name} "
            f"to {mode.name}"
        )

        if message:
            self.log_event(message)

        self.state.forge_mode = mode

    

        
    def shutdown(self):
        self.running = False

    def load_jsonl(self, filename: str):

        """
        Load a JSONL forging program.

        Each line is one ToolpathOperation.
        """

        self.queue.reset()

        path = Path(filename)

        if not path.exists():
            raise FileNotFoundError(path)

        with open(path, "r") as file:

            for line in file:

                line = line.strip()

                if not line:
                    continue

                try:

                    data = json.loads(line)

                    operation = ToolpathOperation(

                        step=data["step"],

                        operation=OperationType[
                            data["operation"].upper()
                        ],

                        x=data.get("x", 0.0),

                        y=data.get("y", 0.0),

                        die_gap=data.get("die_gap", 0.0),

                        rotation=data.get("rotation", 0.0),

                        target_temperature=data.get(
                            "target_temperature",
                            0.0
                        ),

                        metadata=data.get("metadata", {})
                    )

                except (KeyError, ValueError, json.JSONDecodeError) as e:

                    self.set_fault(f"Invalid toolpath: {e}")
                    return

                self.queue.operations.append(operation)


        self.state.toolpath_loaded = True

        self.state.current_step = 0

        self.log_event(
            f"Loaded {len(self.queue.operations)} operations."
        )

    def execute_operation(self):

        operation = self.state.active_operation

        if operation is None:
            self.set_fault("No active operation.")
            return

        phase = self.state.operation_phase

        #start
        if phase == OperationPhase.START:

            self.log_event(
                f"Executing step {operation.step}"
            )

            self.set_operation_phase(OperationPhase.MOVE_XY)
            return

        # move xy
        if phase == OperationPhase.MOVE_XY:
            self.set_motion_state(MotionState.MOVING, "Motion command issued in xy")
            self.motion.move_axis("x", operation.x)
            self.motion.move_axis("y", operation.y)
            self.set_operation_phase(OperationPhase.WAIT_XY)
            return

        # wait xy
        if phase == OperationPhase.WAIT_XY:
            if (
                self.motion.axes["x"].is_idle()
                and
                self.motion.axes["y"].is_idle()
            ):
                self.set_operation_phase(OperationPhase.VERIFY_XY)

            return

      
        # verify xy
        if phase == OperationPhase.VERIFY_XY:
            #
            # TODO verify through:
            #
            # collision check
            #

            self.set_operation_phase(OperationPhase.MOVE_A)
            return

       #move a 
        if phase == OperationPhase.MOVE_A:
            self.set_motion_state(MotionState.MOVING, "Motion command issued in a")
            

            self.motion.move_axis(
                "a",
                operation.rotation
            )

            self.set_operation_phase(OperationPhase.WAIT_A)
            return

        #wait a
        if phase == OperationPhase.WAIT_A:

            if self.motion.axes["a"].is_idle():

                self.set_operation_phase(OperationPhase.VERIFY_A)

            return

        #verify a
        if phase == OperationPhase.VERIFY_A:

            if operation.operation == OperationType.REHEAT:

                self.set_forge_mode(ForgeMode.REHEATING)
                self.set_operation_phase(OperationPhase.COMPLETE)

            elif operation.operation == OperationType.INSPECT:

                self.set_forge_mode(ForgeMode.INSPECTING)
                self.set_operation_phase(OperationPhase.COMPLETE)

            elif operation.operation == OperationType.MOVE_ONLY:

                self.set_forge_mode(ForgeMode.POSITIONING)
                self.set_operation_phase(OperationPhase.COMPLETE)
                

            elif operation.operation == OperationType.DWELL:

                self.set_forge_mode(ForgeMode.WAITING)

                seconds = operation.metadata.get(
                    "seconds",
                    1.0
                )

                self.motion.interface.dwell(seconds)

                self.set_operation_phase(OperationPhase.COMPLETE)

            elif operation.operation == OperationType.CUSTOM:

                mdi = operation.metadata.get("mdi")

                if mdi:

                    self.motion.interface.execute_mdi(mdi)

                self.set_operation_phase(OperationPhase.COMPLETE)

            else:

                self.set_operation_phase(OperationPhase.MOVE_Z)

            return

        #move z 
        if phase == OperationPhase.MOVE_Z:
            self.set_motion_state(MotionState.MOVING, "Motion command issued in z")

            self.motion.move_axis(
                "z",
                operation.die_gap
            )

            self.set_forge_mode(ForgeMode.STRIKING)
            

            self.set_operation_phase(OperationPhase.WAIT_Z)

            return

        #wait z 
        if phase == OperationPhase.WAIT_Z:

            if self.motion.axes["z"].is_idle():

                self.set_operation_phase(OperationPhase.VERIFY_Z)

            return

        #verify z
        if phase == OperationPhase.VERIFY_Z:

            #
            # TODO: verify z placement through any or multiple of the following:
            #
            # load cell
            # stroke sensor
            # displacement sensor
            # force verification
            #

            self.set_operation_phase(OperationPhase.RETRACT_Z)

            return

        #retract 
        if phase == OperationPhase.RETRACT_Z:

            self.motion.move_axis("z", 0.0)

            self.set_operation_phase(OperationPhase.WAIT_RETRACT)

            return

        #wait retract 
        if phase == OperationPhase.WAIT_RETRACT:

            if self.motion.axes["z"].is_idle():

                self.set_operation_phase(OperationPhase.VERIFY_RETRACT)

            return

        #verify retract 
        if phase == OperationPhase.VERIFY_RETRACT:


            # verify fully clear of workpiece
            self.set_operation_phase(OperationPhase.COMPLETE)

            return

        # complete
        if phase == OperationPhase.COMPLETE:

            self.set_operation_phase(OperationPhase.START)

            self.set_motion_state(MotionState.COMPLETE, "Operation complete")

            self.set_brain_state(BrainState.VERIFY_OPERATION)

            return

        self.set_fault(
            f"Unknown operation phase: {phase}"
        )
    
    def load_operation(self):

        if self.queue.finished:

            self.set_brain_state(
                BrainState.COMPLETE_TOOLPATH
            )

            return


        operation = self.queue.current()

        if operation is None:
            self.set_fault(
                "Toolpath queue returned no operation"
            )
            return


        self.state.active_operation = operation

        self.state.current_step = operation.step

        self.set_brain_state(
            BrainState.PLAN_OPERATION
        )

        self.log_event(
            f"Loaded operation {operation.step}"
        )

    def plan_operation(self):

        #
        # TODO:
        #   verify temperature
        #   verify sensors
        #   collision checks
        #   die selection
        #

        self.set_forge_mode(ForgeMode.POSITIONING)
        self.set_brain_state(BrainState.EXECUTE_OPERATION)


    def initialize_machine(self):

        self.log_event("Initializing machine")

        interface = self.motion.interface
        interface.update()

        if interface.estop():
            self.set_fault(
                "Machine in ESTOP"
            )
            return


        if not interface.machine_on():
            interface.machine_on_command()
            time.sleep(1)

        
        if not interface.all_homed():

            if self.state.motion_state != MotionState.MOVING:

                self.log_event(
                    "Homing machine"
                )

                self.motion.home_all()

                self.set_motion_state(
                    MotionState.MOVING,
                    "Homing in progress"
                )

            return

        self.state.machine_enabled = True
        self.state.machine_homed = True
        self.set_motion_state(MotionState.READY, "Machine initialized and ready")

        self.set_brain_state(BrainState.IDLE)

    def idle(self):
        if self.queue.finished:
            return

        self.set_forge_mode(ForgeMode.WAITING)
        self.set_motion_state(MotionState.READY)

        self.set_brain_state(BrainState.LOAD_OPERATION)

        self.log_event(
            "Starting toolpath execution"
        )


    def complete_operation(self):

        operation = self.state.active_operation

        if operation is None:
            self.set_fault("No active operation")
            return

        self.queue.advance()

        self.state.completed_steps += 1

        self.command_history.append(operation)

        self.log_event(
            f"Completed operation {operation.step}"
        )

        self.state.active_operation = None

        self.set_brain_state(BrainState.ADAPT_OPERATION)


    def adapt_operation(self):

        """
        Adaptive manufacturing stage.

        This executes exactly once between every
        ToolpathOperation loaded from the JSONL file.

        Future responsibilities:

            • Thermal camera processing
            • 3D scan analysis
            • Die separation measurement
            • Load cell analysis
            • Billet temperature estimation
            • Adaptive reheating
            • Adaptive strike planning
            • Toolpath modification
            • AI decision making
        """

        #
        # Future:
        #
        # self.read_sensors()
        #
        # self.analyze_part()
        #
        # self.update_process_model()
        #
        # self.modify_remaining_toolpath()
        #
        # self.schedule_reheat()
        #

        self.set_brain_state(BrainState.NEXT_OPERATION)

    def verify_operation(self):

        operation = self.state.active_operation

        if operation is None:
            self.set_fault("No operation to verify")
            return

        #
        # TODO:
        #
        # Verify none of the motors skipped steps
        # Potentially check for any other machine faults 
        #

        interface = self.motion.interface
        interface.update()

        if interface.estop():
            self.set_fault("Machine entered ESTOP")
            return

        if not interface.machine_on():
            self.set_fault("Machine disabled")
            return

        self.log_event(
            f"Verified operation {operation.step}"
        )

        self.set_brain_state(BrainState.COMPLETE_OPERATION)

    def next_operation(self):
        if self.queue.finished:
            self.set_brain_state(BrainState.COMPLETE_TOOLPATH)
            return
        
        self.set_brain_state(BrainState.LOAD_OPERATION)

    def complete_toolpath(self):
        self.set_forge_mode(ForgeMode.COMPLETE)
        self.set_motion_state(MotionState.READY, "Toolpath complete")

        self.set_brain_state(BrainState.IDLE)
        

    def handle_fault(self):
        self.log_event(
            f"FAULT: {self.state.fault_message}"
        )

        self.motion.emergency_stop()
        self.running = False
