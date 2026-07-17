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
from typing import List
from typing import Optional

from linuxcnc_interface import LinuxCNCInterface
from motion_coordinator import MotionCoordinator, MotionCoordinatorState

from toolpath import ToolpathOperation
from toolpath import OperationType


class BrainState(Enum):

    INITIALIZING = auto()
    IDLE = auto()

    EXECUTE_OPERATION = auto()

    VERIFY_OPERATION = auto()      # verify completed operation and handle book keeping

    ADAPT_OPERATION = auto()       # sensor analysis / adaptive logic

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

@dataclass
class SystemState:

    # Execution
    
    toolpath_loaded: bool = False
    brain_state: BrainState = BrainState.INITIALIZING
    forge_mode: ForgeMode = ForgeMode.STARTUP
    motion_state: MotionState = MotionState.UNKNOWN

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

        elif state == BrainState.EXECUTE_OPERATION:
            self.execute_operation()

        elif state == BrainState.VERIFY_OPERATION:
            self.verify_operation()

        elif state == BrainState.ADAPT_OPERATION:
            self.adapt_operation()

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

        self.set_brain_state(BrainState.EXECUTE_OPERATION)

        self.log_event(
            "Starting toolpath execution"
        )

    def execute_operation(self):

         # No work remaining?
        if self.queue.finished:
            self.set_brain_state(
                BrainState.COMPLETE_TOOLPATH
            )
            return
        
        # Retrieve next operation
        operation = self.queue.current()

        if operation is None:
            self.set_fault(
                "Toolpath queue returned no operation."
            )
            return
        
        # Start the Motion Coordinator exactly once.
        if self.motion.state == MotionCoordinatorState.IDLE:

            self.motion.start(operation)

        # Advance the Motion Coordinator HFSM.
        self.motion.update()

        # Motion fault?
        if self.motion.has_fault():

            self.set_fault(self.motion.fault_message)

            return

        # Finished?
        if self.motion.is_complete():

            self.motion.reset()

            self.set_brain_state(
                BrainState.VERIFY_OPERATION
            )

    def verify_operation(self):

        operation = self.queue.current()

        if operation is None:
            self.set_fault("No operation to verify")
            return

        #
        # TODO:
        #
        # Verify none of the motors skipped steps
        # Check for any other machine faults

        interface = self.motion.interface
        interface.update()

        if interface.estop():
            self.set_fault("Machine entered ESTOP")
            return

        if not interface.machine_on():
            self.set_fault("Machine disabled")
            return

        self.queue.advance()

        self.state.completed_steps += 1

        self.command_history.append(operation)

        self.log_event(
            f"Verified operation {operation.step}"
        )

        self.set_brain_state(BrainState.ADAPT_OPERATION)


    def adapt_operation(self):

        """
        Adaptive manufacturing stage.

        At the end of the function it modifies the tool path queue.
        The modifications are based on the analysis of the current operation and the state of the machine.
        If it needs to reheat it will insert a reheat operation into the tool path queue. 
        If it needs to modify the tool path it will simply change the next or any following tool paths in the queue.

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

        self.set_brain_state(BrainState.EXECUTE_OPERATION)

    
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