"""
===============================================================================
ForgeBrain
-------------------------------------------------------------------------------
Highest level abstraction for the forging brain. 

It is the the HSFM state machine outlined in docs/state_machine.md
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
from copy import deepcopy

from lcaf.control.linuxcnc_interface import LinuxCNCAxialInterface
from lcaf.control.motion_coordinator import MotionCoordinator, MotionCoordinatorState

from lcaf.utils.toolpath import ToolpathOperation
from lcaf.utils.toolpath import OperationType


class BrainState(Enum):

    INITIALIZING = auto()
    IDLE = auto()
    EXECUTE_OPERATION = auto()

    # verifies expected deformation behavior 
    # and updates adaptive logic based on sensor analysis
    ADAPT_OPERATION = auto()  
    COMPLETE_TOOLPATH = auto()

    FAULT = auto()
    SHUTDOWN = auto()

class HomingState(Enum):

    NOT_STARTED = auto()

    STARTING = auto()

    HOMING = auto()

    VERIFYING = auto()

    COMPLETE = auto()

    FAULT = auto()

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
    homing_state: HomingState = HomingState.NOT_STARTED
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

    def clone(self):

        new = ToolpathQueue()
        new.operations = deepcopy(self.operations)
        new.index = self.index

        return new

class ForgeBrain:

    """
    High-level autonomous forging controller.
    """

    def __init__(self, motion, planner):

        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s | %(levelname)s | %(name)s | %(message)s"
        )

        self.logger = logging.getLogger("ForgeBrain")

        self.state = SystemState()
        self.queue = ToolpathQueue()

        self.motion = motion
        self.planner = planner 
        self.running = False
        self.command_history = []

    def update(self, telemetry):

        # telemetry.forge_mode/motion_state are this same SystemState's own
        # values, broadcast for *other* subscribers (see
        # TelemetrySnapshot) -- not read back here, since self.state.motion_state
        # is already authoritative and managed by set_motion_state() below.
        self.state.machine_enabled = telemetry.machine_enabled
        self.state.machine_homed = telemetry.machine_homed
        self.state.estop = telemetry.estop
        self.state.billet_temperature = telemetry.billet_temperature
        self.state.last_update = telemetry.timestamp

        # Advancing self.motion's own HFSM happens once per heartbeat, from
        # process_state_machine() -> execute_operation() below -- not here.
        # This callback only absorbs the broadcast snapshot.

    def process_state_machine(self):
        state = self.state.brain_state

        if state == BrainState.INITIALIZING:
            self.initialize_machine()

        elif state == BrainState.IDLE:
            self.idle()

        elif state == BrainState.EXECUTE_OPERATION:
            self.execute_operation()

        elif state == BrainState.ADAPT_OPERATION:
            self.adapt_operation()

        elif state == BrainState.COMPLETE_TOOLPATH:
            self.complete_toolpath()

        elif state == BrainState.FAULT:
            self.handle_fault()

        elif state == BrainState.SHUTDOWN:
            self.shutdown()

    def log_event(self, message):
        self.state.last_event = message
        self.logger.info(message)

    def set_fault(self, message):
        self.state.fault_active = True
        self.state.fault_message = message
        self.set_brain_state(BrainState.FAULT, f"Fault message: {message}")

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
                        target_temperature=data.get("target_temperature",0.0),
                        metadata=data.get("metadata", {})
                    )

                except (KeyError, ValueError, json.JSONDecodeError) as e:

                    self.set_fault(f"Invalid toolpath: {e}")
                    return

                self.queue.operations.append(operation)


        self.state.toolpath_loaded = True

        self.state.current_step = 0

        self.log_event(f"Loaded {len(self.queue.operations)} operations.")


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
            interface.update()

            if not interface.machine_on():
                self.set_fault(
                    "Machine did not turn ON (LinuxCNC rejected STATE_ON -- still in ESTOP, "
                    "or another interlock is blocking it). Homing was not attempted: without "
                    "this check, jog commands issued while still OFF are silently ignored by "
                    "LinuxCNC, so the machine would sit motionless until the homing timeout."
                )
                return

        # motion.all_homed() (not interface.all_homed()) -- home_all() now
        # homes and retracts one axis at a time, and the last axis in that
        # sequence reports raw is_homed() True before its own retract has
        # even started. interface.all_homed() only reflects that raw
        # per-axis flag, so it would (briefly) read True one retract early;
        # motion.all_homed() tracks the whole sequential home+retract
        # sequence instead -- see MotionCoordinator.home_all().
        if not self.motion.all_homed():

            faulted_axis = self.motion.first_faulted_axis()
            if faulted_axis is not None:
                self.set_fault(f"Axis '{faulted_axis}' faulted while homing.")
                return

            if self.state.motion_state != MotionState.MOVING:
                self.motion.home_all()
                self.set_motion_state(MotionState.MOVING, "Homing in progress")

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

        self.set_brain_state(BrainState.EXECUTE_OPERATION,"Starting toolpath execution")

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
            self.set_fault("Toolpath queue returned no operation.")
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
            self.log_event(f"Executed operation {operation.step}")

            self.set_brain_state(BrainState.ADAPT_OPERATION)

    def adapt_operation(self):
        self.queue = self.planner.adapt(self.queue)

        self.set_brain_state(BrainState.EXECUTE_OPERATION)

    def complete_toolpath(self):
        self.set_forge_mode(ForgeMode.COMPLETE)
        self.set_motion_state(MotionState.READY, "Toolpath complete")

        self.set_brain_state(BrainState.IDLE)

    def handle_fault(self):
        self.log_event( f"FAULT: {self.state.fault_message}")

        self.motion.emergency_stop()
        self.running = False