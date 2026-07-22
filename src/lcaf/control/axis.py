from __future__ import annotations

import time
import logging

from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional

from lcaf.control.linuxcnc_interface import LinuxCNCAxialInterface

class AxisState(Enum):
    UNINITIALIZED = auto()
    READY = auto()
    MOVING = auto()
    HOMING = auto()
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

    def __init__(self, axis: str, hal_joint: int):
        self.axis = axis
        self.joint = hal_joint
        self.axial_interface = LinuxCNCAxialInterface(hal_joint)

        self.status = AxisStatus(axis)
        self.logger = logging.getLogger(f"Motor-{axis}")

    def poll(self):

        ##if self.axial_interface.is_faulted():
            ##self.status.state = AxisState.FAULT
        
        self.axial_interface.poll()

        self.status.last_update = time.time()

        if self.status.state == AxisState.FAULT:
            self.logger.error(f"{self.axis}: Axis fault detected.")
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
            
            else:
                if not self.axial_interface.has_homing_ever_been_intialized():
                    self.axial_interface.home_axis()

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
    
    def move(self, position, feed=1):
        if self.is_homed():
            if self.axial_interface.is_position_in_range():
                self.status.state = AxisState.MOVING
                self.axial_interface.move_axis(self.axis, position, feed)
      
            else: 
                self.logger.error(f"{self.axis}: Position out of homed range check forge configuration in toolpath generator")
        else:
            self.logger.error(f"{self.axis}: Axis has not been homed. Home the axis first with the home() method.")

    def soft_stop(self):
        self.axial_interface.soft_stop()
        self.status.state = AxisState.READY
        self.logger.info(f"{self.axis}: Stop")

    def estop(self):
        self.status.state = AxisState.ESTOP
        self.axial_interface.estop()
        self.logger.warning( f"{self.axis}: ESTOP")

    def is_estop_active(self):
        return self.status.state == AxisState.ESTOP

    def is_moving(self):
        return self.status.state == AxisState.MOVING
    
    def position(self):
        return self.axial_interface.get_position()
    
    def is_retracted(self):
        return self.axial_interface.get_position() == 0
    
    def is_idle(self):
        return self.axial_interface.is_idle()
    
    def is_enabled(self):
        return self.axial_interface.is_axis_enabled()
