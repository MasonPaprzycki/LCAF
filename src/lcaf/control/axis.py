from __future__ import annotations

import time
import logging

from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional

from lcaf.control.linuxcnc_interface import LinuxCNCInterface

class AxisState(Enum):
    UNKNOWN = auto()
    READY = auto()
    MOVING = auto()
    HOMING = auto()
    COMPLETE = auto()
    FAULT = auto()
    ESTOP = auto()

# Axis Status
@dataclass
class AxisStatus:

    axis: str
    position: float = 0.0
    commanded_position: float = 0.0
    velocity: float = 0.0
    homed: bool = False
    enabled: bool = False
    moving: bool = False
    fault: bool = False
    in_position: bool = False
    state: AxisState = AxisState.UNKNOWN
    last_update: float = field(default_factory=time.time)

class Axis:

    def __init__(self, axis_name: str, joint: int, interface: LinuxCNCInterface):
        self.axis = axis_name
        self.joint = joint
        self.interface = interface

        self.status = AxisStatus(axis_name)
        self.logger = logging.getLogger(f"Motor-{axis_name}")

    def poll(self):

        self.interface.update()
        self.status.position = self.interface.get_position(self.joint)
        self.status.velocity = self.interface.get_velocity()
        self.status.enabled = self.interface.axis_enabled(self.joint)

        self.status.homed = self.interface.axis_homed(self.joint)
        self.status.in_position = self.interface.axis_in_position()
        self.status.last_update = time.time()

        # Homing detection
        if self.status.state == AxisState.HOMING:
            limits = self.interface.limits(self.joint)

            if limits["min"] or limits["max"]:
                self.status.homed = True
                self.status.state = AxisState.COMPLETE
                self.logger.info(f"{self.axis}: Home switch reached.")

        # Motion completion
        elif self.status.state == AxisState.MOVING:

            if self.status.in_position:
                self.status.state = AxisState.COMPLETE
                self.logger.info(f"{self.axis}: Motion complete.")

        elif self.status.state == AxisState.COMPLETE:
            self.status.state = AxisState.READY

    def home(self):
        self.status.state = AxisState.HOMING
        self.interface.home_axis(self.joint)
        self.logger.info( f"{self.axis}: Homing")

    def is_homed(self):
        return self.status.homed
    
    def move_to(self, position, feed=1000):
        self.status.commanded_position = position
        self.status.state = AxisState.MOVING
        self.interface.move_axis(self.axis, position, feed)

        self.logger.info(f"{self.axis}: Move -> {position}")

    def move_relative(self, distance: float):
        self.move_to(self.status.position + distance)

    def stop(self):
        self.logger.info(f"{self.axis}: Stop")

    def emergency_stop(self):
        self.status.state = AxisState.ESTOP
        self.logger.warning( f"{self.axis}: ESTOP")

    #status queries 
    def is_done(self):
        return self.status.state == AxisState.COMPLETE

    def is_idle(self):
        return self.status.state in ( AxisState.READY, AxisState.COMPLETE )

    def is_faulted(self):
        return self.status.fault

    def is_enabled(self):
        return self.status.enabled

    @property
    def position(self):
        return self.status.position


    @property
    def target(self):
        return self.status.commanded_position