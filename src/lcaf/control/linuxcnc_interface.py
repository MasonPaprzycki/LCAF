from __future__ import annotations

import linuxcnc
import hal

class LinuxCNCInterface:

    def __init__(self):

        self.command = linuxcnc.command()
        self.status = linuxcnc.stat()
        self.error = linuxcnc.error_channel()

        # Default limit switch HAL pins
        # Change these to match machine configuration.

        self.limit_pins = {

            0: {
                "min": "joint.0.neg-lim-sw-in",
                "max": "joint.0.pos-lim-sw-in"
            },

            1: {
                "min": "joint.1.neg-lim-sw-in",
                "max": "joint.1.pos-lim-sw-in"
            },

            2: {
                "min": "joint.2.neg-lim-sw-in",
                "max": "joint.2.pos-lim-sw-in"
            },

            3: {
                "min": "joint.3.neg-lim-sw-in",
                "max": "joint.3.pos-lim-sw-in"
            }

        }

    #Status update 
    def update(self):
        self.status.poll()

    # Machine Status
    def machine_on(self):
        return self.status.task_state == linuxcnc.STATE_ON

    def machine_enabled(self):
        return self.status.enabled

    def estop(self):
        return self.status.estop

    def interpreter_idle(self):
        return self.status.interp_state == linuxcnc.INTERP_IDLE

    def program_running(self):
        return self.status.interp_state == linuxcnc.INTERP_READING

    def mode(self):
        return self.status.task_mode

    def task_state(self):
        return self.status.task_state

    def all_homed(self):
        return all(self.status.homed)

    #axis status

    def get_position(self, joint):
        return self.status.position[joint]

    def get_velocity(self):
        return self.status.current_vel

    def axis_homed(self, joint):
        return self.status.homed[joint]

    def axis_enabled(self, joint):
        return self.status.enabled

    def axis_in_position(self):
        return self.status.inpos

    #homing
    def home_axis(self, joint):
        self.command.mode(linuxcnc.MODE_MANUAL)
        self.command.wait_complete()
        self.command.home(joint)
        #no wait_complete here

    def home_all(self):
        self.command.mode(linuxcnc.MODE_MANUAL)
        self.command.wait_complete()
        self.command.home(-1)

    def unhome_axis(self, joint):
        self.command.unhome(joint)

    #Machine Control 
    def abort(self):
        self.command.abort()

    def estop_reset(self):
        self.command.state(linuxcnc.STATE_ESTOP_RESET)

    def machine_on_command(self):
        self.command.state(linuxcnc.STATE_ON)

    def machine_off(self):
        self.command.state(linuxcnc.STATE_OFF)

    #MDI 
    def execute_mdi(self, mdi):

        self.command.mode(linuxcnc.MODE_MDI)
        self.command.wait_complete()
        self.command.mdi(mdi)
        #no wait_complete here

    def move_axis(self,
                  axis: str,
                  position: float,
                  feed: float = 1000):

        mdi = f"G1 {axis.upper()}{position:.4f} F{feed}"

        self.execute_mdi(mdi)

    def move_axes(self, x=None, y=None, z=None, a=None, feed=1000):

        mdi = "G1 "

        if x is not None:
            mdi += f"X{x:.4f} "

        if y is not None:
            mdi += f"Y{y:.4f} "

        if z is not None:
            mdi += f"Z{z:.4f} "

        if a is not None:
            mdi += f"A{a:.4f} "

        mdi += f"F{feed}"

        self.execute_mdi(mdi)

    def dwell(self, seconds):
        self.execute_mdi(f"G4 P{seconds}")

   #Hal read 
    def read_pin(self, pin_name):
        return hal.get_value(pin_name)

    def read_pins(self, pins):

        values = {}

        for pin in pins:
            values[pin] = hal.get_value(pin)

        return values

    #Hal write 
    def write_pin(self, pin_name, value):
        hal.set_p(pin_name, str(value))

    # Limit switches 
    def limit_min(self, joint):
        return self.read_pin(
            self.limit_pins[joint]["min"]
        )

    def limit_max(self, joint):
        return self.read_pin(
            self.limit_pins[joint]["max"]
        )

    def limits(self, joint):
        return {
            "min": self.limit_min(joint),
            "max": self.limit_max(joint)
        }
    
    #hal helpers 
    def read_axis_pin(self, joint, pin_suffix):
        pin = f"joint.{joint}.{pin_suffix}"

        return self.read_pin(pin)

    def get_errors(self):
        errors = []

        while True:
            error = self.error.poll()

            if error is None:
                break

            errors.append(error)

        return errors
    
    def homing_error(self):

        errors = self.get_errors()

        for error in errors:

            if error is None:
                continue

            text = str(error).lower()

            if "home" in text:
                return error

        return None