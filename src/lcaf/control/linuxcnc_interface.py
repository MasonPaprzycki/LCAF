from __future__ import annotations

import linuxcnc
import hal
import time

class LinuxCNCAxialInterface:

    def __init__(self, hal_joint: int):
        # 1 inch = 25.4 mm
        self.command = linuxcnc.command()
        self.status = linuxcnc.stat()
        self.error = linuxcnc.error_channel()
        self.min_limit = {"inches": 0.0, "mm": 0.0}
        self.max_limit = {"inches": 0.0, "mm": 0.0}
        self.hal_joint = hal_joint
        self.position_offset_to_native = 0.0 

        self.axis_homed = False
        self.homing_ever_been_intialized = False


        #limit switch HAL pins

        # defines the limit switch for the minimum axis position 
        self.limit_switch_hal_pins = {
            "min": f"joint.{hal_joint}.neg-lim-sw-in",
            "max": f"joint.{hal_joint}.pos-lim-sw-in"
        }


        # Homing command needs to check whether the min axis position
        # limit switch is active. If it is active it should not move
        # in the negative direction it would then move in the positive direction
        # until the maximum limit switch is activated.
        # It then sets the min position as zero
        # and the max position as however far the distance between them is.
        # It should pull the hal configuration from linux cnc to know
        # Basically the gear ratio and how many turns or step signals converts into what distance
        # But it should not assume the set distance or length of the axis provided by the user is correct
        # it finds that distance with the limit switches.
        #
        # During homing this interface dynamically updates:
        #
        #     self.min_limit
        #     self.max_limit["mm"]
        #     self.max_limit["inches"]
        #
        # using the measured travel of the axis.

        
    #Status update 
    def poll(self):
        self.status.poll()

    # Machine Status
    def machine_on(self):
        return self.status.task_state == linuxcnc.STATE_ON

    def program_running(self):
        return self.status.interp_state == linuxcnc.INTERP_READING
    
    #Machine Control 
    def abort(self):
        self.command.abort()

    def estop_reset(self):
        self.command.state(linuxcnc.STATE_ESTOP_RESET)

    def estop(self):
        self.command.state(linuxcnc.STATE_ESTOP)
        
    def machine_on_command(self):
        self.command.state(linuxcnc.STATE_ON)

    def machine_off(self):
        self.command.state(linuxcnc.STATE_OFF)

    def soft_stop(self):
        self.command.jog(linuxcnc.JOG_STOP, True, self.hal_joint)

    #axis status getter functions
    def get_linuxcnc_native_position(self):
        self.poll()
        return self.status.position[self.hal_joint]
    
    def get_position(self):
        self.poll()
        return self.get_linuxcnc_native_position() - self.position_offset_to_native

    def get_velocity(self):
        self.poll()
        return self.status.joint[self.hal_joint]["velocity"]

    def has_axis_been_homed(self):
        self.poll()
        return self.axis_homed
    
    def has_homing_ever_been_intialized(self):
        self.poll()
        return self.homing_ever_been_intialized

    def is_axis_enabled(self):
        self.poll()
        return self.status.enabled

    def is_axis_in_position(self):
        self.poll()
        return self.status.joint[self.hal_joint]["inpos"]
    
    def is_position_in_range(self):
        self.poll()
        return (self.min_limit["mm"] <= self.get_position() <= self.max_limit["mm"])
    
    def get_targeted_position(self):
        self.poll()
        return self.status.commanded_position[self.hal_joint]
    
    #homing
    def home_axis(self, speed: float = 1.0, timeout: float = 30.0):
        self.homing_ever_been_intialized = True
        self.poll()

        if self.min_limit_active():
            self.position_offset_to_native = self.get_linuxcnc_native_position()

            try:

                self.jog_positive(speed)

                start_time = time.monotonic()

                while not self.max_limit_active():

                    self.poll()

                    if time.monotonic() - start_time > timeout:
                        raise TimeoutError(
                            "Timed out waiting for the maximum limit switch."
                        )
            finally:
                self.soft_stop()

            self.max_limit["mm"] = self.get_position()
            self.max_limit["inches"] = self.max_limit["mm"] / 25.4
            self.min_limit["mm"] = 0.0
            self.min_limit["inches"] = 0.0

        else:

            try:
                self.jog_negative(speed)

                start_time = time.monotonic()

                while not self.min_limit_active():
                    self.poll()

                    if time.monotonic() - start_time > timeout:
                        raise TimeoutError(
                            "Timed out waiting for the minimum limit switch."
                        )
            finally:
                self.soft_stop()

            return self.home_axis(speed=speed, timeout=timeout)

        self.axis_homed = True
        

    #MDI 
    def execute_mdi(self, mdi):

        self.command.mode(linuxcnc.MODE_MDI)
        self.command.wait_complete()
        self.command.mdi(mdi)
        #no wait_complete here

    def move_axis(self, axis: str, position: float, feed: float = 1000):

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

    #Hal write 
    def min_limit_active(self):
        return self.read_pin(
            pin_suffix=None, pin_name=self.limit_switch_hal_pins["min"]
        )

    def max_limit_active(self):
        return self.read_pin(
            pin_suffix=None,
            pin_name=self.limit_switch_hal_pins["max"]
        )
    
    def write_pin(
            self,  
            pin_suffix: str | None,
            pin_name : str|None,
            value
        ):
        
        if pin_name is None: 
            if  (pin_suffix is not None):
                pin_name = f"joint.{self.hal_joint}.{pin_suffix}"
            else: 
                raise KeyError("Hal pin has not been designated")
        
        hal.set_p(pin_name, str(value))

    def jog_positive(self, speed):
        self.command.mode(linuxcnc.MODE_MANUAL)
        self.command.wait_complete()
        self.command.jog(linuxcnc.JOG_CONTINUOUS, True, self.hal_joint, speed)
    
    def jog_negative(self, speed):
        self.command.mode(linuxcnc.MODE_MANUAL)
        self.command.wait_complete()
        self.command.jog(linuxcnc.JOG_CONTINUOUS, True, self.hal_joint, -speed)
    
    
    #hal helpers 
    def read_pin(
            self, 
            pin_suffix: str | None,
            pin_name : str|None
        ):

        if pin_name is None: 
            if (pin_suffix is not None):
                pin_name = f"joint.{self.hal_joint}.{pin_suffix}"
            else: 
                raise KeyError("Hal pin has not been designated")
            
        return hal.get_value(pin_name)
    

    def get_errors(self):
        errors = []

        while True:
            error = self.error.poll()

            if error is None:
                break

            errors.append(error)

        return errors
    
    def is_idle(self, velocity_tolerance: float = 1e-6):
        self.poll()

        joint = self.status.joint[self.hal_joint]

        return (
            joint["inpos"] and
            abs(joint["velocity"]) <= velocity_tolerance
        )