from __future__ import annotations
from lcaf.utils.joint_configuration import JointConfiguration

import linuxcnc
import hal
import time


class LinuxCNCMachineInterface:
    """
    Single shared connection to a running LinuxCNC instance.

    LinuxCNC exposes one command channel, one status channel, and one error
    channel for the whole machine (NML), not one per joint. This object owns
    that single connection; every LinuxCNCAxialInterface is handed a
    reference to it instead of opening its own.

    This is also the object MotionCoordinator publishes as `self.interface`
    for ForgeBrain (docs/state_machine.md) to query machine-wide state:
    update(), estop(), machine_on(), machine_on_command(), all_homed().
    """

    def __init__(self, use_native_homing: bool = False):
        self.command = linuxcnc.command()
        self.status = linuxcnc.stat()
        self.error = linuxcnc.error_channel()

        self.use_native_homing = use_native_homing
        """
        Mirrors MachineConfiguration.use_linuxcnc_native_processes (see
        lcaf.utils.joint_configuration) -- False (default) means every
        LinuxCNCAxialInterface homes in software by jogging to limit
        switches; True means it calls LinuxCNC's own native
        command.home(joint) instead. This is a machine-wide choice, so it
        lives on the one shared interface rather than per axis.
        """

        self._axes = []

    def register_axes(self, axes):
        """
        Give the machine interface the set of Axis objects it should
        consult for all_homed(). Even in native-homing mode this project
        still tracks "homed" through each Axis's own bookkeeping (which
        LinuxCNCAxialInterface.start_homing/poll_homing keeps in sync with
        status.joint[n]['homed'] either way) rather than reading LinuxCNC's
        status.homed directly here, so ForgeBrain always has one consistent
        source of truth regardless of use_native_homing.
        """

        self._axes = list(axes)

    # Status update
    def poll(self):
        self.status.poll()

    def update(self):
        self.poll()

    # Machine-wide queries
    def machine_on(self):
        return self.status.task_state == linuxcnc.STATE_ON

    def estop(self):
        return bool(self.status.estop)

    def all_homed(self):
        if not self._axes:
            return False

        return all(axis.is_homed() for axis in self._axes)

    # Machine-wide commands
    def machine_on_command(self):
        self.command.state(linuxcnc.STATE_ON)

    def machine_off(self):
        self.command.state(linuxcnc.STATE_OFF)

    def estop_command(self):
        self.command.state(linuxcnc.STATE_ESTOP)

    def estop_reset(self):
        self.command.state(linuxcnc.STATE_ESTOP_RESET)

    def abort(self):
        self.command.abort()

    def get_errors(self):
        errors = []

        while True:
            error = self.error.poll()

            if error is None:
                break

            errors.append(error)

        return errors


class LinuxCNCAxialInterface:

    def __init__(self, joint: JointConfiguration, machine: LinuxCNCMachineInterface):

        self.joint = joint
        self.machine = machine

        # 1 inch = 25.4 mm
        self.command = machine.command
        self.status = machine.status
        self.error = machine.error

        # "native" is whatever unit LinuxCNC itself reports for this joint --
        # inches for a linear joint (configs/machine.json TRAJ LINEAR_UNITS),
        # degrees for an angular joint (always degrees, regardless of
        # LINEAR_UNITS). is_position_in_range() and get_position() both work
        # in this unit. "mm" is a human-readable convenience derived from it
        # and is only meaningful for linear joints -- it is left unset (None)
        # for an angular joint rather than computed nonsensically.
        self.min_limit = {"native": 0.0, "mm": None if joint.is_angular else 0.0}
        self.max_limit = {"native": 0.0, "mm": None if joint.is_angular else 0.0}

        self.position_offset_to_native = 0.0

        self.axis_homed = False
        self.homing_ever_been_intialized = False

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
        #     self.min_limit["native"]
        #     self.max_limit["native"]
        #     self.min_limit["mm"] / self.max_limit["mm"] (linear joints only)
        #
        # using the measured travel of the axis.

        # Homing phase state -- see start_homing()/poll_homing(). None means
        # no homing in progress. Homing never blocks the caller: each
        # poll_homing() call advances at most one phase transition, so the
        # control loop's heartbeat keeps running (telemetry, other axes'
        # polling, ESTOP handling) while a jog is in flight, instead of
        # busy-waiting inside a single call until a limit switch trips.
        self._homing_phase: str | None = None
        self._homing_start_time: float = 0.0
        self._homing_speed: float = 0.0
        self._homing_timeout: float = 30.0

    # Status update
    def poll(self):
        self.status.poll()

    def program_running(self):
        return self.status.interp_state == linuxcnc.INTERP_READING

    # Machine-wide control (estop/on/off/abort) lives on LinuxCNCMachineInterface;
    # use self.machine for that rather than duplicating it per axis.

    def soft_stop(self):
        self.command.jog(linuxcnc.JOG_STOP, True, self.joint.joint)

    # axis status getter functions
    def get_linuxcnc_native_position(self):
        self.poll()
        # status.position/status.commanded_position are trajectory
        # (axis-letter) indexed, not joint-indexed -- joint_actual_position
        # is guaranteed to be the actual position of *this* joint number
        # regardless of axis-letter/joint-number ordering. See
        # https://linuxcnc.org/docs/html/config/python-interface.html.
        return self.status.joint_actual_position[self.joint.joint]

    def get_position(self):
        return self.get_linuxcnc_native_position() - self.position_offset_to_native

    def to_machine_units(self, native_value: float) -> float:
        """
        Convert a native-unit value (this joint's own LinuxCNC unit --
        inches for a linear joint, degrees for the angular one) to "machine
        units": millimetres for a linear joint, degrees for the angular
        one (unchanged). Machine units are the unit space
        MotionCoordinator/ForgeBrain actually command and compare
        positions in -- lcaf.toolpathing's ToolpathOperation.x/y/die_gap
        are millimetre-valued, and move() already sends them to LinuxCNC
        with an explicit G21 regardless of this machine's inch-native TRAJ
        units (see move()). Angular values need no conversion, since A/B/C
        words are always degrees, unaffected by G20/G21.
        """
        return native_value if self.joint.is_angular else native_value * 25.4

    def get_position_machine_units(self) -> float:
        """get_position(), converted to machine units (see
        to_machine_units) -- the unit MotionCoordinator actually compares
        commanded positions in."""
        return self.to_machine_units(self.get_position())

    def get_velocity(self):
        self.poll()
        return self.status.joint[self.joint.joint]["velocity"]

    def has_axis_been_homed(self):
        self.poll()
        return self.axis_homed

    def has_homing_ever_been_intialized(self):
        self.poll()
        return self.homing_ever_been_intialized

    def is_axis_enabled(self):
        self.poll()
        return bool(self.status.joint[self.joint.joint]["enabled"])

    def is_axis_in_position(self):
        self.poll()
        return self.status.joint[self.joint.joint]["inpos"]

    def is_faulted(self):
        """
        True if LinuxCNC has faulted this joint's amp -- for these open-loop
        stepper joints (see docs/potential_issues.md), that fault is almost
        always a tripped FERROR/MIN_FERROR following error, LinuxCNC's own
        commanded-vs-actual comparison against the tolerances configured in
        machine.json. This is the only stall/skipped-step detection this
        machine has.
        """
        self.poll()
        return bool(self.status.joint[self.joint.joint]["fault"])

    def is_position_in_range(self, position_machine_units: float | None = None) -> bool:
        """
        True if position_machine_units (millimetres for a linear joint,
        degrees for the angular one -- see to_machine_units) is within this
        joint's homed range. Defaults to checking the axis's own current
        position when no target is given (used by callers that just want a
        self-check, e.g. a health poll rather than validating a specific
        commanded move -- Axis.move() always passes the real target).
        """
        if position_machine_units is None:
            position_machine_units = self.get_position_machine_units()

        minimum = self.to_machine_units(self.min_limit["native"])
        maximum = self.to_machine_units(self.max_limit["native"])
        return minimum - 1e-9 <= position_machine_units <= maximum + 1e-9

    def get_targeted_position(self):
        self.poll()
        # joint_position is LinuxCNC's desired/commanded joint position,
        # joint-indexed -- linuxcnc.stat() has no "commanded_position"
        # attribute at all (see get_linuxcnc_native_position above).
        return self.status.joint_position[self.joint.joint]

    # homing
    def start_homing(self, speed: float = 0.03937, timeout: float = 30.0):
        """
        Begin homing without blocking. Dispatches to native or software
        homing depending on self.machine.use_native_homing (mirrors
        MachineConfiguration.use_linuxcnc_native_processes -- a machine-wide
        choice, not a per-joint one). Call poll_homing() every subsequent
        heartbeat until it returns True (or raises) to actually advance and
        complete homing -- this call only issues the first command (or, for
        a switchless joint, finishes immediately).

        speed is only used by software homing: a jog velocity in this
        joint's native units/second -- inches/s for a linear joint, deg/s
        for an angular joint (jogs bypass the G-code interpreter entirely,
        so there is no G20/G21 to declare it in, unlike move()). The
        0.03937 in/s default equals the previous 1.0 mm/s default homing
        speed (1.0 / 25.4), kept deliberately slow/conservative. Native
        homing's speed instead comes from the generated INI's
        HOME_SEARCH_VEL/HOME_LATCH_VEL (see generate_ini()).
        """
        self.homing_ever_been_intialized = True
        self._homing_speed = speed
        self._homing_timeout = timeout
        self.poll()

        if self.machine.use_native_homing:
            self.command.mode(linuxcnc.MODE_MANUAL)
            self.command.wait_complete()
            self.command.home(self.joint.joint)
            self._homing_phase = "native_wait"
            self._homing_start_time = time.monotonic()
        else:
            self._start_software_homing()

    def poll_homing(self) -> bool:
        """
        Advance homing by at most one step. Call every heartbeat while
        homing is in progress (Axis.poll() drives this) -- never blocks
        waiting on a limit switch or LinuxCNC status, unlike the old
        blocking home_axis() this replaced. Returns True once homing has
        completed successfully (axis_homed is now True); returns False if
        homing is still in progress. Raises RuntimeError on a reported
        LinuxCNC joint fault, or TimeoutError if the configured timeout
        elapses before the expected condition (native-homed status, or a
        limit switch) is observed -- Axis.poll() catches both and moves the
        axis to AxisState.FAULT rather than letting them crash the control
        process.
        """
        if self._homing_phase is None:
            return self.axis_homed

        self.poll()

        if self._homing_phase == "native_wait":
            joint_status = self.status.joint[self.joint.joint]

            if joint_status["fault"]:
                self._homing_phase = None
                raise RuntimeError(
                    f"Joint {self.joint.joint} ({self.joint.axis}) faulted while homing."
                )

            if joint_status["homed"]:
                # HOME_OFFSET is always 0.0 (see generate_ini()), so native
                # position 0 is already where this project's own coordinate
                # zero belongs -- no offset needed.
                self.position_offset_to_native = 0.0
                self._set_homed_limits()
                self.axis_homed = True
                self._homing_phase = None
                return True

            if time.monotonic() - self._homing_start_time > self._homing_timeout:
                self._homing_phase = None
                raise TimeoutError(
                    f"Timed out waiting for joint {self.joint.joint} "
                    f"({self.joint.axis}) to report native-homed."
                )

            return False

        if self._homing_phase == "software_seek_min":
            if self.min_limit_active():
                self.soft_stop()
                self.position_offset_to_native = self.get_linuxcnc_native_position()
                self.jog_positive(self._homing_speed)
                self._homing_phase = "software_seek_max"
                self._homing_start_time = time.monotonic()
                return False

            if time.monotonic() - self._homing_start_time > self._homing_timeout:
                self.soft_stop()
                self._homing_phase = None
                raise TimeoutError("Timed out waiting for the minimum limit switch.")

            return False

        if self._homing_phase == "software_seek_max":
            if self.max_limit_active():
                self.soft_stop()
                self._set_homed_limits(measured_max_native=self.get_position())
                self.axis_homed = True
                self._homing_phase = None
                return True

            if time.monotonic() - self._homing_start_time > self._homing_timeout:
                self.soft_stop()
                self._homing_phase = None
                raise TimeoutError("Timed out waiting for the maximum limit switch.")

            return False

        return False

    def is_homing_in_progress(self) -> bool:
        return self._homing_phase is not None

    def _start_software_homing(self):
        """
        This project's own software homing, phase setup: jog toward a limit
        switch, or zero immediately for a switchless joint. Used when
        MachineConfiguration.use_linuxcnc_native_processes is False (the
        default) -- see generate_ini()'s NO_FORCE_HOMING, which is what
        keeps LinuxCNC from rejecting the moves this performs. Each
        subsequent phase step is advanced by poll_homing(), never here.
        """
        if not self.joint.has_limit_switches:
            # Continuous/switchless joint (A): there is nothing to seek.
            # Zero at the current position and trust the configured static
            # max_travel/min_travel (JointConfiguration guarantees
            # max_travel is set when has_limit_switches is False).
            self.position_offset_to_native = self.get_linuxcnc_native_position()
            self._set_homed_limits()
            self.axis_homed = True
            self._homing_phase = None
            return

        self.poll()
        self._homing_start_time = time.monotonic()

        if self.min_limit_active():
            self.position_offset_to_native = self.get_linuxcnc_native_position()
            self.jog_positive(self._homing_speed)
            self._homing_phase = "software_seek_max"
        else:
            self.jog_negative(self._homing_speed)
            self._homing_phase = "software_seek_min"

    def _set_homed_limits(self, measured_max_native: float | None = None):
        """
        Set min_limit/max_limit once homing has established where native
        position 0 is. measured_max_native, if given, overrides the
        configured max_travel with an actual measured travel distance
        (software homing on a switched joint only -- see
        _start_software_homing/poll_homing). Every other case trusts the
        configured max_travel/min_travel (JointConfiguration.min_travel is
        0.0 for a switched joint, -max_travel for a switchless angular one
        -- see its docstring).
        """
        assert self.joint.max_travel is not None

        self.min_limit["native"] = self.joint.min_travel
        self.max_limit["native"] = (
            measured_max_native if measured_max_native is not None else self.joint.max_travel
        )
        if not self.joint.is_angular:
            self.min_limit["mm"] = self.min_limit["native"] * 25.4
            self.max_limit["mm"] = self.max_limit["native"] * 25.4

    # MDI
    def execute_mdi(self, mdi):
        self.command.mode(linuxcnc.MODE_MDI)
        self.command.wait_complete()
        self.command.mdi(mdi)
        # no wait_complete here

    def move(self, position: float, feed: float = 1000):
        # G21 pins this MDI to millimetres regardless of the machine's native
        # TRAJ units (inches -- see configs/machine.json / axis.jsonl). Every
        # position/feed value reaching this method originates from
        # lcaf.toolpathing (ToolpathOperation.x/y/die_gap/rotation), which is
        # generated in millimetres; without an explicit units word here the
        # interpreter falls back to whatever G20/G21 mode is saved in the
        # RS274NGC parameter file from the last run, which is not guaranteed
        # to be mm. A/B/C words (rotation) are always degrees and unaffected
        # by G20/G21, so this is safe for the angular joint too.
        mdi = f"G21 G1 {self.joint.axis}{position:.4f} F{feed}"

        self.execute_mdi(mdi)

    def dwell(self, seconds):
        self.execute_mdi(f"G4 P{seconds}")

    def min_limit_active(self):
        return hal.get_value(self.joint.negative_limit_hal_pin)

    def max_limit_active(self):
        return hal.get_value(self.joint.positive_limit_hal_pin)

    def jog_positive(self, speed):
        self.command.mode(linuxcnc.MODE_MANUAL)
        self.command.wait_complete()
        self.command.jog(linuxcnc.JOG_CONTINUOUS, True, self.joint.joint, speed)

    def jog_negative(self, speed):
        self.command.mode(linuxcnc.MODE_MANUAL)
        self.command.wait_complete()
        self.command.jog(linuxcnc.JOG_CONTINUOUS, True, self.joint.joint, -speed)

    def is_idle(self, velocity_tolerance: float = 1e-6):
        self.poll()

        joint = self.status.joint[self.joint.joint]

        return (
            joint["inpos"] and
            abs(joint["velocity"]) <= velocity_tolerance
        )
