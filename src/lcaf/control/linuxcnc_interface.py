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

        # hal.get_value() (used by LinuxCNCAxialInterface.min_limit_active/
        # max_limit_active) attaches to LinuxCNC's shared-memory HAL segment
        # lazily, on the first component created in this process -- without
        # ever creating one, it raises "Cannot call before creating
        # component". This component owns no pins of its own; it exists
        # purely so hal.get_value() has something to attach through.
        self.hal_component = hal.component("lcaf_control")
        self.hal_component.ready()

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
        # using the measured travel of the axis -- but only when
        # joint.dual_limit_switches is True (this joint has a switch at
        # both ends). When it's False (one limit switch: only the negative
        # end is wired), homing still seeks that one switch to establish
        # zero, but then trusts joint.extended_distance as configured
        # instead of measuring it, since there's no second switch to seek.
        # See JointConfiguration.dual_limit_switches and
        # _start_software_homing/poll_homing below.

        # Homing phase state -- see start_homing()/poll_homing(). None means
        # no homing in progress. Homing never blocks the caller: each
        # poll_homing() call advances at most one phase transition, so the
        # control loop's heartbeat keeps running (telemetry, other axes'
        # polling, ESTOP handling) while a jog is in flight, instead of
        # busy-waiting inside a single call until a limit switch trips.
        self._homing_phase: str | None = None
        self._homing_start_time: float = 0.0
        self._homing_speed: float = 0.0
        self._homing_timeout: float = 60.0

        # Retract-to-zero phase state -- see start_retract_to_zero()/
        # poll_retract_to_zero() below. Separate from _homing_phase (and
        # never touches axis_homed/min_limit/max_limit the way initial
        # homing does) since MotionCoordinator runs this repeatedly, once
        # per retract, for the lifetime of the session -- not just once at
        # startup.
        self._retract_phase: str | None = None
        self._retract_start_time: float = 0.0

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

    def is_on_hard_limit(self):
        """
        True if LinuxCNC currently reports this joint on its negative or
        positive hard limit (status.joint[n]['min_hard_limit']/
        ['max_hard_limit'] -- LinuxCNC's own live read of the same
        neg-lim-sw-in/pos-lim-sw-in pin min_limit_active()/max_limit_active()
        poll, not a latched fault flag).

        This is a *different* signal from is_faulted(): that only reflects
        status.joint[n]['fault'], which LinuxCNC's Python interface
        documents as "axis amp fault" (FERROR/MIN_FERROR here -- see
        is_faulted()'s docstring) and does not change when a hard limit
        trips. A hard-limit trip instead disables this joint's (and every
        other joint's) amp-enable-out machine-wide without ever touching
        'fault' -- see docs/hardware_setup.md "What a tripped limit switch
        actually does". Without checking this separately, that condition is
        invisible to this project: nothing else here would report it as a
        fault, and the axis would just sit disabled indefinitely.

        Expected to read True for exactly the joint/direction being
        deliberately sought during software homing (_jog_toward_limit_switch
        overrides the fault itself, but this status field still reflects the
        live switch state) -- callers should only treat this as a fault
        outside of homing. See Axis.poll().
        """
        self.poll()
        joint = self.status.joint[self.joint.joint]
        return bool(joint["min_hard_limit"]) or bool(joint["max_hard_limit"])

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
    def start_homing(self, speed: float = 0.03937, timeout: float = 60.0):
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
            # Re-armed every heartbeat, not just once in _jog_toward_limit_switch():
            # LinuxCNC clears an override as soon as the joint next reports
            # "in position" (see override_limits() docstring), and this
            # project has no way to confirm from here exactly when the jog
            # actually leaves that state. Calling it again here is a no-op
            # if it's already armed, and guarantees it's still armed at the
            # instant min_limit_active() actually goes true below.
            self.command.override_limits()

            if self.min_limit_active():
                self.soft_stop()
                self.position_offset_to_native = self.get_linuxcnc_native_position()

                if self.joint.dual_limit_switches:
                    self._jog_toward_limit_switch(positive=True)
                    self._homing_phase = "software_seek_max"
                    self._homing_start_time = time.monotonic()
                    return False

                # Single-limit-switch joint: zero is established, and there
                # is no second switch to seek -- trust the configured
                # extended_distance (see JointConfiguration.dual_limit_switches).
                self._set_homed_limits()
                self.axis_homed = True
                self._homing_phase = None
                return True

            if time.monotonic() - self._homing_start_time > self._homing_timeout:
                self.soft_stop()
                self._homing_phase = None
                raise TimeoutError("Timed out waiting for the minimum limit switch.")

            return False

        if self._homing_phase == "software_seek_max":
            # See the matching comment in the software_seek_min branch above.
            self.command.override_limits()

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

    # Retract-to-zero -- re-seek the negative limit switch before every
    # retract move (see MotionCoordinator/Axis.retract_to_zero()), not just
    # once at process start. These joints are open-loop steppers with no
    # position feedback (docs/potential_issues.md): the commanded "0.0"
    # position can silently drift from true mechanical zero as steps are
    # missed over a session, so re-referencing to the physical switch on
    # every retract -- instead of trusting the accumulated commanded
    # position -- is the only way this project can catch that drift.
    #
    # joint.retract_to (this project's Y, set to 1.5) is the one exception:
    # its "retract" is a plain move to that configured position instead,
    # with no switch to re-reference against -- see start_retract_to_zero().
    #
    # Deliberately separate from _homing_phase/start_homing/poll_homing:
    # this never touches axis_homed, min_limit, or max_limit (a
    # dual_limit_switches joint's measured travel from initial homing must
    # survive every later retract unchanged), and it requires initial
    # homing to have already completed once.
    def is_retract_in_progress(self) -> bool:
        return self._retract_phase is not None

    def start_retract_to_zero(self, speed: float = 0.03937, timeout: float = 60.0):
        """
        Begin retracting this joint -- for almost every joint that means
        re-seeking the negative limit switch to re-establish zero, without
        touching the previously-established max_limit (unlike
        start_homing(), which -- for a dual_limit_switches joint -- also
        remeasures travel by seeking the positive switch). Call
        poll_retract_to_zero() every subsequent heartbeat until it returns
        True (or raises) to actually advance and complete it -- mirrors
        start_homing()/poll_homing()'s non-blocking phase pattern.

        joint.retract_to changes what "retract" means for this particular
        joint -- see its docstring (this project's Y axis is mechanically
        arranged so its retracted position is partway into its
        positive-direction travel, not zero). In that case this instead
        commands a plain move to joint.retract_to in the coordinate frame
        already established by the last full home_all() -- there is no
        limit switch at that position to re-seek against, so this path
        never re-references position_offset_to_native the way the
        negative-switch retract does.

        Only valid for a switched joint (has_limit_switches) that has
        already completed its initial start_homing()/poll_homing() --
        raises RuntimeError otherwise (a fresh, never-homed joint has no
        min_limit/max_limit established yet and should go through
        start_homing() instead, which does everything this does plus the
        initial travel measurement/establishment).
        """
        if not self.joint.has_limit_switches:
            raise RuntimeError(
                f"Joint {self.joint.joint} ({self.joint.axis}) has no limit switches -- "
                "retract-to-zero is only meaningful for a switched joint."
            )

        if not self.axis_homed:
            raise RuntimeError(
                f"Joint {self.joint.joint} ({self.joint.axis}) must complete initial homing "
                "(start_homing/poll_homing) before it can retract-to-zero."
            )

        self._homing_speed = speed
        self._homing_timeout = timeout
        self.poll()
        self._retract_start_time = time.monotonic()

        if self.joint.retract_to is not None:
            target_machine_units = self.to_machine_units(self.joint.retract_to)
            feed_per_minute = self.to_machine_units(speed) * 60
            self.move(target_machine_units, feed=feed_per_minute)
            self._retract_phase = "move_to_retract_position"
        elif self.machine.use_native_homing:
            # LinuxCNC's own native homing always re-seeks the negative
            # switch and never remeasures travel (MAX_LIMIT is a static INI
            # value regardless of dual_limit_switches -- see
            # generate_ini()), so rerunning it here already is exactly a
            # retract-to-zero, with no separate code path needed.
            self.command.mode(linuxcnc.MODE_MANUAL)
            self.command.wait_complete()
            self.command.home(self.joint.joint)
            self._retract_phase = "native_wait"
        else:
            self._jog_toward_limit_switch(positive=False)
            self._retract_phase = "software_seek_min"

    def poll_retract_to_zero(self) -> bool:
        """
        Advance retract-to-zero by at most one step. Returns True once
        re-zeroing has completed; False while still in progress. Raises
        RuntimeError on a reported LinuxCNC joint fault, or TimeoutError if
        the configured timeout elapses first -- Axis.poll() catches both
        and moves the axis to AxisState.FAULT. See start_retract_to_zero().
        """
        if self._retract_phase is None:
            return True

        self.poll()

        if self._retract_phase == "move_to_retract_position":
            joint_status = self.status.joint[self.joint.joint]

            if joint_status["fault"]:
                self._retract_phase = None
                raise RuntimeError(
                    f"Joint {self.joint.joint} ({self.joint.axis}) faulted while retracting "
                    "to its configured retract_to position."
                )

            if self.is_axis_in_position():
                self._retract_phase = None
                return True

            if time.monotonic() - self._retract_start_time > self._homing_timeout:
                self._retract_phase = None
                raise TimeoutError(
                    f"Timed out waiting for joint {self.joint.joint} ({self.joint.axis}) "
                    "to reach its configured retract_to position during retract."
                )

            return False

        if self._retract_phase == "native_wait":
            joint_status = self.status.joint[self.joint.joint]

            if joint_status["fault"]:
                self._retract_phase = None
                raise RuntimeError(
                    f"Joint {self.joint.joint} ({self.joint.axis}) faulted while retracting to zero."
                )

            if joint_status["homed"]:
                # HOME_OFFSET is always 0.0 (see generate_ini()), so native
                # position 0 is already this project's own coordinate zero.
                self.position_offset_to_native = 0.0
                self._retract_phase = None
                return True

            if time.monotonic() - self._retract_start_time > self._homing_timeout:
                self._retract_phase = None
                raise TimeoutError(
                    f"Timed out waiting for joint {self.joint.joint} ({self.joint.axis}) "
                    "to re-home during retract-to-zero."
                )

            return False

        if self._retract_phase == "software_seek_min":
            # Re-armed every heartbeat -- see the matching comment in
            # poll_homing()'s software_seek_min branch.
            self.command.override_limits()

            if self.min_limit_active():
                self.soft_stop()
                self.position_offset_to_native = self.get_linuxcnc_native_position()
                self._retract_phase = None
                return True

            if time.monotonic() - self._retract_start_time > self._homing_timeout:
                self.soft_stop()
                self._retract_phase = None
                raise TimeoutError(
                    "Timed out waiting for the minimum limit switch during retract-to-zero."
                )

            return False

        return False

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
            # retracted_distance/extended_distance (JointConfiguration
            # requires both to be set for every joint).
            self.position_offset_to_native = self.get_linuxcnc_native_position()
            self._set_homed_limits()
            self.axis_homed = True
            self._homing_phase = None
            return

        self.poll()
        self._homing_start_time = time.monotonic()

        if self.min_limit_active():
            self.position_offset_to_native = self.get_linuxcnc_native_position()

            if self.joint.dual_limit_switches:
                self._jog_toward_limit_switch(positive=True)
                self._homing_phase = "software_seek_max"
            else:
                # Single-limit-switch joint (dual_limit_switches=False):
                # already sitting on the only switch it has, so zero is
                # already established -- trust the configured
                # extended_distance instead of seeking a second switch that
                # doesn't exist.
                self._set_homed_limits()
                self.axis_homed = True
                self._homing_phase = None
        else:
            self._jog_toward_limit_switch(positive=False)
            self._homing_phase = "software_seek_min"

    def _set_homed_limits(self, measured_max_native: float | None = None):
        """
        Set min_limit/max_limit once homing has established where native
        position 0 is. measured_max_native, if given, overrides the
        configured extended_distance with an actual measured travel
        distance (software homing on a switched joint only -- see
        _start_software_homing/poll_homing). Every other case trusts the
        configured retracted_distance/extended_distance
        (JointConfiguration.min_travel is -retracted_distance -- see its
        docstring).

        retracted_distance/extended_distance may be None -- a joint with a
        genuinely disabled soft limit on that end (this project's switchless
        A joint disables both -- see JointConfiguration.retracted_distance/
        extended_distance). Mirrors generate_ini()'s own handling of the
        same None: LinuxCNC's documented substitute for an omitted
        MIN_LIMIT/MAX_LIMIT is -1e99/1e99, so this uses the same sentinels
        rather than asserting a value that was deliberately left unset.
        """
        self.min_limit["native"] = (
            self.joint.min_travel if self.joint.retracted_distance is not None else -1e99
        )
        if measured_max_native is not None:
            self.max_limit["native"] = measured_max_native
        elif self.joint.extended_distance is not None:
            self.max_limit["native"] = self.joint.extended_distance
        else:
            self.max_limit["native"] = 1e99
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
        # TRAJ units (inches -- see configs/machine.json / axis.json). Every
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

    def _jog_toward_limit_switch(self, positive: bool):
        """
        Jog deliberately toward a limit switch during software homing
        (_start_software_homing/poll_homing) -- the only place this project
        intentionally drives a joint into one.

        The same switch is also wired straight into LinuxCNC's own
        joint.N.neg-lim-sw-in/pos-lim-sw-in pins (see generate_hal()), which
        LinuxCNC's motion controller checks every servo cycle independent of
        this Python process. Normally, tripping one of those pins reports
        "joint N on limit switch error" and disables every joint's enable
        output machine-wide (LinuxCNC's intended last-resort crash
        protection -- see docs/hardware_setup.md). LinuxCNC only *skips*
        that fault while its own native homing state machine is running
        (command.home()); software homing here never uses that state
        machine, it issues plain JOG_CONTINUOUS commands instead. Without
        this, the instant the switch trips, LinuxCNC would fault the whole
        machine before poll_homing()'s own poll of min_limit_active()/
        max_limit_active() ever gets a chance to react and stop the jog.

        command.override_limits() is LinuxCNC's own mechanism for exactly
        this situation (the same one its "Override Limits" GUI checkbox
        sends) -- it suppresses the hard-limit fault machine-wide until this
        joint is back in position (i.e. until soft_stop() brings the jog to
        a stop), at which point LinuxCNC re-arms it automatically. Native
        homing (use_linuxcnc_native_processes=True) never calls this method,
        since it doesn't need it -- see start_homing().
        """
        self.command.override_limits()
        if positive:
            self.jog_positive(self._homing_speed)
        else:
            self.jog_negative(self._homing_speed)

    def is_idle(self, velocity_tolerance: float = 1e-6):
        self.poll()

        joint = self.status.joint[self.joint.joint]

        return (
            joint["inpos"] and
            abs(joint["velocity"]) <= velocity_tolerance
        )
