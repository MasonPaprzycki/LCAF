from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from pathlib import Path

_logger = logging.getLogger(__name__)


@dataclass(slots=True)
class JointConfiguration:
    """
    Describes one physical CNC joint driven by a single stepper motor.

    A "joint" here is a LinuxCNC concept: one stepper-driven degree of
    freedom, numbered 0-3 (`joint`), that also has a Cartesian letter
    (`axis`, X/Y/Z/A). Each joint owns exactly one Mesa stepgen instance
    (`mesa_stepgen`) and, optionally, one enable output and a pair of limit
    inputs. See docs/hardware_setup.md section 2 ("What is a joint?") for
    the full joint <-> stepgen <-> terminal-block mapping.

    This object is the hardware description of one joint. It is used by
    the LinuxCNC HAL/INI generator (generate_hal/generate_ini below) and by
    LinuxCNCAxialInterface (lcaf.control.linuxcnc_interface) at runtime.

    Units: every distance/speed/acceleration field on this class is in the
    joint's *own* native unit -- inches (and inches/s, inches/s^2) for a
    linear joint (is_angular=False), degrees (deg/s, deg/s^2) for the
    angular joint (is_angular=True). There is deliberately no "_mm"/"_in"
    suffix on these fields: the same field means a different unit depending
    on is_angular, so a fixed suffix would be wrong for one case or the
    other. docs/hardware_setup.md has a units table spelling this out
    per-field if you're ever unsure.

    Every joint's usable range is [-retracted_distance, extended_distance]
    in its native unit -- two independent, always-positive distances
    measured from the joint's own zero position (see retracted_distance/
    extended_distance below), regardless of whether that zero comes from a
    physical limit switch or not:
    A switched joint (has_limit_switches=True; X/Y/Z here) has 0 wherever
    LinuxCNCAxialInterface.start_homing()/poll_homing() finds the negative
    limit switch at homing time, regardless of the physical machine's
    absolute position -- retracted_distance is normally 0 for these (the
    switch itself is the retracted end of travel).
    The switchless angular joint (has_limit_switches=False, is_angular=True;
    A here) instead has 0 wherever it happened to be sitting at power-on,
    since it has no reference switch to seek, and it is free to rotate
    either direction from there since this project's A joint has no
    cable-wrap constraint. See docs/hardware_setup.md.

    Either distance may instead be left None to mean "no software soft
    limit on this end of travel" (this project's A joint uses this for
    both, since it is genuinely unbounded -- see retracted_distance/
    extended_distance below): generate_ini() then omits that end's
    MIN_LIMIT/MAX_LIMIT entirely, which is itself LinuxCNC's own documented
    way of saying "no limit" (it substitutes -1e99/1e99). __post_init__
    logs a warning whenever this happens, since it genuinely disables a
    safety check -- only a physical limit switch or mechanical stop
    protects that end of travel from then on.

    Target platform:
        Raspberry Pi host running LinuxCNC
        Mesa 7I76E / 7I76EU Ethernet step/dir + I/O card (driver hm2_eth)

    All pin names refer to Mesa HAL pin names rather than Raspberry Pi GPIOs.
    See docs/hardware_setup.md for the 7I76E terminal-block wiring these
    pin names correspond to.
    """

    # Identity
    joint: int
    """LinuxCNC joint number (0-3 in this project). Selects the stepgen
    instance and every generated joint.N.* HAL pin."""

    axis: str
    """Cartesian letter this joint is bound to (X/Y/Z/A). trivkins maps
    this 1:1 to the joint number -- there is no shared kinematics here."""

    # Stepper Motor
    motor_steps_per_revolution: int
    """Full steps per motor shaft revolution (e.g. 200 for a 1.8 degree/step motor)."""

    microsteps: int
    """Microstep multiplier set on the external stepper driver (e.g. 8)."""

    travel_per_motor_rev: float
    """
    How far the joint moves for one full motor revolution, in the joint's
    native unit: inches/rev for a linear joint (e.g. a leadscrew's pitch),
    degrees/rev for the angular joint (e.g. a belt/gearbox reduction ratio
    expressed as output-degrees per motor turn). Used with
    motor_steps_per_revolution and microsteps to derive steps_per_unit.
    """

    max_velocity: float
    max_acceleration: float
    """
    Linear joint: inches/s and inches/s^2.
    Angular joint: deg/s and deg/s^2.
    """

    # Mesa HAL Pins
    mesa_stepgen: str
    """
    The Mesa stepgen instance driving this joint, e.g.:
        hm2_7i76e.0.stepgen.00
    """

    enable_output: str | None = None
    """Field output pin wired to the driver's enable input, e.g.
    hm2_7i76e.0.7i76.0.0.output-00. None if this joint's driver is always
    enabled (not recommended -- see docs/hardware_setup.md)."""

    negative_limit_input: str | None = None
    """Physical Mesa field-input pin wired to the negative (zero-end) limit
    switch, e.g. hm2_7i76e.0.7i76.0.0.input-00. None if has_limit_switches
    is False. When MachineConfiguration.use_linuxcnc_native_processes is
    True, this same signal also doubles as LinuxCNC's native home switch
    (joint.N.home-sw-in) -- see generate_hal()."""

    positive_limit_input: str | None = None
    """Physical Mesa field-input pin wired to the positive (max-travel-end)
    limit switch, e.g. hm2_7i76e.0.7i76.0.0.input-01. None if
    has_limit_switches is False."""

    # Joint kinematics
    is_angular: bool = False
    """True for the rotary joint (INI TYPE = ANGULAR, values interpreted in
    degrees). False for a linear joint (INI TYPE = LINEAR, values in inches)."""

    has_limit_switches: bool = True
    """
    False for a joint with no physical limit switches (e.g. a continuously
    rotating axis): start_homing() zeroes the joint at its current position
    instead of seeking a switch, and retracted_distance/extended_distance
    are trusted as configured rather than being measured. See
    LinuxCNCAxialInterface.start_homing/poll_homing.
    """

    dual_limit_switches: bool = True
    """
    Only meaningful when has_limit_switches is True (ignored for a
    switchless joint like A). Selects which of two software-homing
    strategies a switched joint uses -- see
    LinuxCNCAxialInterface._start_software_homing/poll_homing.

    True (default -- this project's original X/Y/Z wiring, a switch at
    each end of travel per docs/hardware_setup.md section 5): homing seeks
    the negative limit switch to establish zero, then also seeks the
    positive limit switch and *measures* the actual travel between them,
    overriding the configured extended_distance with that live measurement.
    Requires positive_limit_input to be set.

    False: this joint has only a negative (zero-end) limit switch wired --
    positive_limit_input must be None. Homing seeks the negative switch to
    establish zero exactly as above, but then stops there and trusts the
    configured extended_distance as a static software limit instead of
    measuring it (there is no second switch to seek). Useful when a joint's
    far end of travel is bounded by a known mechanical stop (leadscrew end,
    frame limit) that isn't worth wiring a second switch to.

    Native homing (MachineConfiguration.use_linuxcnc_native_processes=True)
    behaves the same regardless of this flag: LinuxCNC's own native homing
    sequence never remeasures travel either way (MAX_LIMIT is always the
    static configured value in the generated INI -- see generate_ini()), so
    this only changes behavior under this project's own software homing.
    """

    retract_to: float | None = None
    """
    Absolute position (in this joint's own native unit -- inches or
    degrees -- measured from its own zero, the same frame as
    retracted_distance/extended_distance) that this joint should move to
    on "retract" instead of re-seeking its negative limit switch. This
    project's Y axis is mechanically arranged so its retracted position is
    partway into its positive-direction travel rather than at zero, so it
    is the one joint with this set (to 1.5 -- not all the way to its own
    extended_distance soft limit). Read by LinuxCNCAxialInterface.
    start_retract_to_zero()/poll_retract_to_zero(): instead of re-seeking
    the negative limit switch (the normal retract, which also
    re-references position_offset_to_native against physical drift), it
    commands a plain move to retract_to in the coordinate frame already
    established by the last full home_all() -- there is no limit switch at
    this position to re-seek against, so nothing here corrects for drift
    the way the negative-switch retract does.

    None (default) for every other joint -- retract behaves as always,
    re-seeking the negative limit switch. When set, must fall within
    [-retracted_distance, extended_distance] if those are configured (see
    __post_init__).
    """

    # Signal polarity. `inverted` is the one supported software fix for a
    # wiring/motor mismatch: it flips the stepgen's position-scale sign in
    # HAL, which reverses that joint's motion direction end-to-end without
    # re-wiring anything. This is the normal fix for a reversed motor/lead
    # direction. Z in this project's axis.json has this set (see
    # docs/hardware_setup.md). A step-signal or enable-signal polarity
    # mismatch is a physical-layer problem the 7I76E has no HAL invert pin
    # for -- that's fixed by swapping wires at the terminal block, not a
    # config flag (see docs/hardware_setup.md).
    inverted: bool = False

    invert_negative_limit: bool = False
    invert_positive_limit: bool = False
    """
    Set if a limit switch reads "active" when open and "inactive" when
    closed (backwards from the normally-closed wiring this project assumes
    -- see docs/hardware_setup.md "Limit switch wiring"). Reads the field
    input component's inverted "-not" sibling pin instead of the raw one.
    Fully supported in HAL (see generate_hal()).
    """

    # Native-homing sequencing. Only meaningful when
    # MachineConfiguration.use_linuxcnc_native_processes is True, in which
    # case generate_ini() uses this joint's own number (0-3) as its
    # HOME_SEQUENCE when this is left at -1, so "Home All" homes joints one
    # at a time in joint-number order. Set explicitly only to home two
    # joints simultaneously (matching LinuxCNC's HOME_SEQUENCE semantics).
    # Inert when use_linuxcnc_native_processes is False.
    home_sequence: int = -1

    # Software travel limits. When set, both are always-positive distances,
    # in this joint's own native unit (inches for linear, degrees for
    # angular), measured from its own zero position (see the class
    # docstring) -- the joint's soft-limited range is always
    # [-retracted_distance, extended_distance]. Normally set for every
    # joint (generate_ini() uses them, regardless of has_limit_switches, as
    # the INI's MIN_LIMIT/MAX_LIMIT -- see generate_ini()); either may be
    # left None instead to genuinely disable that end's software limit
    # (this project's A joint does this for both -- see below), in which
    # case generate_ini() omits that entry rather than inventing a
    # placeholder, matching LinuxCNC's own "no limit" semantics.
    #   - A switched linear joint (X/Y/Z): retracted_distance is normally 0
    #     -- 0 is wherever homing finds the negative limit switch (measured
    #     in software mode, or wherever LinuxCNC's own native homing lands
    #     in native mode -- see LinuxCNCAxialInterface.start_homing/
    #     poll_homing), and that switch is the physical retracted end of
    #     travel too. extended_distance is the static envelope LinuxCNC's
    #     INI soft limits use (or, for a dual_limit_switches joint, gets
    #     overridden by the live measurement to the positive switch).
    #   - The switchless angular joint (A): 0 is simply wherever it was
    #     sitting when it powered on (there is no reference switch to
    #     seek), and since this project has no cable-wrap constraint on A,
    #     it is free to rotate either direction from there with genuinely
    #     no travel limit -- so both are left None rather than carrying an
    #     arbitrary large sentinel value.
    retracted_distance: float | None = None
    """Positive distance from zero to this joint's retracted (negative-
    direction) soft limit, in inches (linear) or degrees (angular), or None
    to disable this joint's negative-direction soft limit entirely (logs a
    warning -- see __post_init__). Becomes -retracted_distance in the
    generated INI's MIN_LIMIT when set -- see min_travel."""

    extended_distance: float | None = None
    """Positive distance from zero to this joint's extended (positive-
    direction) soft limit, in inches (linear) or degrees (angular), or None
    to disable this joint's positive-direction soft limit entirely (logs a
    warning -- see __post_init__). Becomes the generated INI's MAX_LIMIT
    directly when set."""

    # Step timing (nanoseconds) -- signal-level, not a "freedom units" concern.
    step_length_ns: int = 5000
    step_space_ns: int = 5000
    direction_setup_ns: int = 20000
    direction_hold_ns: int = 20000

    # Derived: steps per inch (linear joints) or steps per degree (angular).
    steps_per_unit: float = field(init=False)

    def __post_init__(self):

        self.axis = self.axis.upper()

        valid_axes = {"X", "Y", "Z", "A", "B", "C", "U", "V", "W"}

        if self.axis not in valid_axes:
            raise ValueError(f"Unsupported axis '{self.axis}'.")

        if self.motor_steps_per_revolution <= 0:
            raise ValueError("motor_steps_per_revolution must be positive.")

        if self.microsteps <= 0:
            raise ValueError("microsteps must be positive.")

        if self.travel_per_motor_rev <= 0:
            raise ValueError("travel_per_motor_rev must be positive.")

        if self.max_velocity <= 0:
            raise ValueError("max_velocity must be positive.")

        if self.max_acceleration <= 0:
            raise ValueError("max_acceleration must be positive.")

        if self.retracted_distance is not None and self.retracted_distance < 0:
            raise ValueError(
                f"Joint {self.joint} ({self.axis}): retracted_distance must not be negative "
                "(it is a positive distance from zero -- see its docstring)."
            )

        if self.extended_distance is not None and self.extended_distance <= 0:
            raise ValueError(
                f"Joint {self.joint} ({self.axis}): extended_distance must be positive "
                "(it is a positive distance from zero -- see its docstring)."
            )

        if self.retracted_distance is None:
            _logger.warning(
                f"Joint {self.joint} ({self.axis}): retracted_distance is None -- this joint's "
                "negative-direction software soft limit is disabled (generate_ini() will omit "
                "MIN_LIMIT, and LinuxCNC itself defaults that to -1e99). Only a physical limit "
                "switch or mechanical stop protects this end of travel now."
            )

        if self.extended_distance is None:
            _logger.warning(
                f"Joint {self.joint} ({self.axis}): extended_distance is None -- this joint's "
                "positive-direction software soft limit is disabled (generate_ini() will omit "
                "MAX_LIMIT, and LinuxCNC itself defaults that to 1e99). Only a physical limit "
                "switch or mechanical stop protects this end of travel now."
            )

        if self.retract_to is not None:
            if self.retracted_distance is not None and self.retract_to < -self.retracted_distance:
                raise ValueError(
                    f"Joint {self.joint} ({self.axis}): retract_to ({self.retract_to}) is beyond "
                    f"this joint's retracted-direction soft limit (-{self.retracted_distance})."
                )
            if self.extended_distance is not None and self.retract_to > self.extended_distance:
                raise ValueError(
                    f"Joint {self.joint} ({self.axis}): retract_to ({self.retract_to}) is beyond "
                    f"this joint's extended-direction soft limit ({self.extended_distance})."
                )

        if (
            self.negative_limit_input is not None
            and self.positive_limit_input is not None
            and self.negative_limit_input == self.positive_limit_input
        ):
            raise ValueError(
                f"Joint {self.joint} ({self.axis}): negative_limit_input and "
                f"positive_limit_input are both '{self.negative_limit_input}' -- they must be "
                "wired to different physical inputs, or homing/limit checks cannot tell which "
                "end of travel was reached."
            )

        if self.has_limit_switches and self.dual_limit_switches and self.positive_limit_input is None:
            raise ValueError(
                f"Joint {self.joint} ({self.axis}): dual_limit_switches=True requires "
                "positive_limit_input to be set (homing seeks it to measure travel) -- set "
                "dual_limit_switches=False if this joint only has a negative limit switch."
            )

        if self.has_limit_switches and not self.dual_limit_switches and self.positive_limit_input is not None:
            raise ValueError(
                f"Joint {self.joint} ({self.axis}): dual_limit_switches=False means this joint "
                "has no positive limit switch wired -- positive_limit_input must be None. Set "
                "dual_limit_switches=True if a positive switch really exists, so travel is "
                "measured instead of trusted from extended_distance."
            )

        self.steps_per_unit = (
            self.motor_steps_per_revolution
            * self.microsteps
            / self.travel_per_motor_rev
        )

    @property
    def negative_limit_hal_pin(self) -> str:
        return f"joint.{self.joint}.neg-lim-sw-in"

    @property
    def positive_limit_hal_pin(self) -> str:
        return f"joint.{self.joint}.pos-lim-sw-in"

    @property
    def joint_prefix(self):
        return f"joint.{self.joint}"

    @property
    def motor_position_command_pin(self):
        return f"{self.joint_prefix}.motor-pos-cmd"

    @property
    def motor_position_feedback_pin(self):
        return f"{self.joint_prefix}.motor-pos-fb"

    @property
    def ini_joint_section(self) -> str:
        return f"JOINT_{self.joint}"

    @property
    def ini_axis_section(self) -> str:
        return f"AXIS_{self.axis}"

    @property
    def unit_label(self) -> str:
        """Human-readable unit label for this joint's distance fields, for
        logging/telemetry/docs -- "deg" for the angular joint, "in" otherwise."""
        return "deg" if self.is_angular else "in"

    @property
    def min_travel(self) -> float | None:
        """The INI MIN_LIMIT for this joint: -retracted_distance, or None if
        retracted_distance is not configured (generate_ini() then omits
        MIN_LIMIT -- see retracted_distance's docstring)."""
        if self.retracted_distance is None:
            return None
        return -self.retracted_distance

    @classmethod
    def from_dict(cls, data: dict) -> "JointConfiguration":
        known = {f for f in cls.__dataclass_fields__ if cls.__dataclass_fields__[f].init}
        filtered = {k: v for k, v in data.items() if k in known}
        return cls(**filtered)

    def to_dict(self) -> dict:
        return {
            f: getattr(self, f)
            for f in self.__dataclass_fields__
            if self.__dataclass_fields__[f].init
        }


@dataclass(slots=True)
class MachineConfiguration:
    """
    Machine-wide settings needed to render a complete LinuxCNC .hal/.ini pair.
    Combined with the JointConfiguration list, this is the single source of
    truth the generator and the runtime interface both read from.

    Unlike JointConfiguration's fields, every field here is unambiguously
    one unit or the other (there's a separate default_linear_*/
    default_angular_* pair for each), so field names carry their unit
    suffix directly: "_in_s" (inches/second), "_in_s2" (inches/second^2),
    "_deg_s", "_deg_s2", "_ns" (nanoseconds).
    """

    machine_name: str

    linear_units: str = "inch"
    """LinuxCNC [TRAJ]LINEAR_UNITS. This machine is configured entirely in
    inches -- see docs/hardware_setup.md "Units" section."""
    angular_units: str = "degree"

    default_linear_velocity_in_s: float = 0.393701
    max_linear_velocity_in_s: float = 0.984252
    max_linear_acceleration_in_s2: float = 15.748031

    default_angular_velocity_deg_s: float = 10.0
    max_angular_velocity_deg_s: float = 45.0
    max_angular_acceleration_deg_s2: float = 200.0

    base_period_ns: int = 50_000
    servo_period_ns: int = 1_000_000

    mesa_board_driver: str = "hm2_eth"
    """HAL loadrt module name for the Mesa board driver, e.g. hm2_eth (7I76E/7I76EU)."""

    mesa_board_config: str = ""
    """
    Literal argument text appended to 'loadrt <mesa_board_driver>' as-is, e.g.
    'board_ip="192.168.1.121" config="num_encoders=1 num_pwmgens=0 num_stepgens=5"'
    for a 7I76E/7I76EU on hm2_eth.
    """

    watchdog_timeout_ns: int = 5_000_000

    use_linuxcnc_native_processes: bool = False
    """
    Chooses which of two entirely different homing/motion-permission
    strategies LinuxCNC runs under, machine-wide (this is not a per-joint
    setting -- every joint homes the same way):

    False (default): homing is done in Python by LinuxCNCAxialInterface --
    jogging each switched joint to its limit switches directly, or zeroing
    a switchless joint immediately (see JointConfiguration.has_limit_switches).
    LinuxCNC's own [JOINT_n]HOME_SEQUENCE is left at -1 (native homing never
    runs) and generate_ini() sets [TRAJ]NO_FORCE_HOMING = 1, because
    LinuxCNC would otherwise never see status.homed become true for any
    joint (that's tracked in this project's own Axis.is_homed() bookkeeping
    instead) and would refuse every MDI move/program run.

    True: LinuxCNC's own native homing sequence runs instead --
    LinuxCNCAxialInterface.start_homing()/poll_homing() calls command.home(joint)
    and waits (without blocking -- see poll_homing()) for status.joint[n]['homed'],
    generate_hal() wires each switched
    joint's negative-limit input to its HOME_SEARCH_VEL-driven
    joint.N.home-sw-in pin too, and generate_ini() fills in real
    HOME_SEARCH_VEL/HOME_LATCH_VEL/HOME_SEQUENCE values. Because LinuxCNC
    genuinely knows every joint is homed, NO_FORCE_HOMING is left off (0)
    -- there is no risk of LinuxCNC rejecting a subsequent MDI move or jog.
    """

    linear_ferror_in: float = 1.0 / 25.4
    linear_min_ferror_in: float = 0.25 / 25.4
    angular_ferror_deg: float = 1.0
    angular_min_ferror_deg: float = 0.25
    """
    [JOINT_n]FERROR/MIN_FERROR for linear (X/Y/Z) and angular (A) joints --
    the maximum following error LinuxCNC will tolerate before faulting the
    joint (see docs/hardware_setup.md). This is the only stall/skipped-step
    detection this machine has (see docs/potential_issues.md): none of the
    four joints have position feedback of their own, so LinuxCNC's own
    commanded-vs-actual comparison, using exactly these tolerances, is what
    actually throws the fault. Tune these here (not by editing generated
    files) once real commissioning data is available -- the shipped
    defaults are reasonable starting points, not measured values.
    """

    joints: list[JointConfiguration] = field(default_factory=list)

    def joint_by_axis(self, axis: str) -> JointConfiguration:
        axis = axis.upper()
        for joint in self.joints:
            if joint.axis == axis:
                return joint
        raise KeyError(f"No joint configured for axis '{axis}'.")

    @property
    def coordinates(self) -> str:
        return "".join(joint.axis for joint in sorted(self.joints, key=lambda j: j.joint))

    @classmethod
    def from_dict(cls, data: dict, joints: list[JointConfiguration]) -> "MachineConfiguration":
        known = {f for f in cls.__dataclass_fields__ if f != "joints"}
        filtered = {k: v for k, v in data.items() if k in known}
        return cls(joints=joints, **filtered)


def load_joint_configurations(path: str | Path) -> list[JointConfiguration]:
    """
    Load one JointConfiguration per element of a JSON file containing an
    array of joint objects.
    """

    path = Path(path)

    if not path.exists():
        raise FileNotFoundError(path)

    with open(path, "r") as file:
        try:
            entries = json.load(file)
        except json.JSONDecodeError as e:
            raise ValueError(f"{path}: invalid joint configuration: {e}") from e

    if not isinstance(entries, list):
        raise ValueError(f"{path}: expected a JSON array of joint configurations.")

    joints: list[JointConfiguration] = []

    for index, data in enumerate(entries):
        try:
            joints.append(JointConfiguration.from_dict(data))
        except (TypeError, ValueError) as e:
            raise ValueError(f"{path}[{index}]: invalid joint configuration: {e}") from e

    if not joints:
        raise ValueError(f"{path}: no joint configurations found.")

    return joints


def load_machine_configuration(
    machine_path: str | Path,
    joints_path: str | Path | None = None,
) -> MachineConfiguration:
    """
    Load the machine-wide settings from `machine_path` (JSON) and the joint
    list from `joints_path` (JSON). If `joints_path` is not given, it is
    resolved relative to `machine_path`'s directory using the
    'axis_config_path' key in the machine JSON (default 'axis.json').
    """

    machine_path = Path(machine_path)

    if not machine_path.exists():
        raise FileNotFoundError(machine_path)

    with open(machine_path, "r") as file:
        data = json.load(file)

    resolved_joints_path = (
        joints_path
        if joints_path is not None
        else machine_path.parent / data.get("axis_config_path", "axis.json")
    )

    joints = load_joint_configurations(resolved_joints_path)

    return MachineConfiguration.from_dict(data, joints)


# ---------------------------------------------------------------------------
# HAL / INI generation
#
# These functions render a complete, ready-to-run LinuxCNC configuration
# from a MachineConfiguration. They have no runtime dependency on the
# `linuxcnc`/`hal` python modules -- they only produce text -- so they can
# run on any machine (including outside the Raspberry Pi/LinuxCNC target)
# as part of build/bootstrap tooling.
# ---------------------------------------------------------------------------

def _inverted_input_pin(pin: str) -> str:
    """
    The 7I76E's field-input component exposes an inverted-reading sibling
    pin for every input named "<pin>-not" (e.g. input-00 -> input-00-not).
    """

    return f"{pin}-not"


#  How close a simulated joint's looped-back position has to get to its
#  ideal min_travel / extended_distance before generate_hal(simulate=True)'s
#  fake limit switches trip. Arbitrary -- there is no real switch involved,
#  this only needs to be small relative to a joint's own travel and bigger
#  than floating-point noise.
_SIM_LIMIT_SWITCH_HYSTERESIS = 0.01


def generate_hal(machine: MachineConfiguration, simulate: bool = False) -> str:
    """
    Render the .hal file that wires this machine's joints for every
    configured joint.

    simulate=False (default): wires the real Mesa stepgens/GPIO, exactly as
    this machine will actually run.

    simulate=True: replaces the Mesa board and stepgens with a direct
    position-command-to-feedback loopback per joint (the same pattern
    LinuxCNC's own shipped sim configs use, see
    /usr/share/linuxcnc/hallib/core_sim.hal) and fake limit switches built
    from the HAL `comp` comparator component, so this exact
    MachineConfiguration -- same joints, same units, same homing mode, same
    INI -- can run against a real LinuxCNC instance with no physical
    hardware attached, for testing this project's control code
    end-to-end. See docs/potential_issues.md.
    """

    lines: list[str] = []

    def emit(text: str = ""):
        lines.append(text)

    emit(f"# Generated by lcaf.utils.joint_configuration for machine '{machine.machine_name}'.")
    if simulate:
        emit("# SIMULATED HARDWARE -- no Mesa board, no real motors. See generate_hal() docstring.")
    emit("# Do not edit by hand -- regenerate from the machine/axis config files.")
    emit()
    emit("loadrt trivkins")
    emit(f"loadrt motmod base_period_nsec={machine.base_period_ns} servo_period_nsec={machine.servo_period_ns}")

    if not simulate:
        emit("loadrt hostmot2")
        board_line = f"loadrt {machine.mesa_board_driver}"
        if machine.mesa_board_config:
            board_line += f" {machine.mesa_board_config}"
        emit(board_line)

    joints = sorted(machine.joints, key=lambda j: j.joint)

    if simulate:
        comp_names = []
        for joint in joints:
            if not joint.has_limit_switches:
                continue
            comp_names.append(f"complim_{joint.axis.lower()}_neg")
            if joint.dual_limit_switches:
                comp_names.append(f"complim_{joint.axis.lower()}_pos")
        if comp_names:
            emit(f"loadrt comp names={','.join(comp_names)}")

    emit()

    if simulate:
        emit("addf motion-command-handler servo-thread")
        emit("addf motion-controller servo-thread")
        for joint in joints:
            if joint.has_limit_switches:
                emit(f"addf complim_{joint.axis.lower()}_neg servo-thread")
                if joint.dual_limit_switches:
                    emit(f"addf complim_{joint.axis.lower()}_pos servo-thread")
    else:
        board_name = joints[0].mesa_stepgen.split(".stepgen.")[0]
        # capture-position/update-freq are per-board stepgen-module functions
        # (no channel number) -- each processes every enabled stepgen.NN
        # channel on this board internally. There is no per-channel
        # "stepgen.NN.capture-position" HAL function; addf'ing one per joint
        # fails with "function ... not found". See hostmot2(9).
        emit(f"addf {board_name}.read servo-thread")
        emit(f"addf {board_name}.stepgen.capture-position servo-thread")
        emit("addf motion-command-handler servo-thread")
        emit("addf motion-controller servo-thread")
        emit(f"addf {board_name}.stepgen.update-freq servo-thread")
        emit(f"addf {board_name}.write servo-thread")

    emit()
    emit("net watchdog-reset <= motion.motion-enabled")
    emit()

    for joint in joints:
        emit(f"# --- Joint {joint.joint} ({joint.axis}) ---")

        if simulate:
            pos_signal = f"{joint.axis.lower()}-pos"
            emit(f"net {pos_signal} {joint.motor_position_command_pin} => {joint.motor_position_feedback_pin}")

            if joint.has_limit_switches:
                if joint.retracted_distance is None:
                    raise ValueError(
                        f"Joint {joint.joint} ({joint.axis}): retracted_distance is None -- "
                        "generate_hal(simulate=True) needs a concrete retracted_distance to "
                        "build this joint's simulated negative limit switch position."
                    )
                if joint.dual_limit_switches and joint.extended_distance is None:
                    raise ValueError(
                        f"Joint {joint.joint} ({joint.axis}): extended_distance is None -- "
                        "generate_hal(simulate=True) needs a concrete extended_distance to "
                        "build this joint's simulated positive limit switch position "
                        "(dual_limit_switches=True)."
                    )
                neg_comp = f"complim_{joint.axis.lower()}_neg"
                hyst = _SIM_LIMIT_SWITCH_HYSTERESIS

                # comp.out is TRUE when in1 > in0 (with hysteresis) -- see
                # https://linuxcnc.org/docs/2.9/html/man/man9/comp.9.html.
                # Negative switch: trips once position drops below ~min_travel
                # (in0 = live position, in1 = the near-min_travel threshold --
                # normally ~0, since retracted_distance is normally 0 for a
                # switched joint).
                # Positive switch (dual_limit_switches only -- mirrors
                # whether positive_limit_input is actually wired on real
                # hardware): trips once position rises above
                # ~extended_distance (in0 = the near-extended_distance
                # threshold, in1 = live position).
                if joint.dual_limit_switches:
                    pos_comp = f"complim_{joint.axis.lower()}_pos"
                    emit(f"net {pos_signal} => {neg_comp}.in0 {pos_comp}.in1")
                    emit(f"setp {pos_comp}.in0 {round(joint.extended_distance - hyst, 6)}")
                    emit(f"setp {pos_comp}.hyst {hyst}")
                else:
                    emit(f"net {pos_signal} => {neg_comp}.in0")

                emit(f"setp {neg_comp}.in1 {round(joint.min_travel + hyst, 6)}")
                emit(f"setp {neg_comp}.hyst {hyst}")

                neg_signal = f"{joint.axis.lower()}-neg-lim"
                emit(f"net {neg_signal} {neg_comp}.out => {joint.negative_limit_hal_pin}")
                if machine.use_linuxcnc_native_processes:
                    emit(f"net {neg_signal} => {joint.joint_prefix}.home-sw-in")
                if joint.dual_limit_switches:
                    emit(f"net {joint.axis.lower()}-pos-lim {pos_comp}.out => {joint.positive_limit_hal_pin}")
            else:
                emit(f"# Joint {joint.joint} ({joint.axis}) has no limit switches; homed by "
                     "zeroing at boot (has_limit_switches=False in axis.json).")

            emit()
            continue

        scale = joint.steps_per_unit
        if joint.inverted:
            scale = -scale

        emit(f"setp {joint.mesa_stepgen}.step_type 0")
        emit(f"setp {joint.mesa_stepgen}.steplen {joint.step_length_ns}")
        emit(f"setp {joint.mesa_stepgen}.stepspace {joint.step_space_ns}")
        emit(f"setp {joint.mesa_stepgen}.dirsetup {joint.direction_setup_ns}")
        emit(f"setp {joint.mesa_stepgen}.dirhold {joint.direction_hold_ns}")
        emit(f"setp {joint.mesa_stepgen}.position-scale {scale}")
        emit(f"setp {joint.mesa_stepgen}.maxvel [{joint.ini_joint_section}]STEPGEN_MAXVEL")
        emit(f"setp {joint.mesa_stepgen}.maxaccel [{joint.ini_joint_section}]STEPGEN_MAXACCEL")

        emit(f"net {joint.axis.lower()}-pos-cmd {joint.motor_position_command_pin} => {joint.mesa_stepgen}.position-cmd")
        emit(f"net {joint.axis.lower()}-pos-fb {joint.mesa_stepgen}.position-fb => {joint.motor_position_feedback_pin}")

        enable_signal = f"net {joint.axis.lower()}-enable {joint.joint_prefix}.amp-enable-out => {joint.mesa_stepgen}.enable"
        if joint.enable_output:
            enable_signal += f" => {joint.enable_output}"
        emit(enable_signal)

        if joint.negative_limit_input:
            neg_pin = joint.negative_limit_input
            if joint.invert_negative_limit:
                neg_pin = _inverted_input_pin(neg_pin)
            neg_signal = f"{joint.axis.lower()}-neg-lim"
            emit(f"net {neg_signal} {neg_pin} => {joint.negative_limit_hal_pin}")
            if machine.use_linuxcnc_native_processes:
                # Same physical switch also serves as LinuxCNC's native home
                # switch -- see MachineConfiguration.use_linuxcnc_native_processes.
                emit(f"net {neg_signal} => {joint.joint_prefix}.home-sw-in")
        elif not joint.has_limit_switches:
            emit(f"# Joint {joint.joint} ({joint.axis}) has no limit switches; homed by "
                 "zeroing at boot (has_limit_switches=False in axis.json).")

        if joint.positive_limit_input:
            pos_pin = joint.positive_limit_input
            if joint.invert_positive_limit:
                pos_pin = _inverted_input_pin(pos_pin)
            emit(f"net {joint.axis.lower()}-pos-lim {pos_pin} => {joint.positive_limit_hal_pin}")

        emit()

    emit("# --- Machine-wide E-Stop chain ---")
    emit("net estop-loop iocontrol.0.user-enable-out => iocontrol.0.emc-enable-in")

    return "\n".join(lines) + "\n"


def _format_ini_value(value) -> str:
    if isinstance(value, bool):
        return "1" if value else "0"
    return str(value)


# Default following-error tolerances. A following error is how far behind
# its commanded position LinuxCNC will let a joint fall before faulting --
# too tight and normal accel/decel trips false faults, too loose and a
# genuinely stalled/skipping motor goes undetected. These are reasonable
# starting points, not measured per-joint; expect to tune them during
# commissioning (see docs/potential_issues.md). The actual values are
# MachineConfiguration.linear_ferror_in / linear_min_ferror_in /
# angular_ferror_deg / angular_min_ferror_deg -- machine-wide, configured in
# machine.json rather than hardcoded here, so they can be tuned without
# touching code.

# Fraction of a joint's own max_velocity used as its native-homing search
# and final latch speed when MachineConfiguration.use_linuxcnc_native_processes
# is True. Not user-configurable: homing speed has always been a
# conservative fixed fraction of normal running speed in this project (see
# the previous fixed 0.03937 in/s software-homing default), not a tunable a
# user is expected to reach for.
_NATIVE_HOME_SEARCH_VEL_FRACTION = 0.1
_NATIVE_HOME_LATCH_VEL_FRACTION = 0.02


def generate_ini(machine: MachineConfiguration) -> str:
    """
    Render the .ini file matching the .hal produced by generate_hal() for
    the same MachineConfiguration.
    """

    joints = sorted(machine.joints, key=lambda j: j.joint)
    coordinates = machine.coordinates

    sections: list[tuple[str, list[tuple[str, str]]]] = []

    def section(name: str, entries: list[tuple[str, str]]):
        sections.append((name, entries))

    section("EMC", [
        ("VERSION", "1.1"),
        ("MACHINE", machine.machine_name),
        ("DEBUG", "0"),
    ])

    section("DISPLAY", [
        ("DISPLAY", "axis"),
        ("POSITION_OFFSET", "RELATIVE"),
        ("POSITION_FEEDBACK", "ACTUAL"),
        ("MAX_FEED_OVERRIDE", "1.2"),
        ("MAX_SPINDLE_OVERRIDE", "1.0"),
        ("MIN_SPINDLE_OVERRIDE", "0.5"),
        # Machine units per second, literally -- the same value [TRAJ] uses
        # below, with no per-minute conversion. See
        # MachineConfiguration.default_linear_velocity_in_s docstring.
        ("DEFAULT_LINEAR_VELOCITY", _format_ini_value(machine.default_linear_velocity_in_s)),
        ("MAX_LINEAR_VELOCITY", _format_ini_value(machine.max_linear_velocity_in_s)),
    ])

    section("FILTER", [])

    section("TASK", [
        ("TASK", "milltask"),
        ("CYCLE_TIME", "0.010"),
    ])

    section("RS274NGC", [
        ("PARAMETER_FILE", f"{machine.machine_name}.var"),
    ])

    section("EMCMOT", [
        ("EMCMOT", "motmod"),
        ("COMM_TIMEOUT", "1.0"),
        ("BASE_PERIOD", str(machine.base_period_ns)),
        ("SERVO_PERIOD", str(machine.servo_period_ns)),
    ])

    section("HAL", [
        ("HALFILE", f"{machine.machine_name}.hal"),
    ])

    section("KINS", [
        ("KINEMATICS", f"trivkins coordinates={coordinates}"),
        ("JOINTS", str(len(joints))),
    ])

    section("TRAJ", [
        ("COORDINATES", " ".join(coordinates)),
        ("LINEAR_UNITS", machine.linear_units),
        ("ANGULAR_UNITS", machine.angular_units),
        ("DEFAULT_LINEAR_VELOCITY", _format_ini_value(machine.default_linear_velocity_in_s)),
        ("MAX_LINEAR_VELOCITY", _format_ini_value(machine.max_linear_velocity_in_s)),
        ("DEFAULT_ANGULAR_VELOCITY", _format_ini_value(machine.default_angular_velocity_deg_s)),
        ("MAX_ANGULAR_VELOCITY", _format_ini_value(machine.max_angular_velocity_deg_s)),
        ("POSITION_FILE", "position.txt"),
        # False (use_linuxcnc_native_processes=False, the default): this
        # project's own Python homing never makes LinuxCNC's own
        # status.homed true, so without NO_FORCE_HOMING LinuxCNC would
        # refuse every MDI move/program run forever. True: native homing
        # runs for real, LinuxCNC genuinely knows every joint is homed, so
        # this is left off -- see
        # MachineConfiguration.use_linuxcnc_native_processes.
        ("NO_FORCE_HOMING", "0" if machine.use_linuxcnc_native_processes else "1"),
    ])

    section("EMCIO", [
        ("EMCIO", "io"),
        ("CYCLE_TIME", "0.100"),
        ("TOOL_TABLE", "tool.tbl"),
    ])

    for joint in joints:
        vel = joint.max_velocity
        accel = joint.max_acceleration

        ferror = machine.angular_ferror_deg if joint.is_angular else machine.linear_ferror_in
        min_ferror = machine.angular_min_ferror_deg if joint.is_angular else machine.linear_min_ferror_in

        if machine.use_linuxcnc_native_processes:
            if joint.has_limit_switches:
                # Seek the negative limit switch (also wired as
                # joint.N.home-sw-in, see generate_hal()), then back off and
                # latch at a slower speed -- the standard two-phase LinuxCNC
                # home sequence. HOME_IGNORE_LIMITS=YES because the search
                # deliberately drives up to the same switch the negative
                # soft/hard limit also watches. Native homing never seeks a
                # positive switch or remeasures travel -- MAX_LIMIT below is
                # always the static configured extended_distance regardless
                # of joint.dual_limit_switches, so this branch is identical for
                # a one- or two-limit-switch joint (dual_limit_switches only
                # changes behavior under this project's own software
                # homing -- see LinuxCNCAxialInterface).
                home_search_vel = -abs(vel) * _NATIVE_HOME_SEARCH_VEL_FRACTION
                home_latch_vel = abs(vel) * _NATIVE_HOME_LATCH_VEL_FRACTION
                home_ignore_limits = "YES"
            else:
                # Switchless joint (A): HOME_SEARCH_VEL=0 tells LinuxCNC to
                # treat wherever it currently is as home, with no seek move
                # -- LinuxCNC's own native equivalent of this project's
                # software zero-at-boot homing.
                home_search_vel = 0.0
                home_latch_vel = 0.0
                home_ignore_limits = "NO"
            # HOME_SEQUENCE < 0 in axis.json means "no explicit sequence
            # requested" -- default to this joint's own number, so "Home
            # All" homes joints one at a time in joint-number order.
            home_sequence = joint.home_sequence if joint.home_sequence >= 0 else joint.joint
        else:
            home_search_vel = 0.0
            home_latch_vel = 0.0
            home_ignore_limits = "NO"
            home_sequence = -1

        # LinuxCNC's own documented behavior for an omitted MIN_LIMIT/
        # MAX_LIMIT is to substitute -1e99/1e99 (see the [JOINT_n]/[AXIS_x]
        # MIN_LIMIT/MAX_LIMIT docs) -- so a None retracted_distance/
        # extended_distance (JointConfiguration warns about this already at
        # construction time) is rendered here by simply omitting the entry,
        # rather than this generator inventing its own placeholder number.
        joint_entries = [
            ("TYPE", "ANGULAR" if joint.is_angular else "LINEAR"),
            ("HOME", "0.0"),
            ("HOME_OFFSET", "0.0"),
            ("HOME_SEARCH_VEL", _format_ini_value(round(home_search_vel, 6))),
            ("HOME_LATCH_VEL", _format_ini_value(round(home_latch_vel, 6))),
            ("HOME_SEQUENCE", str(home_sequence)),
            ("HOME_USE_INDEX", "NO"),
            ("HOME_IGNORE_LIMITS", home_ignore_limits),
            ("MAX_VELOCITY", _format_ini_value(vel)),
            ("MAX_ACCELERATION", _format_ini_value(accel)),
            ("STEPGEN_MAXVEL", _format_ini_value(vel * 1.2)),
            ("STEPGEN_MAXACCEL", _format_ini_value(accel * 1.2)),
        ]
        if joint.min_travel is not None:
            joint_entries.append(("MIN_LIMIT", _format_ini_value(joint.min_travel)))
        if joint.extended_distance is not None:
            joint_entries.append(("MAX_LIMIT", _format_ini_value(joint.extended_distance)))
        joint_entries += [
            ("FERROR", _format_ini_value(round(ferror, 6))),
            ("MIN_FERROR", _format_ini_value(round(min_ferror, 6))),
        ]
        section(joint.ini_joint_section, joint_entries)

        axis_entries = []
        if joint.min_travel is not None:
            axis_entries.append(("MIN_LIMIT", _format_ini_value(joint.min_travel)))
        if joint.extended_distance is not None:
            axis_entries.append(("MAX_LIMIT", _format_ini_value(joint.extended_distance)))
        axis_entries += [
            ("MAX_VELOCITY", _format_ini_value(vel)),
            ("MAX_ACCELERATION", _format_ini_value(accel)),
        ]
        section(joint.ini_axis_section, axis_entries)

    rendered = [
        "# Generated by lcaf.utils.joint_configuration for machine "
        f"'{machine.machine_name}'.",
        "# Do not edit by hand -- regenerate from the machine/axis config files.",
        "",
    ]

    for name, entries in sections:
        rendered.append(f"[{name}]")
        for key, value in entries:
            rendered.append(f"{key} = {value}")
        rendered.append("")

    return "\n".join(rendered)


_TOOL_TABLE_FILENAME = "tool.tbl"
"""Must match the [EMCIO]TOOL_TABLE value generate_ini() writes above."""


def _default_tool_table_text() -> str:
    """
    Minimal, valid LinuxCNC tool table.

    LCAF has no tool changer or tool-number logic anywhere in
    lcaf.control -- but LinuxCNC's iocontrol refuses to start at all
    (`iocontrol.cc: can't load tool table`) if [EMCIO]TOOL_TABLE doesn't
    point at a file that exists, even an empty one. This file's *content*
    is therefore irrelevant to this project; it only needs to exist.
    """

    return (
        "; Generated by lcaf.utils.joint_configuration. LCAF has no tool "
        "changer, so this is intentionally empty -- LinuxCNC's iocontrol "
        "just requires the file referenced by [EMCIO]TOOL_TABLE to exist. "
        "Safe to hand-edit: write_config_files() only creates this file if "
        "it's missing, it never overwrites an existing one.\n"
    )


def write_config_files(
    machine: MachineConfiguration,
    output_dir: str | Path,
    simulate: bool = False,
) -> tuple[Path, Path]:
    """
    Render and write '<machine_name>.hal' and '<machine_name>.ini' into
    output_dir, creating it if needed. Returns (hal_path, ini_path).

    Also creates output_dir/tool.tbl with a minimal empty tool table if one
    doesn't already exist there -- see _default_tool_table_text(). Unlike
    the .hal/.ini (rewritten every call), an existing tool.tbl is never
    touched, so hand edits survive regeneration.

    simulate=True passes through to generate_hal() -- everything else
    (joints, units, homing mode, the INI) is identical to a real run; only
    the hardware layer is swapped for a software loopback. See
    generate_hal()'s docstring.
    """

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    hal_path = output_dir / f"{machine.machine_name}.hal"
    ini_path = output_dir / f"{machine.machine_name}.ini"

    hal_path.write_text(generate_hal(machine, simulate=simulate))
    ini_path.write_text(generate_ini(machine))

    tool_table_path = output_dir / _TOOL_TABLE_FILENAME
    if not tool_table_path.exists():
        tool_table_path.write_text(_default_tool_table_text())

    return hal_path, ini_path
