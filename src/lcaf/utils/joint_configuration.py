from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path


@dataclass(slots=True)
class JointConfiguration:
    """
    Describes one physical CNC joint driven by a single stepper motor.
    A joint contains a motor and two limit switches.

    This object contains the hardware description of one axis. It is used by
    the LinuxCNC HAL/INI generator and by LinuxCNCAxialInterface.

    Target platform:
        Raspberry Pi host running LinuxCNC
        Mesa 7I76E / 7I76EU Ethernet step/dir + I/O card (driver hm2_eth)

    All pin names refer to Mesa HAL pin names rather than Raspberry Pi GPIOs.
    See docs/hardware_setup.md for the 7I76E terminal-block wiring these
    pin names correspond to.
    """

    # Identity
    joint: int
    axis: str

    # Stepper Motor
    motor_steps_per_revolution: int
    microsteps: int
    leadscrew_pitch_mm: float
    """
    Linear joints: leadscrew travel (mm) per motor revolution.
    Angular joints (is_angular=True): degrees of travel per motor revolution.
    """

    max_velocity_mm_s: float
    max_acceleration_mm_s2: float
    """
    Angular joints: interpreted as deg/s and deg/s^2 respectively.
    """

    # Mesa HAL Pins
    mesa_stepgen: str
    """
    Example:
        hm2_7i96s.0.stepgen.00
    """

    enable_output: str | None = None
    negative_limit_input: str | None = None
    """Physical Mesa field-input pin wired to the negative limit switch, e.g. hm2_7i76e.0.7i76.0.0.input-00"""
    positive_limit_input: str | None = None
    """Physical Mesa field-input pin wired to the positive limit switch, e.g. hm2_7i76e.0.7i76.0.0.input-01"""

    # Joint kinematics
    is_angular: bool = False

    has_limit_switches: bool = True
    """
    False for a joint with no physical limit switches (e.g. a continuously
    rotating axis): homing zeroes the joint at its current position instead
    of jogging to find switches, and soft_min_mm/soft_max_mm are used as-is
    rather than being measured. See LinuxCNCAxialInterface.home_axis.
    """

    # Signal polarity
    invert_step: bool = False
    invert_direction: bool = False
    invert_enable: bool = False

    invert_negative_limit: bool = False
    invert_positive_limit: bool = False
    invert_home_switch: bool = False

    # Software travel limits
    soft_min_mm: float | None = None
    soft_max_mm: float | None = None

    # Homing (LinuxCNC-native homing is disabled by default; this project
    # homes axes in software by jogging to the limit switches, see
    # LinuxCNCAxialInterface.home_axis)
    home_offset_mm: float = 0.0
    home_sequence: int = -1

    # Step timing (nanoseconds)
    step_length_ns: int = 5000
    step_space_ns: int = 5000
    direction_setup_ns: int = 20000
    direction_hold_ns: int = 20000

    # Derived values
    steps_per_mm: float = field(init=False)

    def __post_init__(self):

        self.axis = self.axis.upper()

        valid_axes = {"X", "Y", "Z", "A", "B", "C", "U", "V", "W"}

        if self.axis not in valid_axes:
            raise ValueError(f"Unsupported axis '{self.axis}'.")

        if self.motor_steps_per_revolution <= 0:
            raise ValueError("motor_steps_per_revolution must be positive.")

        if self.microsteps <= 0:
            raise ValueError("microsteps must be positive.")

        if self.leadscrew_pitch_mm <= 0:
            raise ValueError("leadscrew_pitch_mm must be positive.")

        if self.max_velocity_mm_s <= 0:
            raise ValueError("max_velocity_mm_s must be positive.")

        if self.max_acceleration_mm_s2 <= 0:
            raise ValueError("max_acceleration_mm_s2 must be positive.")

        if not self.has_limit_switches and (self.soft_min_mm is None or self.soft_max_mm is None):
            raise ValueError(
                f"Joint {self.joint} ({self.axis}) has has_limit_switches=False, so its "
                "travel range cannot be measured; soft_min_mm/soft_max_mm are required."
            )

        self.steps_per_mm = (
            self.motor_steps_per_revolution
            * self.microsteps
            / self.leadscrew_pitch_mm
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
    """

    machine_name: str

    linear_units: str = "mm"
    angular_units: str = "degree"

    default_linear_velocity_mm_s: float = 10.0
    max_linear_velocity_mm_s: float = 25.0
    max_linear_acceleration_mm_s2: float = 400.0

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
    Load one JointConfiguration per non-empty line of a JSONL file.
    """

    path = Path(path)

    if not path.exists():
        raise FileNotFoundError(path)

    joints: list[JointConfiguration] = []

    with open(path, "r") as file:
        for line_number, line in enumerate(file, start=1):
            line = line.strip()

            if not line or line.startswith("#"):
                continue

            try:
                data = json.loads(line)
                joints.append(JointConfiguration.from_dict(data))
            except (json.JSONDecodeError, TypeError, ValueError) as e:
                raise ValueError(f"{path}:{line_number}: invalid joint configuration: {e}") from e

    if not joints:
        raise ValueError(f"{path}: no joint configurations found.")

    return joints


def load_machine_configuration(
    machine_path: str | Path,
    joints_path: str | Path | None = None,
) -> MachineConfiguration:
    """
    Load the machine-wide settings from `machine_path` (JSON) and the joint
    list from `joints_path` (JSONL). If `joints_path` is not given, it is
    resolved relative to `machine_path`'s directory using the
    'axis_config_path' key in the machine JSON (default 'axis.jsonl').
    """

    machine_path = Path(machine_path)

    if not machine_path.exists():
        raise FileNotFoundError(machine_path)

    with open(machine_path, "r") as file:
        data = json.load(file)

    resolved_joints_path = (
        joints_path
        if joints_path is not None
        else machine_path.parent / data.get("axis_config_path", "axis.jsonl")
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


def generate_hal(machine: MachineConfiguration) -> str:
    """
    Render the .hal file that wires the Mesa stepgens/GPIO to LinuxCNC's
    motion joints for every configured joint.
    """

    lines: list[str] = []

    def emit(text: str = ""):
        lines.append(text)

    emit(f"# Generated by lcaf.utils.joint_configuration for machine '{machine.machine_name}'.")
    emit("# Do not edit by hand -- regenerate from the machine/axis config files.")
    emit()
    emit("loadrt trivkins")
    emit(f"loadrt motmod base_period_nsec={machine.base_period_ns} servo_period_nsec={machine.servo_period_ns}")

    board_line = f"loadrt {machine.mesa_board_driver}"
    if machine.mesa_board_config:
        board_line += f" {machine.mesa_board_config}"
    emit(board_line)

    emit()
    emit("addf hm2_read-request servo-thread")
    emit("addf motion-command-handler servo-thread")
    emit("addf motion-controller servo-thread")

    for joint in sorted(machine.joints, key=lambda j: j.joint):
        emit(f"addf {joint.mesa_stepgen}.capture-position servo-thread")

    emit("addf hm2_write servo-thread")
    emit()
    emit("net watchdog-reset <= motion.motion-enabled")
    emit()

    for joint in sorted(machine.joints, key=lambda j: j.joint):
        emit(f"# --- Joint {joint.joint} ({joint.axis}) ---")

        scale = joint.steps_per_mm
        if joint.invert_direction:
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

        if joint.enable_output and joint.invert_enable:
            emit(
                f"# NOTE: invert_enable is set for joint {joint.joint} ({joint.axis}), but the "
                "7I76E's field outputs have no HAL-level output invert. Swap to the driver's "
                "opposite enable terminal (or its own DIP switch) instead -- see hardware_setup.md."
            )

        if joint.negative_limit_input:
            neg_pin = joint.negative_limit_input
            if joint.invert_negative_limit:
                neg_pin = _inverted_input_pin(neg_pin)
            emit(f"net {joint.axis.lower()}-neg-lim {neg_pin} => {joint.negative_limit_hal_pin}")
        elif not joint.has_limit_switches:
            emit(f"# Joint {joint.joint} ({joint.axis}) has no limit switches; homed by "
                 "zeroing at boot (has_limit_switches=False in axis.jsonl).")

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
        ("DEFAULT_LINEAR_VELOCITY", _format_ini_value(machine.default_linear_velocity_mm_s / 60.0)),
        ("MAX_LINEAR_VELOCITY", _format_ini_value(machine.max_linear_velocity_mm_s / 60.0)),
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
        ("DEFAULT_LINEAR_VELOCITY", _format_ini_value(machine.default_linear_velocity_mm_s)),
        ("MAX_LINEAR_VELOCITY", _format_ini_value(machine.max_linear_velocity_mm_s)),
        ("DEFAULT_ANGULAR_VELOCITY", _format_ini_value(machine.default_angular_velocity_deg_s)),
        ("MAX_ANGULAR_VELOCITY", _format_ini_value(machine.max_angular_velocity_deg_s)),
        ("POSITION_FILE", "position.txt")
    ])

    section("EMCIO", [
        ("EMCIO", "io"),
        ("CYCLE_TIME", "0.100"),
        ("TOOL_TABLE", "tool.tbl"),
    ])

    for joint in joints:
        if joint.soft_min_mm is None or joint.soft_max_mm is None:
            raise ValueError(
                f"Joint {joint.joint} ({joint.axis}) is missing soft_min_mm/soft_max_mm; "
                "required to generate INI travel limits."
            )

        vel = joint.max_velocity_mm_s
        accel = joint.max_acceleration_mm_s2

        section(joint.ini_joint_section, [
            ("TYPE", "ANGULAR" if joint.is_angular else "LINEAR"),
            ("HOME", "0.0"),
            ("HOME_OFFSET", _format_ini_value(joint.home_offset_mm)),
            ("HOME_SEQUENCE", str(joint.home_sequence)),
            ("HOME_USE_INDEX", "NO"),
            ("HOME_IGNORE_LIMITS", "NO"),
            ("MAX_VELOCITY", _format_ini_value(vel)),
            ("MAX_ACCELERATION", _format_ini_value(accel)),
            ("STEPGEN_MAXVEL", _format_ini_value(vel * 1.2)),
            ("STEPGEN_MAXACCEL", _format_ini_value(accel * 1.2)),
            ("MIN_LIMIT", _format_ini_value(joint.soft_min_mm)),
            ("MAX_LIMIT", _format_ini_value(joint.soft_max_mm)),
            ("FERROR", "1.0"),
            ("MIN_FERROR", "0.25"),
        ])

        section(joint.ini_axis_section, [
            ("MIN_LIMIT", _format_ini_value(joint.soft_min_mm)),
            ("MAX_LIMIT", _format_ini_value(joint.soft_max_mm)),
            ("MAX_VELOCITY", _format_ini_value(vel)),
            ("MAX_ACCELERATION", _format_ini_value(accel)),
        ])

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


def write_config_files(
    machine: MachineConfiguration,
    output_dir: str | Path,
) -> tuple[Path, Path]:
    """
    Render and write '<machine_name>.hal' and '<machine_name>.ini' into
    output_dir, creating it if needed. Returns (hal_path, ini_path).
    """

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    hal_path = output_dir / f"{machine.machine_name}.hal"
    ini_path = output_dir / f"{machine.machine_name}.ini"

    hal_path.write_text(generate_hal(machine))
    ini_path.write_text(generate_ini(machine))

    return hal_path, ini_path
