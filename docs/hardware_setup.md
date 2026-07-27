# Hardware Setup: Raspberry Pi + Mesa 7I76E/7I76EU + LinuxCNC

This is the wiring and configuration guide for this project's target
hardware. It covers the physical board, where to wire motors and limit
switches, and what every field in `configs/axis.jsonl` /
`configs/machine.json` means. For how to set up the Raspberry Pi itself
(installing LinuxCNC, getting this repo onto it, running the control
software), see [software_setup.md](software_setup.md). For open concerns
that still need checking on real hardware, see
[potential_issues.md](potential_issues.md).

```
configs/machine.json + configs/axis.jsonl
           |
           v
lcaf.utils.joint_configuration  (generate_hal / generate_ini)
           |
           v
configs/generated/<machine_name>.hal + .ini   <-- what LinuxCNC loads
           |
           v
linuxcnc (the actual realtime process, started separately)
           |
           v
lcaf.control.linuxcnc_interface (LinuxCNCMachineInterface / LinuxCNCAxialInterface)
```

You should never need to hand-write a `.hal`/`.ini` file, or touch
`linuxcnc_interface.py`/`axis.py`/`motion_coordinator.py`, to add a motor or
change a pin. Edit `configs/axis.jsonl` (per-joint hardware) and
`configs/machine.json` (machine-wide settings); everything below them
regenerates itself.

## 1. What is a "joint," and where does it plug into the Mesa card?

"Joint" is LinuxCNC's term for one stepper-driven degree of freedom. This
machine has four, bound one-to-one to X/Y/Z/A (`trivkins` kinematics -- no
shared motion between them). Each joint owns exactly one Mesa stepgen (the
FPGA circuit that turns step pulses into an actual STEP/DIR signal) and,
optionally, one enable output and a pair of limit-switch inputs:

| Joint | Axis | Motor wiring (TB2/TB3) | Enable output (TB6) | Limit switches (TB6) |
|---|---|---|---|---|
| 0 | X | TB2 pins 2-5 (STEP0, DIR0) | OUTPUT0 (pin 17) | INPUT0 neg (pin 1), INPUT1 pos (pin 2) |
| 1 | Y | TB2 pins 8-11 (STEP1, DIR1) | OUTPUT1 (pin 18) | INPUT2 neg (pin 3), INPUT3 pos (pin 4) |
| 2 | Z | TB2 pins 14-17 (STEP2, DIR2) | OUTPUT2 (pin 19) | INPUT4 neg (pin 5), INPUT5 pos (pin 6) |
| 3 | A | TB3 pins 2-5 (STEP4, DIR4) | OUTPUT3 (pin 20) | none -- A is continuous, see section 5 |

Every column above is just a Mesa HAL pin name set in `axis.jsonl`
(`mesa_stepgen`, `enable_output`, `negative_limit_input`,
`positive_limit_input`) -- `generate_hal()` wires exactly this table into
the `.hal` file for you. You don't need to know HAL syntax to add a motor;
you need to know which physical terminal it's plugged into, which is this
table.

## 2. The board

Target board: **Mesa Electronics 7I76E** (priced/wired against the 7I76EU
variant; the two are pin- and HAL-compatible). It connects to the Pi over
Ethernet, not through a carrier card.

This project uses 4 of its 5 step/dir channels (X/Y/Z/A), 4 of its 16
outputs (per-axis enable), and 6 of its 32 inputs (X/Y/Z limit switches --
A has none, see section 5). Everything else on the card (inputs 6-31,
outputs 4-15, the one encoder input, the spindle interface, the 5th
stepgen) is unused and free for future expansion.

## 3. Jumpers

Leave every jumper at its factory default except these:

| Jumper | Position | Why |
|---|---|---|
| W1 (VIN source) | LEFT (default) | Powers VIN from field power -- you only need to feed 24V into TB1. |
| W2, W3 (IP address) | DOWN, DOWN (default) | Fixed IP `192.168.1.121`, matching `mesa_board_config` in `machine.json`. Change both together if you ever need a different IP. |
| W8 (setup/operate) | LEFT (operate mode) | Must be LEFT for normal running. Only move to RIGHT for Mesa's flashing/inspection tools. |

## 4. Network

Give the Raspberry Pi's NIC a static IP on the same /24 as the card, e.g.
`192.168.1.10`, and connect them directly with an Ethernet cable. Before
touching LinuxCNC, confirm the card answers:

```
ping 192.168.1.121
```

## 5. Wiring

All field wiring uses the card's pluggable 3.5mm screw terminals. Voltages
below assume a 24VDC field supply (`hm2` field power accepts 5-32VDC).

### Power (TB1)

Pins 1-4 = VFIELD (24V in), pin 8 = GROUND. **VFIELD must connect directly
to the DC source with no switch, breaker, or relay in the circuit** (a fuse
is fine) -- never switch field power through a mechanical contact.

### Motors (TB2 for X/Y/Z, TB3 for A)

Use the table in section 1 to find each joint's STEP/DIR pin pair. Each
pair is a differential (+/-) signal: wire both legs to your driver's
STEP+/STEP-/DIR+/DIR- inputs if it has them; otherwise wire the `+` leg and
return the driver's common to the 7I76E's GND or +5VP pin on the same
connector, per your driver's single-ended wiring diagram.

### Limit switches (TB6)

Where to put them: **one at each physical end of travel on X, Y, and Z**.
The switch at the end where you want machine coordinate 0 to be is the
*negative* limit; the one at the far end of travel is the *positive*
limit. Wire them **normally closed** -- one leg to field power (+24V), the
other to the input pin -- so a broken wire or short reads as a fault
(switch goes inactive) instead of silently looking like "not at the
limit." A has no limit switches; see section 6 for why.

Use the table in section 1 for which TB6 pin is which joint's negative/
positive input. If your switches are wired the opposite way (normally
open, or you can't rewire them), set `invert_negative_limit` /
`invert_positive_limit` for that joint in `axis.jsonl` instead of
rewiring. The two switches on a joint must be different physical inputs --
`JointConfiguration` refuses to load an `axis.jsonl` that wires
`negative_limit_input` and `positive_limit_input` to the same pin, since
there would be no way to tell which end of travel was actually reached.

### What a tripped limit switch actually does

Each switch is wired straight into LinuxCNC's own hard-limit input for that
joint (`joint.N.neg-lim-sw-in` / `joint.N.pos-lim-sw-in` -- section 1's
table, `JointConfiguration.negative_limit_hal_pin`/`positive_limit_hal_pin`).
**This is not a per-direction soft stop.** LinuxCNC treats either switch
tripping as "this should never happen in normal operation" and reacts by
disabling every joint's enable output machine-wide (reporting `"joint N on
limit switch error"`), not just inhibiting further travel on the one
joint/direction that hit it. Recovering requires
`linuxcnc.command().override_limits()` (or the Axis GUI's "Override
Limits" checkbox) plus turning the machine back on and jogging off the
switch -- and this project should be re-homed after any such event anyway,
per the open-loop-steppers point in `docs/potential_issues.md`.

This condition does **not** set `status.joint[n]['fault']` -- LinuxCNC's
Python interface documents that field as "axis amp fault" only (following
error here, see `LinuxCNCAxialInterface.is_faulted()`), so a hard-limit
trip is otherwise invisible to this project. `Axis.poll()` checks
`LinuxCNCAxialInterface.is_on_hard_limit()`
(`status.joint[n]['min_hard_limit']`/`['max_hard_limit']`) separately for
exactly this reason, and moves the axis to `FAULT` -- except while that
axis's own state is `HOMING`, since driving onto the switch it's searching
for is expected there, not a fault (see section 7).

If what you actually want is "block further travel past the negative
switch but still allow positive travel, and vice versa," that already
happens for ordinary point-to-point moves through each joint's *soft*
limits (`[JOINT_n] MIN_LIMIT`/`MAX_LIMIT` in the generated INI, derived
from `max_travel`) plus `Axis.move()`'s own `is_position_in_range()` check
before it ever issues a move. The hard switches are a last-resort backstop
behind that, not the primary mechanism -- deliberately, because these are
open-loop steppers with no position feedback (`docs/potential_issues.md`),
so a switch tripping outside homing likely means steps were already missed
and something is already wrong, which is exactly when a full machine-wide
stop is the safer reaction.

Homing is the one deliberate exception to all of this -- section 7 covers
how it avoids tripping this same fault while intentionally driving into
the switch it's searching for.

### Enable outputs (TB6)

Wire each joint's `OUTPUTn` pin (section 1) to your driver's enable input.
If the driver enables/disables backwards from what LinuxCNC expects, fix it
by wiring to the driver's *other* enable terminal or its own
enable-polarity DIP switch -- the 7I76E's outputs have no software invert.

## 6. The A axis has no limit switches, and that's fine

A is a continuous rotary joint: "home" is simply wherever it's sitting when
it powers on (no seeking, no switches -- `"has_limit_switches": false` for
joint 3 in `axis.jsonl`), and it is free to rotate either direction from
there. This project's A joint has no cable-wrap constraint (nothing on
slip rings needs it), so unlike X/Y/Z its travel is symmetric:
`[-max_travel, max_travel]` in `axis.jsonl`, not `[0, max_travel]`.

## 7. Two homing strategies: `use_linuxcnc_native_processes`

`configs/machine.json` has one machine-wide switch,
`use_linuxcnc_native_processes`:

- **`false` (default).** This project's own Python code homes each joint by
  jogging it to its limit switches directly (or zeroing immediately for the
  switchless A joint). LinuxCNC's own native homing sequence never runs.
  Because LinuxCNC would otherwise never see any joint as "homed" and would
  refuse every subsequent move, `generate_ini()` sets
  `[TRAJ]NO_FORCE_HOMING = 1` automatically in this mode. The switch it
  jogs into is also wired into LinuxCNC's own hard-limit fault pin (see
  section 5), and LinuxCNC only exempts *its own* native homing state
  machine from that fault -- a plain jog never qualifies. So before each
  deliberate seek-jog, `LinuxCNCAxialInterface._jog_toward_limit_switch()`
  calls `command.override_limits()` first (the same mechanism behind the
  Axis GUI's "Override Limits" checkbox), and `poll_homing()` calls it
  again on every heartbeat for as long as that seek is still in progress
  (LinuxCNC clears an override once the joint next reports "in position,"
  and there's no clean signal from here for exactly when a jog leaves that
  state, so this re-arms it defensively rather than risk it lapsing right
  before the switch trips); without it, the instant the switch trips,
  LinuxCNC would disable the whole machine before this project's own
  polling loop ever got a chance to react and stop the jog. See
  `debug/tests/test_linuxcnc_interface.py` for this behavior under a stubbed
  `linuxcnc`/`hal` (the real modules only exist on the Pi target -- section
  12 below).
- **`true`.** LinuxCNC's own native homing sequence runs instead
  (`command.home(joint)`), using the same limit switches -- `generate_hal()`
  wires each switched joint's negative-limit input to LinuxCNC's
  `home-sw-in` pin too, and `generate_ini()` fills in real
  `HOME_SEARCH_VEL`/`HOME_LATCH_VEL`/`HOME_SEQUENCE` values, including
  `HOME_IGNORE_LIMITS = YES`. Nothing else in `axis.jsonl` needs to change
  to switch modes. This mode never needs `override_limits()` -- LinuxCNC's
  own homing state machine is already exempt from the hard-limit fault
  while it's running.

Either way, machine coordinate 0 on X/Y/Z is wherever the negative limit
switch is, and the machine always requires a fresh `home_all()` every
process start -- see section 9 and `docs/potential_issues.md`.

## 8. Motor configuration fields (`axis.jsonl`)

One JSON object per line, one line per joint. To add or reconfigure a
motor, set these:

| Field | Meaning |
|---|---|
| `joint` | LinuxCNC joint number (0-3). |
| `axis` | Cartesian letter (X/Y/Z/A). |
| `motor_steps_per_revolution` | Full steps per motor shaft revolution (e.g. 200 for a 1.8°/step motor). |
| `microsteps` | Microstep multiplier set on the external driver (e.g. 8). Together with the field above, this is "steps per revolution." |
| `travel_per_motor_rev` | Distance moved per full motor revolution: a leadscrew's pitch in in/rev for X/Y/Z, or an output-degrees-per-turn ratio in deg/rev for A. |
| `max_velocity`, `max_acceleration` | Joint's top speed/acceleration: in/s and in/s² for X/Y/Z, deg/s and deg/s² for A. |
| `mesa_stepgen` | Which stepgen drives this joint -- see section 1's table. |
| `enable_output` | Field output wired to the driver's enable input -- see section 1's table. |
| `negative_limit_input` / `positive_limit_input` | Field inputs wired to the limit switches -- see section 1's table. `null` for A. Must differ from each other when both are set -- rejected at load time otherwise (section 5). |
| `is_angular` | `true` for A only. |
| `has_limit_switches` | `false` for A only -- see section 6. |
| `inverted` | `true` if the joint moves the wrong physical direction. Flips the motor's direction in software; the normal fix for a reversed motor/lead, cheaper than re-wiring. |
| `invert_negative_limit` / `invert_positive_limit` | `true` if that limit switch reads backwards from the normally-closed wiring in section 5. |
| `home_sequence` | Only used when `use_linuxcnc_native_processes` is `true` (section 7). Leave at `-1` to home this joint on its own in "Home All"; LinuxCNC then defaults it to the joint number. |
| `max_travel` | Total usable travel. For X/Y/Z: the one-directional distance from the negative limit switch, in inches. For A: the symmetric +/- bound in degrees (section 6) -- since A has no physical limit, set this generously (the default is 100000°, i.e. effectively unbounded for any real job). |
| `step_length_ns`, `step_space_ns`, `direction_setup_ns`, `direction_hold_ns` | Stepper driver signal timing in nanoseconds. Defaults are usually fine; check your driver's datasheet if steps are missed or the motor is silent. |

## 9. Machine-wide fields (`machine.json`)

| Field | Meaning |
|---|---|
| `machine_name` | Base name for the generated `.hal`/`.ini` and the INI's `[EMC]MACHINE`. |
| `axis_config_path` | Path to the joint JSONL file (default `axis.jsonl`). |
| `linear_units`, `angular_units` | Leave as `"inch"`/`"degree"` -- see section 10. |
| `default_linear_velocity_in_s`, `max_linear_velocity_in_s`, `max_linear_acceleration_in_s2` | Machine-wide linear jog/program defaults and cap, in/s and in/s². |
| `default_angular_velocity_deg_s`, `max_angular_velocity_deg_s`, `max_angular_acceleration_deg_s2` | Same, for A, in deg/s and deg/s². |
| `use_linuxcnc_native_processes` | See section 7. |
| `linear_ferror_in`, `linear_min_ferror_in`, `angular_ferror_deg`, `angular_min_ferror_deg` | Following-error fault tolerances (how far behind commanded position LinuxCNC lets a joint fall before faulting). This is the only stall/skipped-step detection this machine has -- see `docs/potential_issues.md` -- so tune these against real measured following error during commissioning, not the shipped defaults. |
| `base_period_ns`, `servo_period_ns` | LinuxCNC's realtime thread periods. |
| `mesa_board_driver`, `mesa_board_config` | The `loadrt` driver name and arguments for the Mesa card. |
| `watchdog_timeout_ns` | HAL watchdog timeout before LinuxCNC considers the realtime link dead. |

## 10. Units and travel range

**This machine is configured entirely in inches** for X/Y/Z
(`"linear_units": "inch"`) and degrees for A (always, independent of
`linear_units`). If converting a metric-datasheet value (a leadscrew pitch
in mm, a measured travel in mm) into `axis.jsonl`, divide by 25.4.

**One deliberate exception:** the toolpath planner (`lcaf.toolpathing`, see
[toolpath_slicer.md](toolpath_slicer.md)) works entirely in millimetres --
its own, separate coordinate space. `LinuxCNCAxialInterface.move()` always
sends an explicit `G21` so LinuxCNC interprets those millimetre values
correctly regardless of the inch-native machine config above. You never
need to convert a toolpath file to inches.

**Every axis is zero-based, matching where it physically starts:**

- **X/Y/Z**: range is `[0, max_travel]`. 0 is wherever homing finds the
  negative limit switch (measured live in software-homing mode; wherever
  native homing lands in native mode -- section 7). The toolpath planner's
  own X=0 is the clamp, the same physical point (see
  [toolpath_slicer.md](toolpath_slicer.md)), so a normal toolpath needs no
  manual coordinate-shifting to fit this range.
- **A**: range is `[-max_travel, max_travel]`. 0 is wherever it was sitting
  at power-on (section 6); it is free to move either direction from there.

## 11. Setting everything up

1. Wire the board per section 5, and check jumpers against section 3.
2. Give the Pi a static IP on the 7I76E's subnet (section 4) and confirm
   `ping 192.168.1.121` works with the card powered.
3. Edit `configs/axis.jsonl` and `configs/machine.json` for your hardware
   (sections 7-9). Leave `inverted`/`invert_negative_limit`/
   `invert_positive_limit` `false` on a first pass -- you'll only know
   which ones you need once you see real motion.
4. Regenerate the `.hal`/`.ini` by hand once, to review the output before
   trusting it live:
   ```
   python -c "from lcaf.utils.joint_configuration import load_machine_configuration, write_config_files; write_config_files(load_machine_configuration('configs/machine.json'), 'configs/generated')"
   ```
   This also creates an empty `configs/generated/tool.tbl` the first time
   (LinuxCNC's `iocontrol` requires this file to exist even though this
   project has no tool changer -- see `software_setup.md` section 5). It is
   never overwritten on later regenerations.
5. Start LinuxCNC against the generated INI:
   `linuxcnc configs/generated/LCAF_Forge.ini`.
6. In a second terminal, confirm the board came up and pins exist:
   `halcmd show pin hm2_7i76e`. Check the `config=` stepgen/encoder counts
   in `machine.json` actually match what the card's firmware reports here.
7. With the machine ON but *before* running any program, jog each joint by
   hand in small steps and confirm: it moves the physical direction you
   expect, the correct limit switch triggers at each end of travel, and the
   enable output actually enables/disables the driver. Fix any mismatch
   with the matching field from section 8, regenerate, and restart
   LinuxCNC -- one flag at a time.
8. Only once every joint jogs and homes correctly by hand, start the LCAF
   control process (`python -m lcaf.control.main`), which connects to the
   already-running LinuxCNC over NML and takes over homing/motion via
   `MotionCoordinator`. It will refuse to move any axis until `home_all()`
   completes in that process -- see `docs/potential_issues.md`.

Regenerating `configs/generated/*` while LinuxCNC is already running does
not affect that running instance -- restart LinuxCNC (step 5) to pick up a
config change.

## 12. Code walkthrough: config to state machine

- **`joint_configuration.py`** is pure data + text generation: no import of
  `linuxcnc`/`hal`, so it can run anywhere. `JointConfiguration` is one
  joint's hardware description; `MachineConfiguration` bundles all of them
  plus machine-wide settings; `generate_hal`/`generate_ini` render the
  files LinuxCNC actually loads.

- **`linuxcnc_interface.py`** is the only place that imports `linuxcnc`/
  `hal`. `LinuxCNCMachineInterface` owns the single shared
  `linuxcnc.command()`/`stat()`/`error_channel()` connection -- LinuxCNC
  exposes one NML channel for the whole machine, not one per joint.
  `LinuxCNCAxialInterface` is the per-joint half: it knows how to
  move/jog/home/read one joint, using either native or software homing
  depending on `use_linuxcnc_native_processes` (section 7).

- **`axis.py`**'s `Axis` wraps one `LinuxCNCAxialInterface` with a small
  state machine of its own (`UNINITIALIZED -> READY -> MOVING/HOMING`, plus
  `FAULT`/`ESTOP`) purely for axis-level bookkeeping and logging -- it does
  not duplicate LinuxCNC's own motion state.

- **`motion_coordinator.py`**'s `MotionCoordinator` loads
  `MachineConfiguration` (default: `configs/machine.json` +
  `configs/axis.jsonl`), regenerates the `.hal`/`.ini`, builds one `Axis`
  per configured joint, and sequences every toolpath operation through the
  fixed safe motion order (retract Z/Y/X, rotate A, move X/Y/Z). This part
  is not to be modified per the same constraint that protects
  `forge_brain.py`'s HSFM.

- **`forge_brain.py`** (untouched) only ever talks to `MotionCoordinator`
  through `self.motion.interface` (machine-wide queries/commands) and
  `self.motion` itself (`start`, `update`, `home_all`, `emergency_stop`,
  `is_complete`, `has_fault`, `reset`). Nothing above this line needs to
  change for `forge_brain.py` to keep working against different hardware --
  that's the whole point of the config files.

See [potential_issues.md](potential_issues.md) for open concerns with this
pipeline that aren't config problems -- things worth checking before you
trust it against real hardware.
