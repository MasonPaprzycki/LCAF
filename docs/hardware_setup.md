# Hardware Setup: Raspberry Pi + Mesa 7I76E/7I76EU + LinuxCNC

This is the wiring and configuration guide for this project's target
hardware. It covers the physical board, where to wire motors and limit
switches, and what every field in `configs/axis.json` /
`configs/machine.json` means. For how to set up the Raspberry Pi itself
(installing LinuxCNC, getting this repo onto it, running the control
software), see [software_setup.md](software_setup.md). For open concerns
that still need checking on real hardware, see
[potential_issues.md](potential_issues.md).

```
configs/machine.json + configs/axis.json
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
change a pin. Edit `configs/axis.json` (per-joint hardware) and
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
| 0 | X   | TB2 pins 2-5 (STEP0, DIR0) | OUTPUT0 (pin 17) | INPUT0 neg (pin 1), INPUT1 pos (pin 2) |
| 1 | Y | TB2 pins 8-11 (STEP1, DIR1) | OUTPUT1 (pin 18) | INPUT2 neg (pin 3), INPUT3 pos (pin 4) |
| 2 | Z | TB2 pins 14-17 (STEP2, DIR2) | OUTPUT2 (pin 19) | INPUT4 neg (pin 5), INPUT5 pos (pin 6) |
| 3 | A | TB3 pins 2-5 (STEP4, DIR4) | OUTPUT3 (pin 20) | none -- A is continuous, see section 5 |

Every column above is just a Mesa HAL pin name set in `axis.json`
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
positive input. If your switches are normally closed, set `invert_negative_limit` /
`invert_positive_limit` as true for that joint in `axis.json`. If they are normally open set as false. The two switches on a joint must be different physical inputs --
`JointConfiguration` refuses to load an `axis.json` that wires
`negative_limit_input` and `positive_limit_input` to the same pin, since
there would be no way to tell which end of travel was actually reached.

**One switch or two?** `JointConfiguration.dual_limit_switches` supports
both, but this project's own X/Y/Z are all wired **single-switch**: one
negative (zero-end) limit switch each, with `positive_limit_input: null`
and `dual_limit_switches: false` in `axis.json`. Homing seeks the one
negative switch to establish zero, then stops there and trusts the
configured `extended_distance` as a static limit (LinuxCNC's own native
homing -- the only homing this project uses -- never remeasures travel from
a positive switch regardless of `dual_limit_switches`; see section 7). If a
joint ever does get a second, positive-end switch wired, set
`dual_limit_switches: true` and `positive_limit_input` to that pin instead
so `generate_hal(simulate=True)` models it and the field/pin validation
matches reality -- homing behavior itself doesn't change. See
`JointConfiguration.dual_limit_switches` and section 7 below.

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
from `retracted_distance`/`extended_distance`) plus `Axis.move()`'s own
`is_position_in_range()` check
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
joint 3 in `axis.json`), and it is free to rotate either direction from
there. This project's A joint has no cable-wrap constraint (nothing on
slip rings needs it), so unlike X/Y/Z its `retracted_distance` and
`extended_distance` are both `null` in `axis.json`, genuinely disabling
its software soft limits in both directions rather than approximating
"unbounded" with an arbitrary large sentinel value. `generate_ini()` omits
`MIN_LIMIT`/`MAX_LIMIT` for A entirely, which is itself LinuxCNC's own
documented way of saying "no limit" (it substitutes `-1e99`/`1e99`), and
`JointConfiguration.__post_init__` logs a warning at load time as a
reminder that this disables a safety check -- for A that's an accepted
tradeoff, since it has no physical limit to check against anyway.

## 7. Homing: always LinuxCNC's own native homing sequence

This project always homes through LinuxCNC's own native homing sequence --
specifically its own "Home All" (`command.home(-1)`, issued once,
machine-wide, by `LinuxCNCMachineInterface.home_all_command()`), the exact
same command the Axis GUI's own "Home All" button sends, then waiting for
each joint's own `status.joint[n]['homed']`. `generate_hal()` wires each
switched joint's negative-limit input to LinuxCNC's `home-sw-in` pin too,
and `generate_ini()` fills in real `HOME_SEARCH_VEL`/`HOME_LATCH_VEL`/
`HOME_SEQUENCE` values, including `HOME_IGNORE_LIMITS = YES` for every
joint (so a switched joint's own home-seek can drive into the same switch
its hard limit also watches without LinuxCNC's homing state machine
faulting on it).

This project previously issued a separate `command.home(joint)` for each
joint individually instead of the `-1` form -- removed, since it was an
unnecessary reimplementation of exactly what "Home All" already does
natively, and real-hardware testing found it didn't behave identically to
the GUI's own "Home All" button (which operators had already confirmed
works correctly, including the `home_sequence`-grouped X+Y-then-Z ordering
in this project's own `axis.json`) -- the individually-numbered form went
out cleanly over the shared NML command channel but never produced any
physical motion. `command.home(-1)` sidesteps that gap entirely by using
the identical mechanism as the button.

This project previously also supported driving homing itself in Python
(jogging each joint to its limit switches directly, with
`command.override_limits()` suppressing the resulting hard-limit fault,
toggled by a `use_linuxcnc_native_processes` machine.json switch) --
removed, because it turned out to be fundamentally racy: LinuxCNC's own
realtime motion loop re-checks each joint's hard-limit HAL pin every servo
cycle (~1ms) completely independently of this Python process, and disables
every joint's amp-enable-out machine-wide (with no automatic recovery --
see `docs/potential_issues.md`) the instant it sees the pin active without
that joint already in LinuxCNC's own override snapshot. A Python client
calling `override_limits()` reactively, once per its own control-loop
heartbeat (~20-50ms), cannot reliably win that race against LinuxCNC's own
1ms realtime check. Native homing's `HOME_IGNORE_LIMITS` avoids the problem
entirely, since it's a compiled state-machine-level bypass evaluated on the
same cycle the trip is detected, not a reactive external mechanism.

Machine coordinate 0 on X/Y/Z is wherever the negative limit switch is, and
the machine always requires a fresh `home_all()` every process start -- see
section 9 and `docs/potential_issues.md`.

### Home All is one command; each joint backs off independently afterward

`MotionCoordinator.home_all()` issues exactly one command --
`LinuxCNCMachineInterface.home_all_command()`'s `command.home(-1)` -- and
LinuxCNC's own task-level homing sequencer takes it from there, honoring
each joint's own `[JOINT_n]HOME_SEQUENCE` (this project's X and Y share
sequence `1` and home together; Z is sequence `2` and follows; see
`axis.json`) as one contiguous native sequence, identically to clicking
"Home All" in the Axis GUI. This project's own code never re-implements
that sequencing or ordering -- each `Axis` just watches its own
`status.joint[n]['homed']`
(`LinuxCNCAxialInterface.begin_homing_wait()`/`poll_homing()`), without
issuing any command of its own, until LinuxCNC's own sequencer gets to it.

The backoff to `retracted_distance` is entirely LinuxCNC's own doing, not a
separate move this project commands: `generate_ini()` sets
`HOME_OFFSET = 0.0` (the negative limit switch itself is the datum) but
`HOME = retracted_distance` (not `0.0`) -- LinuxCNC's own native homing
sequence makes a final move to `HOME` as its last step, after the
search/latch phases establish the switch as zero, and
`status.joint[n]['homed']` only becomes true once *that* move completes
(see `homing.c`'s `HOME_FINAL_MOVE_*` states -> `HOME_FINISHED`). So by the
time `LinuxCNCAxialInterface.poll_homing()` ever sees `homed=True`, the
joint has already backed off the switch -- there is nothing further for
this project's own code to command. Joints in different `HOME_SEQUENCE`
groups finish at different times purely because LinuxCNC's own sequencer
moves on to the next group as soon as the current one finishes; this
project's own polling doesn't gate one axis on another either way. A
later retract-to-zero
(`LinuxCNCAxialInterface.start_retract_to_zero()`/
`poll_retract_to_zero()`) is different: it re-seeks one specific joint by
number (`command.home(joint)`, not `-1`), since only that one joint is
retracting at that point in a toolpath operation, not the whole machine.

**This also matters for correctness, not just convenience:** `HOME` must
itself fall within `[MIN_LIMIT, MAX_LIMIT]`. Leaving `HOME` at `0.0` while
`MIN_LIMIT` is `retracted_distance` (a positive floor above `0`) makes the
joint's own designated home position fall outside its own soft limits --
LinuxCNC's native homing sequence then refuses to complete at all (no
motion, `homed` never becomes true, the control process sits in
`INITIALIZING` until its own homing timeout). `generate_ini()` avoids this
by construction (`HOME` and `MIN_LIMIT` are both simply `retracted_distance`
for a switched joint), but if you ever hand-edit a generated INI, keep this
invariant in mind.

`dual_limit_switches` (section 5) doesn't change homing behavior at all --
native homing never remeasures travel regardless of it (see
`generate_ini()`); it only controls whether `generate_hal(simulate=True)`
builds one or two fake limit switches, and cross-checks that
`positive_limit_input` is set/unset consistently with it.

### Retract-to-zero: re-seeking the switch every retract, not just at boot

Every toolpath operation starts with `MotionCoordinator` retracting Z, then
Y, then X (`docs/state_machine.md`). Because these are open-loop stepper
joints with no position feedback (`docs/potential_issues.md`), a plain
"move to the commanded 0.0" retract would trust whatever position this
project's own bookkeeping has accumulated, which can silently drift from
the true mechanical zero if a step was ever missed mid-session. Instead,
`MotionCoordinator.retract_axis()` -> `Axis.retract_to_zero()` ->
`LinuxCNCAxialInterface.start_retract_to_zero()`/`poll_retract_to_zero()`
re-seeks the negative limit switch on **every** retract (not just the
one-time `home_all()` at process start, by re-running LinuxCNC's own native
homing sequence) and re-zeros from that physical reference, the same way
initial homing does -- it just never touches the `max_limit`/travel
established by that initial homing (native homing never remeasures travel
either way -- see section 7 -- so this holds regardless of
`dual_limit_switches`).

**Y is the one exception:** `JointConfiguration.retract_to` is set to `1.5`
for it, because Y's retracted position is mechanically partway into its
positive-direction travel, not zero -- there is no positive limit switch to
re-seek against either (section 5). So for Y, `start_retract_to_zero()`
instead commands a plain MDI move to `retract_to` in the coordinate
frame the last `home_all()` already established, and `poll_retract_to_zero()`
waits for the joint to report in-position rather than for a switch to trip.
This never re-references `position_offset_to_native` the way the
negative-switch retract does, so it does not correct for stepper drift the
same way -- Y still gets a fresh `home_all()` reference every process
start (section 9) like every other joint, it just isn't re-referenced on
every single retract. Every other joint keeps `retract_to: null`
and behaves as described above.

This means the negative switch is now tripped deliberately, routinely,
every single operation cycle -- not just once at boot the way the rest of
this document originally described hard-limit trips ("this should never
happen in normal operation," section 5's "What a tripped limit switch
actually does"). The same `HOME_IGNORE_LIMITS` setting covers it (section
7), so it still never trips LinuxCNC's machine-wide hard-limit fault -- but
it does mean the negative switch on X/Y/Z sees far more physical actuations
over the life of the machine than a one-time homing switch would. Budget
for that in switch selection/maintenance if this matters for your
hardware.

## 8. Motor configuration fields (`axis.json`)

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
| `dual_limit_switches` | `true` if this joint has a switch at *both* ends of travel and homing should measure the real distance between them; `false` (this project's actual X/Y/Z, despite `true` being the class default) if it only has the one negative/zero-end switch, in which case `positive_limit_input` must be `null` and the configured `extended_distance` is trusted as-is rather than measured. Only meaningful when `has_limit_switches` is `true` -- see section 7. |
| `retract_to` | The absolute position (native units, same zero as `retracted_distance`/`extended_distance`) this joint moves to on retract instead of re-seeking the negative limit switch -- `1.5` for Y (see section 7's "Retract-to-zero"). `null` (default) for every other joint. Must fall within `[retracted_distance, extended_distance]` when both are configured. |
| `inverted` | `true` if the joint moves the wrong physical direction. Flips the motor's direction in software; the normal fix for a reversed motor/lead, cheaper than re-wiring. |
| `invert_negative_limit` / `invert_positive_limit` | `true` if that limit switch reads backwards from the normally-closed wiring in section 5. |
| `home_sequence` | See section 7. Leave at `-1` to home this joint on its own in "Home All"; LinuxCNC then defaults it to the joint number. |
| `retracted_distance` | Positive position this joint backs off to immediately after LinuxCNC's native homing finds its negative limit switch (which always sits at 0), in inches (or degrees for A) -- its standoff/parked position, and this end's soft-limit floor (`0.25` for X/Y/Z: enough clearance that normal operation never re-triggers the switch). For A: `null` (section 6) -- A has no physical limit, no reference switch, and so nothing to back off from either; instead of carrying an arbitrary large sentinel value, this end's soft limit is genuinely disabled (a logged warning at load time says so). Becomes both the generated INI's `MIN_LIMIT` *and* `HOME` when set (see section 7 -- `HOME` is what actually makes LinuxCNC's native homing back off to this position); omitted from `MIN_LIMIT` (LinuxCNC then applies its own `-1e99` default) and `HOME` left at `0.0` when `null`. Must be less than `extended_distance` when both are set. |
| `extended_distance` | Positive distance from zero to this joint's extended (positive-direction) soft limit, in inches (or degrees for A). For X/Y/Z: the one-directional distance from the negative limit switch. For A: `null`, same reasoning as `retracted_distance` above. Becomes the generated INI's `MAX_LIMIT` directly when set; omitted (LinuxCNC default `1e99`) when `null`. Setting either to `null` on any joint disables that end's software safety check the same way -- for X/Y/Z it also disables `lcaf.toolpathing.toolpath_slicer`'s matching travel-limit check, so only do this with a real physical limit switch or mechanical stop backing that end up. |
| `step_length_ns`, `step_space_ns`, `direction_setup_ns`, `direction_hold_ns` | Stepper driver signal timing in nanoseconds. Defaults are usually fine; check your driver's datasheet if steps are missed or the motor is silent. |

## 9. Machine-wide fields (`machine.json`)

| Field | Meaning |
|---|---|
| `machine_name` | Base name for the generated `.hal`/`.ini` and the INI's `[EMC]MACHINE`. |
| `axis_config_path` | Path to the joint JSON file (default `axis.json`). |
| `linear_units`, `angular_units` | Leave as `"inch"`/`"degree"` -- see section 10. |
| `default_linear_velocity_in_s`, `max_linear_velocity_in_s`, `max_linear_acceleration_in_s2` | Machine-wide linear jog/program defaults and cap, in/s and in/s². |
| `default_angular_velocity_deg_s`, `max_angular_velocity_deg_s`, `max_angular_acceleration_deg_s2` | Same, for A, in deg/s and deg/s². |
| `linear_ferror_in`, `linear_min_ferror_in`, `angular_ferror_deg`, `angular_min_ferror_deg` | Following-error fault tolerances (how far behind commanded position LinuxCNC lets a joint fall before faulting). This is the only stall/skipped-step detection this machine has -- see `docs/potential_issues.md` -- so tune these against real measured following error during commissioning, not the shipped defaults. |
| `base_period_ns`, `servo_period_ns` | LinuxCNC's realtime thread periods. |
| `mesa_board_driver`, `mesa_board_config` | The `loadrt` driver name and arguments for the Mesa card. |
| `watchdog_timeout_ns` | HAL watchdog timeout before LinuxCNC considers the realtime link dead. |

## 10. Units and travel range

**This machine is configured entirely in inches** for X/Y/Z
(`"linear_units": "inch"`) and degrees for A (always, independent of
`linear_units`). If converting a metric-datasheet value (a leadscrew pitch
in mm, a measured travel in mm) into `axis.json`, divide by 25.4.

**One deliberate exception:** the toolpath planner (`lcaf.toolpathing`, see
[toolpath_slicer.md](toolpath_slicer.md)) works entirely in millimetres --
its own, separate coordinate space. `LinuxCNCAxialInterface.move()` always
sends an explicit `G21` so LinuxCNC interprets those millimetre values
correctly regardless of the inch-native machine config above. You never
need to convert a toolpath file to inches.

**Every axis is zero-based, matching where it physically starts, with range
`[retracted_distance, extended_distance]`:**

- **X/Y/Z**: 0 is wherever LinuxCNC's own native homing finds the negative
  limit switch (section 7); the joint immediately backs off to
  `retracted_distance` (`0.25`) after that, which is also this end's soft
  limit floor -- so the practical range is `[0.25, extended_distance]`. The
  toolpath planner's own X=0 is the clamp at the physical switch, not the
  post-backoff resting point (see
  [toolpath_slicer.md](toolpath_slicer.md)) -- a toolpath's own zero
  coordinate would therefore command the joint below its own soft limit
  floor if commanded as-is. `ForgeBrain.load_jsonl()` corrects for this at
  parse time (`_retracted_offset_mm()`): every parsed `x`/`y`/`die_gap` is
  offset by that joint's own `retracted_distance` (converted to
  millimetres), so a toolpath's own zero always lands exactly on
  `retracted_distance`, not below it -- without ever touching the JSONL
  file itself. `rotation` (A) is never offset -- A has no
  `retracted_distance` (see section 6), so there is no standoff position to
  shift toward.
- **A**: `retracted_distance` and `extended_distance` are both `null`
  (section 6) -- genuinely unbounded, not a symmetric range. 0 is wherever
  it was sitting at power-on; it is free to move either direction from
  there with no software soft limit at all.

## 11. Setting everything up

1. Wire the board per section 5, and check jumpers against section 3.
2. Give the Pi a static IP on the 7I76E's subnet (section 4) and confirm
   `ping 192.168.1.121` works with the card powered.
3. Edit `configs/axis.json` and `configs/machine.json` for your hardware
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
  exposes one NML channel for the whole machine, not one per joint -- and
  issues the one machine-wide native "Home All" (`home_all_command()`, see
  section 7). `LinuxCNCAxialInterface` is the per-joint half: it knows how
  to move/read one joint and wait out its own share of that Home All
  (`begin_homing_wait()`/`poll_homing()`), plus re-home itself individually
  for a later retract-to-zero.

- **`axis.py`**'s `Axis` wraps one `LinuxCNCAxialInterface` with a small
  state machine of its own (`UNINITIALIZED -> READY -> MOVING/HOMING`, plus
  `FAULT`/`ESTOP`) purely for axis-level bookkeeping and logging -- it does
  not duplicate LinuxCNC's own motion state.

- **`motion_coordinator.py`**'s `MotionCoordinator` loads
  `MachineConfiguration` (default: `configs/machine.json` +
  `configs/axis.json`), regenerates the `.hal`/`.ini`, builds one `Axis`
  per configured joint, and sequences every toolpath operation through the
  fixed safe motion order (retract Z/Y/X, rotate A, move X/Y/Z). *That
  order* is the invariant not to be casually changed, for the same reason
  `forge_brain.py`'s HSFM shape is treated as stable -- what each retract
  step actually *does* underneath it has changed at least once already
  (retract-to-zero re-seeks the negative limit switch instead of a plain
  MDI move to 0.0 -- see section 7's "Retract-to-zero" above), without
  touching the order or the state names themselves.

- **`forge_brain.py`** (untouched) only ever talks to `MotionCoordinator`
  through `self.motion.interface` (machine-wide queries/commands) and
  `self.motion` itself (`start`, `update`, `home_all`, `emergency_stop`,
  `is_complete`, `has_fault`, `reset`). Nothing above this line needs to
  change for `forge_brain.py` to keep working against different hardware --
  that's the whole point of the config files.

See [potential_issues.md](potential_issues.md) for open concerns with this
pipeline that aren't config problems -- things worth checking before you
trust it against real hardware.
