# Hardware Setup: Raspberry Pi + Mesa 7I76E/7I76EU + LinuxCNC

This describes the physical/network setup this project's config generator
targets, and how a change to that hardware flows through the code:

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
               |
               v
    lcaf.control.axis.Axis  -->  lcaf.control.motion_coordinator.MotionCoordinator
                                            |
                                            v
                                   lcaf.control.forge_brain.ForgeBrain (the state machine)

You should never need to hand-write a .hal or .ini file, or touch
`linuxcnc_interface.py`/`axis.py`/`motion_coordinator.py`, to add a motor or
change a pin. Edit `configs/axis.jsonl` (per-joint hardware) and
`configs/machine.json` (machine-wide settings), and everything below them
regenerates and rewires itself.

## 1. The board

Target board: **Mesa Electronics 7I76E** (this project was priced/wired
against the 7I76EU variant; the two are pin- and HAL-compatible -- both are
sold as "7I76E/7I76ED Ethernet Step/Dir Plus I/O Daughtercard", manual
`7i76eman.pdf` on mesanet.com). It is a standalone remote FPGA card -- it
does not sit on a separate carrier card, it connects to the Pi directly over
Ethernet.

Per the manual, it provides:

- 5 channels of differential step/dir (4 on connector TB2, a 5th on TB3)
- 1 encoder input with index (spindle-oriented; unused by this project)
- 1 isolated analog spindle output + 2 isolated spindle control outputs (unused)
- 48 isolated field I/O points: 32 sinking inputs, 16 outputs (sourcing on
  the 7I76E, sinking on the 7I76ED)
- 1 RS-422 port for Mesa SSERIAL expansion (unused)

This project only uses 4 of the 5 step/dir channels (X, Y, Z, A) and 4 of
the 16 outputs (per-axis enable) plus 6 of the 32 inputs (limit switches for
X/Y/Z only -- see "Why A has no limit switches" below). Everything else on
the card (inputs 6-31, outputs 4-15, the encoder, the spindle interface, the
RS-422 port, and the 5th stepgen) is free for future use (coolant, a tool
changer, a real spindle, etc.).

## 2. Jumpers

Default/as-shipped jumper positions are almost all correct for this
project. Only these matter:

| Jumper | Position | Why |
|---|---|---|
| W1 (VIN source) | LEFT (default) | VIN is powered from field power; you only need to feed 24V into TB1, not a separate 5V/VIN supply. |
| W2, W3 (IP address) | DOWN, DOWN (default) | Fixed IP `192.168.1.121`. Matches `mesa_board_config` in `configs/machine.json`. If you change the IP (e.g. to run multiple Mesa cards), update jumpers *and* `board_ip=` in machine.json together. |
| W8 (setup/operate) | LEFT (operate mode, default) | Must be in operate mode for normal running. Only move to RIGHT (setup mode, 115.2K baud) when using Mesa's `RPD`/`WPD`/`UFLBP` command-line tools to inspect/flash the card. |
| W9 (flash select) | UP (primary, default) | Only move to DOWN if the primary flash image fails to configure (see manual "DUAL EEPROMS"); move back to UP once recovered. |
| W4 (preconfig pullups) | UP (default) | Leave enabled. |
| W5, W6, W7, W12 | defaults | Only relevant if you're using the P1/P2 expansion headers, which this project does not. |

## 3. Network

The 7I76E talks UDP (protocol "LBP16") directly to the `hm2_eth` HAL driver
on the Pi -- there's no OS-level network stack involvement beyond basic
IPv4/ARP, but the Pi still needs a matching static IP on the same subnet as
the card's default `192.168.1.121`.

On the Raspberry Pi, give the NIC connected to the 7I76E a static address in
the same /24, e.g. `192.168.1.10`, and connect the two directly with an
Ethernet cable (a switch works too, but isn't necessary point-to-point).
Windows machines are known to drop the 7I76E's UDP replies while refreshing
ARP (see manual "WINDOWS ARP ISSUES") -- not relevant on the Pi/Linux target,
but worth knowing if you ever bring the card up from a Windows dev box for
bench testing.

Verify connectivity before touching LinuxCNC:

    ping 192.168.1.121

## 4. Wiring

All field wiring uses the card's pluggable 3.5mm screw terminals. Voltages
below assume a 24VDC field supply, which is what this project's config
targets (`hm2 field power is 5-32VDC`).

### TB1 -- Field power

| Pin | Signal |
|---|---|
| 1-4 | VFIELD (24V in) |
| 5 | VIN (leave unconnected; W1 ties it to VFIELD) |
| 8 | GROUND |

Per the manual: **VFIELD must connect directly to the DC source with no
switch, breaker, or relay in the circuit** (a fuse is fine). Never switch
field power through a mechanical contact.

### TB2 -- X/Y/Z step/dir (joints 0-2)

| Pins | Signal | Joint |
|---|---|---|
| 2/3 | STEP0-/+ | X (joint 0) |
| 4/5 | DIR0-/+ | X (joint 0) |
| 8/9 | STEP1-/+ | Y (joint 1) |
| 10/11 | DIR1-/+ | Y (joint 1) |
| 14/15 | STEP2-/+ | Z (joint 2) |
| 16/17 | DIR2-/+ | Z (joint 2) |

Use the differential (+/-) pair if your driver has RS-422-style differential
inputs; otherwise wire only the `+` side and return the driver's common to
the 7I76E's GND or +5VP pin on the same connector, per the driver's
single-ended wiring diagram.

### TB3 -- A step/dir (joint 3) + unused encoder/RS-422

| Pins | Signal | Joint |
|---|---|---|
| 2/3 | STEP4-/+ | A (joint 3) |
| 4/5 | DIR4-/+ | A (joint 3) |
| 7/8, 10/11, 13/14 | ENCA/B/IDX | unused (spindle encoder) |
| 16-19 | RS-422 RX/TX | unused |

### TB6 -- Inputs 0-15 / Outputs 0-7

| Pin | Signal | Used for |
|---|---|---|
| 1 | INPUT0 | X negative limit switch |
| 2 | INPUT1 | X positive limit switch |
| 3 | INPUT2 | Y negative limit switch |
| 4 | INPUT3 | Y positive limit switch |
| 5 | INPUT4 | Z negative limit switch |
| 6 | INPUT5 | Z positive limit switch |
| 7-16 | INPUT6-15 | free |
| 17 | OUTPUT0 | X driver enable |
| 18 | OUTPUT1 | Y driver enable |
| 19 | OUTPUT2 | Z driver enable |
| 20 | OUTPUT3 | A driver enable |
| 21-24 | OUTPUT4-7 | free |

### TB5 -- Inputs 16-31 / Outputs 8-15

All free for future expansion (none used by this project).

### Limit switch wiring

7I76E inputs are **sinking** (they sense a positive voltage sourced onto
them, they don't source anything themselves). The manual recommends wiring
limit switches **normally closed**, one leg to field power (+24V) and the
other to the input pin, so the normal/at-rest machine state is "input
active." That way a broken wire or a wire shorted to ground reads as a
*fault* (input goes inactive) instead of silently looking like "not at
limit." `invert_negative_limit`/`invert_positive_limit` in
`axis.jsonl` flip a joint's interpretation in software (via the field
component's `-not` reading pin) if your wiring convention runs the other way.

### Why A has no limit switches

The A axis is a continuous rotary joint: it always homes to zero wherever
it happens to be sitting at boot (no seeking, no switches) and is free to
rotate a full circle in either direction from there. That's
`"has_limit_switches": false` for joint 3 in `axis.jsonl` --
`LinuxCNCAxialInterface.home_axis()` special-cases this: it zeroes the
position offset immediately and trusts the joint's static `soft_min_mm`/
`soft_max_mm` (`-360.0`/`360.0`) instead of jogging to find a switch. See
`docs/toolpath_slicer.md` for how the toolpath planner treats A the same way
(unbounded, not validated against `axis.jsonl`).

## 5. `configs/axis.jsonl` field reference

One JSON object per line, one line per joint. Fields that matter for
wiring/HAL generation:

| Field | Meaning |
|---|---|
| `joint` | LinuxCNC joint number (0-3 here); also selects the stepgen instance and `joint.N.*` HAL pins. |
| `axis` | Cartesian letter (X/Y/Z/A). |
| `motor_steps_per_revolution`, `microsteps`, `leadscrew_pitch_mm` | Used to derive `steps_per_mm` (or steps/degree for an angular joint). |
| `mesa_stepgen` | e.g. `hm2_7i76e.0.stepgen.00` -- which stepgen instance drives this joint. |
| `enable_output` | Field output pin (e.g. `hm2_7i76e.0.7i76.0.0.output-00`) wired to the driver's enable input. |
| `negative_limit_input` / `positive_limit_input` | Field input pins for the limit switches, or `null` if `has_limit_switches` is `false`. |
| `is_angular` | `true` for a rotary joint (INI `TYPE = ANGULAR`, values interpreted in degrees). |
| `has_limit_switches` | `false` skips switch-seeking homing entirely (see above). |
| `invert_direction` | Flips the stepgen's `position-scale` sign. |
| `invert_negative_limit` / `invert_positive_limit` | Reads the field input's `-not` pin instead. |
| `invert_enable` | Documented as unsupported at the HAL level for the 7I76E's field outputs (no per-output invert pin exists) -- the generator emits a warning comment instead of wiring something wrong; fix polarity by wiring to the driver's other enable terminal. |
| `soft_min_mm` / `soft_max_mm` | INI travel bounds (`MIN_LIMIT`/`MAX_LIMIT`). For a switched linear joint this is a generous mechanical envelope, not the authoritative range -- the real range is measured by `home_axis()` off the physical switches at homing time and enforced by `LinuxCNCAxialInterface.is_position_in_range()`, independent of the INI. For a switchless joint (A) these values *are* authoritative, since nothing measures them at runtime. |

`configs/machine.json` carries the machine-wide settings: velocities,
thread periods, and `mesa_board_driver`/`mesa_board_config` (the `loadrt`
line's driver name and arguments -- for the 7I76E this is `hm2_eth` plus
`board_ip=` and a `config=` string requesting 1 encoder, 0 PWM generators,
and 5 stepgens, matching the card's fixed onboard firmware).

## 6. Generating and running

`lcaf.utils.joint_configuration.write_config_files()` renders
`configs/generated/LCAF_Forge.hal` and `.ini` from those two files.
`MotionCoordinator.__init__` calls this automatically every time it starts,
so the generated files are always in sync with `axis.jsonl`/`machine.json`
-- but LinuxCNC itself only reads them when *it* starts, so the commissioning
order is:

1. Edit `configs/axis.jsonl` / `configs/machine.json` for your hardware.
2. Regenerate once by hand to review the output before trusting it live:
   ```
   python -c "from lcaf.utils.joint_configuration import load_machine_configuration, write_config_files; write_config_files(load_machine_configuration('configs/machine.json'), 'configs/generated')"
   ```
3. Power up the 7I76E, confirm `ping 192.168.1.121` works.
4. Start LinuxCNC against the generated INI: `linuxcnc configs/generated/LCAF_Forge.ini`.
5. In a second terminal, sanity-check the board came up and pins exist:
   `halcmd show pin hm2_7i76e`. Confirm the `config=` stepgen/encoder counts
   in `machine.json` actually match what the card's firmware exposes here --
   adjust `mesa_board_config` and regenerate if not.
6. Only then start the LCAF control process (`python -m lcaf.control.main`),
   which connects to the already-running LinuxCNC over NML.

Regenerating `configs/generated/*` while LinuxCNC is already running does
not affect that running instance -- restart LinuxCNC to pick up hardware
config changes.

## 7. Code walkthrough: config to state machine

- **`joint_configuration.py`** is pure data + text generation: it has no
  import of `linuxcnc`/`hal` and can run anywhere. `JointConfiguration` is
  one joint's hardware description; `MachineConfiguration` bundles all of
  them plus machine-wide settings; `generate_hal`/`generate_ini` render the
  files LinuxCNC actually loads.

- **`linuxcnc_interface.py`** is the only place that imports `linuxcnc`/
  `hal`. `LinuxCNCMachineInterface` owns the single shared
  `linuxcnc.command()`/`stat()`/`error_channel()` connection -- LinuxCNC
  exposes one NML channel for the whole machine, not one per joint, so
  every axis is handed a reference to this one object rather than opening
  its own. It's also what `MotionCoordinator` publishes as `self.interface`
  for `forge_brain.py` to query (`estop()`, `machine_on()`, `all_homed()`,
  etc.) -- `all_homed()` checks each registered `Axis.is_homed()` rather
  than LinuxCNC's own `status.homed`, because homing here is done in
  software (jogging to switches, or zeroing immediately for a switchless
  joint) rather than through LinuxCNC's native homing sequence.
  `LinuxCNCAxialInterface` is the per-joint half: it takes a
  `JointConfiguration` plus the shared machine interface, and only knows how
  to move/jog/home/read *one* joint (indexing into the shared `status.joint[n]`).

- **`axis.py`**'s `Axis` wraps one `LinuxCNCAxialInterface` with a small
  state machine of its own (`UNINITIALIZED -> READY -> MOVING/HOMING`,
  plus `FAULT`/`ESTOP`) purely for axis-level bookkeeping and logging --
  it does not duplicate LinuxCNC's own motion state. `Axis` is constructed
  from a `JointConfiguration` and the shared `LinuxCNCMachineInterface`
  directly, so there's exactly one owner of the real NML connection no
  matter how many axes exist.

- **`motion_coordinator.py`**'s `MotionCoordinator` loads
  `MachineConfiguration` (default: `configs/machine.json` +
  `configs/axis.jsonl`), regenerates the `.hal`/`.ini`, builds one `Axis` per
  configured joint keyed by lowercase axis letter (`"x"`, `"y"`, `"z"`,
  `"a"`), and registers them with the shared interface. Its own
  `MotionCoordinatorState` (retract Z/Y/X, rotate A, move X/Y/Z, ...)
  sequences a single toolpath operation across those axes -- this part is
  unchanged by any of the above and is not to be modified per the same
  constraint that protects `forge_brain.py`'s HSFM.

- **`forge_brain.py`** (untouched) only ever talks to `MotionCoordinator`
  through `self.motion.interface` (machine-wide queries/commands) and
  `self.motion` itself (`start`, `update`, `home_all`, `emergency_stop`,
  `is_complete`, `has_fault`, `reset`). Nothing above this line needs to
  change for `forge_brain.py` to keep working against different hardware --
  that's the whole point of the config files.
