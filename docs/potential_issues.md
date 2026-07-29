# Potential Issues

- **Retract-to-zero trips the negative limit switch on every retract, not
  just at boot.** `MotionCoordinator.retract_axis()` re-seeks X/Y/Z's own
  negative limit switch on every single retract (see
  `docs/hardware_setup.md` section 7, "Retract-to-zero") rather than
  trusting a plain commanded move to 0.0 -- deliberate, since these are
  open-loop steppers with no position feedback and a plain move could
  silently carry forward drift. The tradeoff: a switch that used to see one
  actuation per session (at `home_all()`) now sees one per operation, for
  the life of the machine. This relies on `HOME_IGNORE_LIMITS` (see
  `docs/hardware_setup.md` section 7) succeeding on every one of those
  retracts, the same mechanism initial homing already depends on -- nothing
  new there, just far more frequent. Worth watching during commissioning:
  switch mechanical life, and whether the extra seek time per operation is
  acceptable for cycle time on a real toolpath.
- **No fault recovery path is implemented.** `AxisState.FAULT` and
  `MotionCoordinatorState.FAULT` are now both reachable (see above), but
  there is still no code anywhere that clears a fault and returns to
  `READY`/`IDLE` -- `docs/state_machine.md` describes `FAULT -> READY`
  "after fault recovery has been completed" and `FAULT -> SHUTDOWN` for
  `ForgeBrain`, but no explicit recovery procedure (clearing LinuxCNC's own
  amp fault, re-homing, operator confirmation, or some combination) exists
  yet. This needs a real decision about what "recovered" means for this
  machine (at minimum: re-homing is almost certainly required afterward,
  per the open-loop-steppers point below) before it's implemented, rather
  than guessing at a recovery procedure. This now covers three distinct
  fault sources into the same unimplemented recovery gap: following error
  (`is_faulted()`), a hard-limit-switch trip outside homing
  (`is_on_hard_limit()` -- see `docs/hardware_setup.md`), and ESTOP -- a
  hard-limit recovery specifically also needs `override_limits()` before
  LinuxCNC will accept the re-enable + jog-off-switch + re-home sequence,
  which is one more reason not to guess at this without a real decision.
  Separately, `LinuxCNCMachineInterface.get_errors()` (drains LinuxCNC's
  NML error channel, which is where `"joint N on limit switch error"`
  itself actually lands as text) is defined but never called anywhere in
  the control loop -- that message currently reaches no log of this
  project's own.
- **Open-loop steppers, including Z.** None of the four joints send real
  position feedback into LinuxCNC -- X/Y/A have no encoders at all, and Z's
  closed-loop control is entirely internal to its own driver (the driver
  reads its own encoder and closes the loop itself); nothing about that
  ever reaches the Mesa card or HAL, so **LinuxCNC treats Z exactly like the
  three genuinely open-loop joints.** `status.joint[n]['inpos']` and
  `status.joint[n]['fault']` (following error vs. `machine.json`'s
  `linear_ferror_in`/`linear_min_ferror_in`/`angular_ferror_deg`/
  `angular_min_ferror_deg`) are therefore the only two signals this system
  has for "did the commanded motion actually complete correctly" -- both
  are what `MotionCoordinator`/`Axis` check for that question (see above),
  which is the most this software can honestly verify without real
  feedback. (`Axis` separately also checks `is_on_hard_limit()`, but that
  answers a different question -- "did a limit switch trip" -- not
  motion-completion.) It cannot detect a stall that happens to drift within
  tolerance, or that the machine was moved by hand while powered off.
  Re-home every joint after any ESTOP, fault, or power interruption for
  that reason. Tune `linear_ferror_in`/`linear_min_ferror_in`/
  `angular_ferror_deg`/`angular_min_ferror_deg` against real measured
  following error during commissioning -- the shipped defaults are starting
  points, not measured values.
- **Thin test coverage.** `debug/tests/test_toolpath_slicer.py`,
  `debug/tests/test_joint_configuration.py`,
  `debug/tests/test_linuxcnc_interface.py`, `debug/tests/test_axis.py`, and
  `debug/tests/test_motion_coordinator.py` (the latter three against fake
  `linuxcnc`/`hal` modules in `debug/tests/fake_linuxcnc.py`, installed by
  `debug/tests/conftest.py` -- the real modules only exist on the Raspberry
  Pi target) are the only test files -- see `debug/tests/README.md` for
  what each one covers. That coverage is specifically the native homing /
  `HOME_IGNORE_LIMITS` behavior, `dual_limit_switches` single-vs-dual-switch
  homing, retract-to-zero, and the `is_on_hard_limit()` fault detection
  described in `docs/hardware_setup.md` sections 5 and 7; most of
  `motion_coordinator.py` (everything past the retract states) and
  `axis.py`/`linuxcnc_interface.py` beyond what's listed above still have no
  automated coverage. The fakes model LinuxCNC's Python API surface, not
  its actual servo-thread/hard-limit logic (`control.c`), and they never
  advance `joint_actual_position`/`inpos`/`velocity` on their own in
  response to an MDI move (tests that need a specific outcome set those
  fields directly) -- so passing tests confirm this project's own call
  sequencing is correct, not that real LinuxCNC reacts the way the source
  reading behind that fix predicts -- that still needs confirming against
  real hardware per `docs/hardware_setup.md` section 11 (specifically: does
  a deliberate homing/retract seek into a switch complete without an "on
  limit switch error", and does an *unintended* switch trip outside
  homing/retracting still correctly disable the machine and get caught by
  `is_on_hard_limit()`).
