# Potential Issues

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
  than guessing at a recovery procedure.
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
  are now what `MotionCoordinator`/`Axis` actually check (see above), which
  is the most this software can honestly verify without real feedback. It
  cannot detect a stall that happens to drift within tolerance, or that the
  machine was moved by hand while powered off. Re-home every joint after
  any ESTOP, fault, or power interruption for that reason. Tune
  `linear_ferror_in`/`linear_min_ferror_in`/`angular_ferror_deg`/
  `angular_min_ferror_deg` against real measured following error during
  commissioning -- the shipped defaults are starting points, not measured
  values.
- **Thin test coverage.** `tests/test_toolpath_slicer.py` is still the only
  test file. Both `joint_configuration.py`'s HAL/INI generator (native vs.
  software homing, symmetric vs. zero-based travel) and the
  `motion_coordinator.py`/`axis.py`/`linuxcnc_interface.py` fixes above have
  no automated coverage -- they were checked by hand with stubbed
  `linuxcnc`/`hal` modules during this pass (since the real modules only
  exist on the Raspberry Pi target), not by a real test in this repo.
