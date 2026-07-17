# Immediate TODO

## Objective

The objective of this document is to define every remaining task required to reach the first successful execution of a JSONL forging toolpath on the physical machine.

This document intentionally excludes long-term research topics including:

- Adaptive control
- FEM integration
- MPM integration
- Geometry prediction
- Sensor fusion
- Billet process modeling
- AI process planning

The only objective is:

```
ForgeBrain
      ↓
LinuxCNC
      ↓
Mesa
      ↓
Servo Drives
      ↓
Machine Motion
```

---

# Priority 1 — Complete LinuxCNC Interface

This is currently the largest software bottleneck.

ForgeBrain cannot become deterministic until LinuxCNCInterface exposes enough information to determine machine state.


# Priority 2 — Finish Motor.py

Motor currently issues commands but does not own a complete software representation of an axis.

---

# Priority 5 — JSONL Toolpath Execution

Implement a JSONL parser inside ForgeBrain.



# Priority 7 — Verification Layer

---

# Priority 8 — Controller Startup


No initialization code should remain outside main.py.

---

# Priority 9 — LinuxCNC Configuration

Verify

```
machine.ini

hal.conf
```

Confirm

```
Joint Numbers

Axis Letters

Home Directions

Machine Limits

Feed Limits

HAL Pins

Mesa Pins

Encoder Scaling

Motor Scaling
```

ForgeBrain assumes LinuxCNC configuration is correct.

---

# Priority 10 — Physical Machine Test

The first physical test should intentionally be minimal.

Create a JSONL file containing

```
Move X

↓

Move Y

↓

Rotate A

↓

Strike

↓

Retract

↓

End
```

Execution procedure

```
Start LinuxCNC

↓

Power Machine

↓

Home Machine

↓

Launch ForgeBrain

↓

Load JSONL

↓

Execute

↓

Observe Motion

↓

Return Idle
```

Only after this succeeds should additional manufacturing operations be added.

---

# Definition of Minimum Viable Controller

The controller is considered operational when all of the following are true.

```
✓ ForgeBrain starts successfully

✓ LinuxCNCInterface connects

✓ Motors synchronize correctly

✓ Machine homes

✓ JSONL loads

✓ Queue executes

✓ Every motion waits for completion

✓ Every operation verifies completion

✓ Entire toolpath executes autonomously

✓ Controller returns to IDLE

✓ No operator intervention required
```

At this point the supervisory controller is complete.

All future development (sensors, adaptive control, simulations, digital twins, AI planning, and process modeling) will extend this verified execution pipeline rather than modifying its core architecture.