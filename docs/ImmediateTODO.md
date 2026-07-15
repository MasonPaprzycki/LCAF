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

## Required Status Queries

Implement

```
machine_idle()

motion_complete()

motion_active()

interpreter_busy()

interpreter_idle()

program_running()

queue_empty()

machine_ready()

machine_fault()

machine_enabled()

machine_on()
```

---

## Required Motion Commands

Implement

```
move_axis()

move_axes()

home_axis()

home_all()

abort()

stop()

feed_hold()

resume()

dwell()

execute_mdi()
```

These should become the only methods ForgeBrain ever uses.

---

## Required HAL Interface

Implement

```
read_pin()

write_pin()

read_pins()

read_axis_pin()

limit_min()

limit_max()

limits()
```

Verify every HAL pin against the LinuxCNC configuration.

No hardcoded pin names should remain outside LinuxCNCInterface.

---

## Required Machine State

Expose

```
Current Position

Commanded Position

Current Velocity

Axis Enabled

Axis Homed

Machine Enabled

Machine Powered

Interpreter State

Motion State

Task State

Current Errors
```

Nothing outside LinuxCNCInterface should directly query LinuxCNC.

---

# Priority 2 — Finish Motor.py

Motor currently issues commands but does not own a complete software representation of an axis.

---

## Finish Axis State Machine

Automatically update

```
UNKNOWN

↓

DISABLED

↓

READY

↓

MOVING

↓

COMPLETE

↓

READY
```

Exceptional transitions

```
ANY

↓

FAULT
```

```
ANY

↓

ESTOP
```

State transitions should be computed entirely from LinuxCNC status.

ForgeBrain should never manually assign Motor states.

---

## Implement Polling

Motor.poll()

should

```
Read LinuxCNCInterface

↓

Update AxisStatus

↓

Update AxisState

↓

Return
```

Remove

```
interface.update()
```

from Motor.

ForgeBrain should perform one LinuxCNC update per heartbeat.

---

## Motion Completion

Implement

```
is_done()

is_idle()

is_faulted()

is_enabled()

is_homed()

position

target
```

using actual LinuxCNC status rather than placeholder values.

---

# Priority 3 — Complete ForgeBrain Heartbeat

ForgeBrain should become the only execution loop in the entire controller.

Heartbeat

```
Poll LinuxCNC

↓

Poll Motors

↓

Poll Sensors

↓

Update Machine State

↓

Execute Current Brain State

↓

Publish Telemetry

↓

Repeat
```

Nothing else owns an execution loop.

---

# Priority 4 — Motion Coordinator

The MotionCoordinator currently exists conceptually but needs to be fully implemented.

Responsibilities

```
Coordinate X

Coordinate Y

Coordinate Z

Coordinate A

Wait for completion

Report completion
```

It should never generate trajectories.

LinuxCNC remains responsible for motion planning.

---

## Required Motion Sequence

Every forging operation should execute

```
Retract Z

↓

Move XY

↓

Wait

↓

Rotate A

↓

Wait

↓

Strike Z

↓

Wait

↓

Retract Z

↓

Complete
```

Every stage blocks until LinuxCNC reports completion.

No command should ever be issued speculatively.

---

# Priority 5 — JSONL Toolpath Execution

Implement a JSONL parser inside ForgeBrain.

Minimum required functionality

```
Load JSONL

↓

Parse Line

↓

Create Operation

↓

Append Queue

↓

Execute

↓

Advance Queue
```

ForgeBrain should maintain

```
Current Operation

Current Queue Index

Remaining Operations

Completed Operations
```

No operation should be removed from memory until successfully completed.

---

## Initial JSONL Specification

Support only

```
Move

Rotate

Strike

Dwell
```

Ignore all future adaptive fields.

Keep the specification as small as possible.

---

# Priority 6 — ForgeBrain State Machine

Complete every remaining state.

```
INITIALIZING

↓

IDLE

↓

LOAD_OPERATION

↓

PLAN_OPERATION

↓

EXECUTE_OPERATION

↓

VERIFY_OPERATION

↓

NEXT_OPERATION

↓

COMPLETE_TOOLPATH
```

Every state should have

```
Entry

Update

Exit
```

No state should perform more than one logical responsibility.

---

# Priority 7 — Verification Layer

Before advancing to the next operation verify

```
Motion Complete

Machine Enabled

No Fault

No ESTOP

Correct Position
```

Only then advance the queue.

---

# Priority 8 — Controller Startup

Create

```
main.py
```

Responsibilities

```
Create LinuxCNCInterface

↓

Create Motors

↓

Create ForgeBrain

↓

Initialize

↓

Run
```

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