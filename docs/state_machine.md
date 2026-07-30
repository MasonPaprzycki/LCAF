# LCAF Hierarchical Finite State Machine (HFSM)

## Overview

The Low Cost Agility Forge (LCAF) control software is implemented as a **Hierarchical Finite State Machine (HFSM)**.

The hierarchy is

```
*independent                     *independent 

ForgeBrain                      AdaptativePlanner
    │                                   |
    ├── Motion Coordinator              |
    │       │                           |
    │       ├── Axis X                  |
    │       ├── Axis Y                  |
    │       ├── Axis Z                  |
    │       └── Axis  A                 |
    │                                   |
    ├── Toolpath Queue ─────────────────┘
    │   ForgeBrain calls adapt() to to pull the adapted execution queue 
    |      Then ForgeBrain internally updates its execution queue 
    |
    ├── Sensor Manager
    │
    ├── Telemetry
    │
    └── LinuxCNC Interface
```

**ForgeBrain** controls when and how commands are executed thorugh the subsystem.  

**AdaptativePlanner** AdaptivePlanner subscribes to the telemetry stream and processes new telemetry asynchronously. It will run simulation and analysis independently but it is only involved in the state machine when forge brain calls adapt() to update its tool path execution queue. Everything else is updated deterministically by Forge brain once every heart beat.  

---

# System Heartbeat

Every iteration of the controller executes exactly one deterministic update cycle.

```
Heartbeat

↓

Pull Sensor States

↓

Pull from Motion Coordinator which pulls axis states and diagnostics from linuxcnc 

↓

Publish telemetry to a central channel

↓

Update ForgeBrain State Machine which then updates motion coordinator and might pull the adaptive step from the AdaptativePlanner

↓

Generate commands 

↓

Send commands down hiearchy 

↓

Heartbeat end

↓

Repeat
```

Every subsystem reads the same LinuxCNC snapshot guaranteeing deterministic behavior across the entire controller.

---
# System architecture 
```


                          ┌────────────────────────────┐
                          │     Adaptive Planner       │
                          │                            │
                          │  Sensor Fusion             │
                          │  Process Models            │
                          │  FEM / MPM                 │
┌─────────┐               │  Digital Twin              │
│Telemetry│──────────────>│  AI Planning               │
└─────────┘               │  Robotics Web              │
     |                    └────────────┬───────────────┘
     |                                 |
     |                                 |
     |                                 |
     |                                 │ FprgeBrain pulls adapted tool path queue 
     |                                 │   during ADAPT_OPERATION
     |                                 │
     ▼                                 ▼
┌───────────────────────────────────────────┐
|             ForgeBrain HFSM               │
└───────────────────────────────────────────┘
      | 
      ▼
INITIALIZING
│
▼
IDLE
│
▼
EXECUTE_OPERATION
        │
        │ supervises
        ▼
┌───────────────────────────────────────────┐
│ Motion Coordinator HFSM                   │
│                                           │
│ IDLE                                      │
│   ↓                                       │
│ RETRACT_Z                                 │
│   ↓                                       │
│ VERIFY_Z_RETRACTED                        │
│   ↓                                       │
│ RETRACT_Y                                 │
│   ↓                                       |
| VERIFY_Y_RETRACTED                        │
│   ↓                                       |
│ RETRACT_X                                 │
│   ↓                                       │
│ VERIFY_X_RETRACTED                        │
│   ↓                                       │
│ ROTATE_A                                  │
│   ↓                                       │
│ VERIFY_ROTATION                           │
│   ↓                                       │
│ MOVE_X                                    │
│   ↓                                       │
│ VERIFY_X_POSITION                         |
|   ↓                                       |
│ MOVE_Y                                    │
│   ↓                                       │
│ VERIFY_Y_POSITION                         │
│   ↓                                       │
│ MOVE_Z                                    │
│   ↓                                       │
│ VERIFY_Z_POSITION                         │
│   ↓                                       │
│ COMPLETE                                  │
└───────────────────────────────────────────┘
        │
        ▼
ADAPT_OPERATION (updates tool path queue from AdaptativePlanner)
│
▼
EXECUTE_OPERATION
│
├────────────── More Operations? ────────────────────┐
│                                                    │
│ Yes                                                │ No
│                                                    │
▼                                                    ▼
will execute the operation                   COMPLETE_TOOLPATH
                                                     │
                                                     ▼
                                                    IDLE

```

---

# ForgeBrain State Machine

## INITIALIZING

Purpose

Prepare the machine for manufacturing.

Responsibilities

- Connect to LinuxCNC
- Initialize machine state
- Verify machine power
- Verify communication
- Home all axes
- Initialize telemetry
- Initialize sensors
- Initialize execution queue

Exit Conditions

```
INITIALIZING → IDLE
```

or

```
INITIALIZING → FAULT
```

---

## IDLE

Purpose

Wait for a manufacturing job.

Responsibilities

- Maintain machine state
- Publish telemetry
- Wait for operator input
- Wait for toolpath

Exit

```
IDLE → EXECUTE_OPERATION
```

---

## EXECUTE_OPERATION

Current Responsibilities

- Exit to COMPLETE_TOOLPATH if there is no operation left in the queue

- Retrieve the current ToolpathOperation from the queue (JSON decoding
  already happened once, for the whole file, back in load_jsonl() -- not
  redone per step here)

- Pass the ToolpathOperation to MotionCoordinator for expansion into motion
  commands, unconditionally, through the same retract/rotate/move sequence
  regardless of the operation's own `operation` field -- STRIKE, REHEAT,
  INSPECT, DWELL, MOVE_ONLY, and CUSTOM are not currently distinguished

- Advance the queue and record the completed step once MotionCoordinator
  reports the operation done

Not yet implemented

- Per-operation-type dispatch (e.g. a REHEAT or DWELL operation doing
  something other than the full physical motion sequence a STRIKE does)
- Target coordinate validation before handing the operation to
  MotionCoordinator

Neither is exercised today since `lcaf.toolpathing` (see
[toolpath_slicer.md](toolpath_slicer.md)) only ever emits STRIKE operations.

EXECUTE_OPERATION

↓

Retrieve current ToolpathOperation

↓

Motion Coordinator IDLE?

├── Yes
│
│   Start Motion Coordinator
│
└── No
     │
     ▼

Update Motion Coordinator

↓

Motion Fault?

├── Yes → FAULT
│
└── No
     │
     ▼

Motion Complete?

├── No
│
└── Yes
     │
     ▼

Reset Motion Coordinator

↓

ADAPT_OPERATION



## ADAPT_OPERATION

Purpose

Provide a single abstraction layer between manufacturing operations.

This state intentionally exists to support future autonomous manufacturing.

It includes a pass to the adaptive_planner which will execute verifiecation of previous expected behavior reports logistics and adapts tool paths. 

Current Responsibilities

- Not implemented yet, when adaptive behavior is implemented it will update the queue

A list of potential future responsibilities

- Billet temperature estimation
- FEM integration
- MPM integration
- Geometry prediction
- Tool wear estimation
- Sensor fusion
- Adaptive toolpath modification
- Reheat scheduling
- AI decision making

Exit

```
ADAPT_OPERATION → PLAN_OPERATION
```

---

## COMPLETE_TOOLPATH

Purpose

Finish manufacturing.

Responsibilities

- Clear execution queue
- Store statistics
- Publish completion
- Return controller to idle

Exit

```
COMPLETE_TOOLPATH → IDLE
```

---

## FAULT

Purpose

Safely terminate manufacturing.

Responsibilities

- Stop execution
- Preserve controller state
- Record fault
- Publish telemetry

Exit

```
FAULT → SHUTDOWN
```

---

## SHUTDOWN

Purpose

Terminate the ForgeBrain execution loop.

---

# Motion Coordinator State Machine

The Motion Coordinator is responsible for expanding a manufacturing operation into a deterministic machine motion sequence.

ForgeBrain provides the desired manufacturing operation.

The Motion Coordinator converts that operation into a sequence of axis commands.

The Motion Coordinator does not perform trajectory generation.

All trajectory planning, interpolation, acceleration, and servo control remain the responsibility of LinuxCNC.

Each motion state represents a required stage of the machine positioning sequence.

```
                         IDLE
                           │
                           ▼
                    RETRACT_Z
                           │
                           ▼
                 VERIFY_Z_RETRACTED
                           │
                           ▼
                  RETRACT_XY
                           │
                           ▼
                VERIFY_XY_RETRACTED
                           │
                           ▼
                    ROTATE_A
                           │
                           ▼
                VERIFY_ROTATION
                           │
                           ▼
                    MOVE_XY
                           │
                           ▼
                 VERIFY_XY_POSITION
                           │
                           ▼
                     MOVE_Z
                           │
                           ▼
                  VERIFY_Z_POSITION
                           │
                           ▼
                     COMPLETE


Any State

↓

FAULT

↓

IDLE
```

---

## State Definitions

### IDLE

Purpose

Wait for a new motion request from ForgeBrain.

Responsibilities

- Maintain current axis state
- Monitor LinuxCNC status
- Await operation command

Transition

```
IDLE → RETRACT_Z
```

---
### RETRACT_Z

Purpose

Return the Z axis to its configured retract position before any lateral or
rotational movement -- a plain commanded move to `retracted_distance` (or
`extended_distance` if this joint's `axis.json` sets `flip_retraction`),
through LinuxCNC's ordinary MDI motion, exactly like `MOVE_Z` later in this
same sequence. This project homes exactly once, at startup
(`INITIALIZING` -> `MotionCoordinator.home_all()`); it never re-seeks a
limit switch again afterward, so retracting never re-homes and never
re-references `position_offset_to_native` -- see `docs/hardware_setup.md`
section 7.

Responsibilities

- Command Z axis retract (`Axis.retract()`) through LinuxCNC
- Monitor retract progress

Transition

```
RETRACT_Z → VERIFY_Z_RETRACTED
```

---

### VERIFY_Z_RETRACTED

Purpose

Confirm the Z axis has reached its configured retract position.

Responsibilities

- Verify LinuxCNC motion completion
- Verify the retract has completed (`Axis.is_retracted()`)

Transition

```
VERIFY_Z_RETRACTED → RETRACT_XY
```

or

```
VERIFY_Z_RETRACTED → FAULT
```

---

### RETRACT_XY

Purpose

Return Y then X to their own configured retract positions, each by the
same plain commanded move as RETRACT_Z above (`docs/hardware_setup.md`
section 7). The actual code (`motion_coordinator.py`) sequences these as
separate RETRACT_Y/VERIFY_Y_RETRACTED then RETRACT_X/VERIFY_X_RETRACTED
states, not a single combined XY step -- this diagram groups them for
brevity, but see the "Motion Coordinator HFSM" diagram earlier in this
document for the exact per-axis states.

Responsibilities

- Command X/Y retract (`Axis.retract()`) through LinuxCNC
- Monitor retract progress

Transition

```
RETRACT_XY → VERIFY_XY_RETRACTED
```

---

### VERIFY_XY_RETRACTED

Purpose

Confirm X and Y have each reached their configured retract position.

Responsibilities

- Verify LinuxCNC motion completion
- Verify the retract has completed for each axis (`Axis.is_retracted()`)

Transition

```
VERIFY_XY_RETRACTED → ROTATE_A
```

or

```
VERIFY_XY_RETRACTED → FAULT
```

---

### ROTATE_A

Purpose

Rotate the workpiece to the required orientation.

The A axis is purely rotational and does not require positional clearance beyond the previous retract states.

Responsibilities

- Command rotary position through LinuxCNC
- Monitor motion status

Transition

```
ROTATE_A → VERIFY_ROTATION
```

---

### VERIFY_ROTATION

Purpose

Confirm the A axis has reached the commanded orientation.

Responsibilities

- Verify LinuxCNC motion completion
- Verify rotary position

Transition

```
VERIFY_ROTATION → MOVE_XY
```

or

```
VERIFY_ROTATION → FAULT
```

---

### MOVE_XY

Purpose

Move the tool to the required forging location.

Responsibilities

- Command X/Y position through LinuxCNC
- Monitor motion status

Transition

```
MOVE_XY → VERIFY_XY_POSITION
```

---

### VERIFY_XY_POSITION

Purpose

Confirm the X/Y axes have reached the commanded position.

Responsibilities

- Verify LinuxCNC motion completion
- Verify commanded position achieved

Transition

```
VERIFY_XY_POSITION → MOVE_Z
```

or

```
VERIFY_XY_POSITION → FAULT
```

---

### MOVE_Z

Purpose

Move the forging tool into the commanded Z position.

This represents the final approach into the forging operation.

Responsibilities

- Command Z position through LinuxCNC
- Monitor motion status

Transition

```
MOVE_Z → VERIFY_Z_POSITION
```

---

### VERIFY_Z_POSITION

Purpose

Confirm the forging position has been reached.

Responsibilities

- Verify LinuxCNC motion completion
- Verify commanded Z position achieved

Transition

```
VERIFY_Z_POSITION → COMPLETE
```

or

```
VERIFY_Z_POSITION → FAULT
```

---

### COMPLETE

Purpose

Report successful completion of the current motion sequence.

Responsibilities

- Notify ForgeBrain motion has completed
- Return Motion Coordinator to idle

Transition

```
COMPLETE → IDLE
```

---

### FAULT

Purpose

Handle motion failures.

Possible causes include

- LinuxCNC motion fault
- Axis fault
- Following error
- Communication failure
- Invalid motion request

Responsibilities

- Stop current motion sequence
- Report fault condition upward
- Preserve diagnostic information

Transition

```
FAULT → IDLE
```

after fault recovery has been handled by the higher-level controller.

---

The Motion Coordinator owns motion sequencing only.

It does not control individual motors directly, generate trajectories, or bypass LinuxCNC.

Its responsibility is to guarantee that every manufacturing operation follows the required safe sequence:

```
Retract Z
↓
Retract X/Y
↓
Rotate A
↓
Move X/Y
↓
Move Z
```

while delegating all actual motion execution to LinuxCNC.

---
# Axis State Machine

Every machine axis owns an independent Axis object.

The Axis class is **not** an independent controller.

It exists solely as an abstraction of a single machine axis and executes commands delegated by the Motion Coordinator.

Each Axis maintains its own operational state.

```
                     UNINITIALIZED
                           │
                           ▼
                        READY
                ┌──────────┼──────────┐
                │          │          │
                ▼          ▼          ▼
             HOMING    RETRACTING   MOVING
        (constant vel) (constant vel) (Toolpath sub operation)
                │          │          │
                └──────────┴─────┬────┘
                                 │
                                 ▼
                              READY


Any Operational State

        │

        ▼

      FAULT

        │

   Fault Recovery

        │

        ▼

    READY
```

## State Definitions

### UNINITIALIZED

The Axis object has been created but has not yet established communication with LinuxCNC.

Responsibilities

- No motion permitted
- Await initialization
- Verify communication with LinuxCNC

Transition

```
UNINITIALIZED → READY
```

---


### READY

The axis is enabled and capable of accepting motion commands.

Responsibilities

- Accept commands from the Motion Coordinator
- Report current axis status
- Monitor LinuxCNC status

Transitions

```
READY → CONSTANT VELOCITY
```

or

```
READY → POSITION MOTION
```

or

```
READY → FAULT
```

---

### HOMING

The axis is executing a continuous velocity command through LinuxCNC to home the axis. 

Motion continues until a limit switch signal. 

The Axis object does not directly terminate motion. It reports the event and LinuxCNC remains responsible for stopping the commanded motion.

Transitions

```
HOMING → READY
```

or

```
HOMING → FAULT
```

---

### RETRACTING

The axis is executing a plain position command through LinuxCNC to its
configured retract position (`retracted_distance`, or `extended_distance`
if this joint's `axis.json` sets `flip_retraction`) -- mechanically no
different from MOVING, just to a fixed, pre-configured target rather than
an operation's own commanded coordinate. This project homes exactly once,
at startup (see HOMING below); RETRACTING never re-homes and never
re-seeks a limit switch. It does require the axis to have already
completed HOMING at least once this session -- entered once per retract by
`MotionCoordinator` (`Axis.retract()`), not just once per session the way
HOMING is.

The Axis object does not directly terminate motion. It reports the event
and LinuxCNC remains responsible for stopping the commanded motion. A hard
limit trip on this axis during RETRACTING is a genuine fault, the same way
it is during MOVING -- unlike HOMING, RETRACTING never deliberately drives
onto a switch.

Transitions

```
RETRACTING → READY
```

or

```
RETRACTING → FAULT
```

---

### MOVING

The axis is executing a position command through LinuxCNC.

LinuxCNC remains responsible for

- trajectory generation
- interpolation
- acceleration
- velocity limiting
- following error detection

The Axis object only monitors LinuxCNC status and determines when the commanded motion has completed.

Transitions

```
MOVING → READY
```

or

```
MOVING → FAULT
```

---

### FAULT

The axis has encountered an abnormal condition preventing safe operation.

FAULT is a latched error condition. 

Examples include

- LinuxCNC axis fault
- Following error
- Drive amplifier fault
- Communication failure
- Unexpected loss of axis state
- Invalid motion command
- Homing timing out or reporting a joint fault before the expected
  limit-switch/native-homed status was observed
- Retracting before this axis has ever completed homing this session

Responsibilities

- Reject all motion commands
- Preserve fault information
- Report fault state through telemetry
- Prevent automatic reactivation

Recovery requires an explicit reset procedure.

Transition

```
FAULT → READY
```

after fault recovery has been completed.

---

The Axis state machine never performs trajectory generation or real-time control.

All motion execution is delegated to LinuxCNC, while the Axis object serves only as an abstraction of axis state, command routing, and fault reporting.

---

# LinuxCNC Interface

The LinuxCNC Interface intentionally contains **no state machine**. It is
split into two classes (`lcaf.control.linuxcnc_interface`), the only module
that imports `linuxcnc`/`hal`:

- **LinuxCNCMachineInterface**: one instance, owning the single shared
  `linuxcnc.command()`/`stat()`/`error_channel()` connection for the whole
  machine (LinuxCNC exposes one NML channel total, not one per joint).
  Answers machine-wide queries (`machine_on`, `estop`, `all_homed`) and
  issues machine-wide commands (`machine_on_command`, `machine_off`,
  `estop_command`, `estop_reset`, `abort`). This is the object
  MotionCoordinator publishes as `self.interface` for ForgeBrain to query.
- **LinuxCNCAxialInterface**: one instance per joint, wrapping that same
  shared connection for its own joint number. Translates Axis requests into
  per-joint LinuxCNC API calls -- MDI motion (`move`, `dwell`), homing
  (always LinuxCNC's own native homing sequence -- see
  `docs/hardware_setup.md` section 7), and status/fault reads -- and
  exposes that joint's LinuxCNC status back to Axis.

Its sole responsibility is translating ForgeBrain/MotionCoordinator/Axis requests into LinuxCNC API calls and exposing LinuxCNC status back to the controller.

```
ForgeBrain

↓

Motion Coordinator

↓

Axis

↓

LinuxCNC Interface (LinuxCNCMachineInterface + LinuxCNCAxialInterface)

↓

LinuxCNC API

↓

HAL

↓

Mesa FPGA
```

LinuxCNC remains the sole owner of

- trajectory generation
- interpolation
- servo control
- limit handling
- homing motion
- following error detection
- deterministic real-time execution

The Axis object never commands hardware directly. Every motion request is delegated through the LinuxCNC Interface.

---

# Design Philosophy

The controller hierarchy is intentionally unidirectional.

```
ForgeBrain
        │
        ▼
Motion Coordinator
        │
        ▼
      Axis
        │
        ▼
LinuxCNC Interface
        │
        ▼
     LinuxCNC
        │
        ▼
       HAL
        │
        ▼
    Mesa FPGA
        │
        ▼
   Servo Drives
        │
        ▼
  Mechanical Forge
```

Information flows upward through periodic polling.

Commands flow downward through delegation.

Every heartbeat begins in ForgeBrain.

Every command originates in ForgeBrain.

Every subsystem is synchronized by the same heartbeat.

The Motion Coordinator expands every manufacturing operation into the invariant safe-motion sequence

```
Retract Z
↓
Retract X/Y
↓
Rotate A
↓
Move X/Y
↓
Move Z
```

while each Axis object is responsible only for representing the operational state of a single axis and forwarding commands to LinuxCNC.

This separation of responsibilities guarantees deterministic execution, simplifies debugging, and provides a stable foundation for future sensor integration, adaptive process control, digital twins, FEM/MPM simulation, and autonomous forging algorithms without modifying the underlying machine control architecture.