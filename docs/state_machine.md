# LCAF Hierarchical Finite State Machine (HFSM)

## Overview

The Low Cost Agility Forge (LCAF) control software is implemented as a **Hierarchical Finite State Machine (HFSM)**.

The hierarchy is

```
ForgeBrain
    │
    ├── Motion Coordinator
    │       │
    │       ├── Motor X
    │       ├── Motor Y
    │       ├── Motor Z
    │       └── Motor A
    │
    ├── Toolpath Queue
    │
    ├── Sensor Manager
    │
    ├── Telemetry
    │
    └── LinuxCNC Interface
```

**ForgeBrain** controls when and how commands are executed thorugh the subsystem.  

No subsystem executes independently. And no module has its own execution loop.

Every module is updated once every heartbeat.

---

# System Heartbeat

Every iteration of the controller executes exactly one deterministic update cycle.

```
Heartbeat

↓

Update LinuxCNC

↓

Update Motor States   

↓

Update Sensor States

↓

Update Motion Coordinator

↓

Update ForgeBrain State Machine

↓

Generate commands 

↓

Send commands down hiearchy 

↓

Publish Telemetry

↓

Heartbeat end

↓

Repeat
```

Every subsystem reads the same LinuxCNC snapshot guaranteeing deterministic behavior across the entire controller.

---
# Hierarchical State Machine

The complete controller hierarchy is shown below.

```
INITIALIZING
│
▼
IDLE
│
▼
LOAD_OPERATION
│
▼
PLAN_OPERATION
│
▼
EXECUTE_OPERATION
│
│
├─────────────────────────────────────────────────────────────────────────────┐
│                                                                             │
│                           Motion Coordinator                                │
│                                                                             │
│      Executes every manufacturing operation using a deterministic           │
│      safe-motion sequence.                                                  │
│                                                                             │
│     ┌──────────────────────────────────────────────────────────────┐        │
│     │                                                              │        │
│     ▼                                                              ▼        │
│  Retract Z                                                  Verify Z Safe    │
│     │                                                              │        │
│     ▼                                                              ▼        │
│  Retract X/Y                                                Verify X/Y Safe  │
│     │                                                              │        │
│     ▼                                                              ▼        │
│  Rotate A                                                   Verify Rotation  │
│     │                                                              │        │
│     ▼                                                              ▼        │
│  Move X/Y                                                   Verify X/Y       │
│     │                                                              │        │
│     ▼                                                              ▼        │
│  Move Z                                                     Verify Z         │
│     │                                                              │        │
│     └──────────────────────────────┬───────────────────────────────┘        │
│                                    ▼                                        │
│                              Motion Complete                                │
│                                                                             │
│      Every motion request is delegated to one or more Motor objects.        │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
│
▼
VERIFY_OPERATION
│
▼
ADAPT_OPERATION
│
▼
NEXT_OPERATION
│
├────────────── More Operations? ──────────────┐
│                                              │
│ Yes                                          │ No
│                                              │
▼                                              ▼
LOAD_OPERATION                          COMPLETE_TOOLPATH
                                               │
                                               ▼
                                             IDLE


Any State

↓

FAULT

↓

SHUTDOWN
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
IDLE → LOAD_OPERATION
```

---

## LOAD_OPERATION

Purpose

Load the next JSONL operation.

Responsibilities

- Read Toolpath Queue
- Decode target X, Y, Z, and A positions
- Store current operation
- Increment execution pointer
- Verify operation exists

Exit

```
LOAD_OPERATION → PLAN_OPERATION
```

or

```
LOAD_OPERATION → COMPLETE_TOOLPATH
```

---

## PLAN_OPERATION

Purpose

Prepare the next manufacturing operation.

Current Responsibilities

- Decode JSON operation
- Select operation type
- Validate target coordinates
- Prepare the standard motion sequence

Future Responsibilities

- Thermal planning
- Process planning
- Tool verification
- Simulation lookup
- Adaptive planning

Exit

```
PLAN_OPERATION → EXECUTE_OPERATION
```

---

## EXECUTE_OPERATION

Purpose

Execute exactly one manufacturing operation.

ForgeBrain never directly commands LinuxCNC.

Instead

```
ForgeBrain

↓

Motion Coordinator

↓

Motor

↓

LinuxCNC Interface

↓

LinuxCNC

↓

HAL

↓

Mesa FPGA

↓

Servo Drive

↓

Motor
```

Every manufacturing operation follows the same deterministic safe-motion sequence.

```
START

↓

Retract Z

↓

Wait for LinuxCNC

↓

Verify Z Safe

↓

Retract X/Y

↓

Wait for LinuxCNC

↓

Verify X/Y Safe

↓

Rotate A

↓

Wait for LinuxCNC

↓

Verify Rotation

↓

Move X/Y

↓

Wait for LinuxCNC

↓

Verify X/Y

↓

Move Z

↓

Wait for LinuxCNC

↓

Verify Z

↓

Motion Complete
```

The next manufacturing stage cannot begin until the previous stage reports completion.

---

## VERIFY_OPERATION

Purpose

Verify successful completion.

Current Responsibilities

- Machine enabled
- LinuxCNC operational
- Motion completed

Future Responsibilities

- Sensor verification
- Force verification
- Thermal verification
- Geometry verification

Exit

```
VERIFY_OPERATION → ADAPT_OPERATION
```

or

```
VERIFY_OPERATION → FAULT
```

---

## ADAPT_OPERATION

Purpose

Provide a single abstraction layer between manufacturing operations.

This state intentionally exists to support future autonomous manufacturing.

Current Responsibilities

- No adaptive behaviour

Future Responsibilities

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
ADAPT_OPERATION → NEXT_OPERATION
```

---

## NEXT_OPERATION

Purpose

Determine whether additional manufacturing operations remain.

Exit

If operations remain

```
NEXT_OPERATION → LOAD_OPERATION
```

Otherwise

```
NEXT_OPERATION → COMPLETE_TOOLPATH
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

Move the Z axis to a known safe clearance height before any lateral or rotational movement.

Responsibilities

- Command Z axis retraction through LinuxCNC
- Monitor motion status

Transition

```
RETRACT_Z → VERIFY_Z_RETRACTED
```

---

### VERIFY_Z_RETRACTED

Purpose

Confirm the Z axis has reached the safe clearance position.

Responsibilities

- Verify LinuxCNC motion completion
- Verify commanded position achieved

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

Move X and Y axes to their safe clearance positions.

Responsibilities

- Command X/Y retraction through LinuxCNC
- Monitor motion status

Transition

```
RETRACT_XY → VERIFY_XY_RETRACTED
```

---

### VERIFY_XY_RETRACTED

Purpose

Confirm X/Y axes have reached the safe position.

Responsibilities

- Verify LinuxCNC motion completion
- Verify commanded position achieved

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
# Motor State Machine

Every machine axis owns an independent Motor object.

The Motor class is **not** an independent controller.

It exists solely as an abstraction of a single machine axis and executes commands delegated by the Motion Coordinator.

Each Motor maintains its own operational state.

```
                     UNINITIALIZED
                           │
                           ▼
                      DISABLED
                           │
                    Enable Axis
                           │
                           ▼
                        ENABLED
                     ┌─────┴─────┐
                     │           │
                     ▼           ▼
          CONSTANT VELOCITY   POSITION MOTION
             (Homing)          (Toolpath Move)
                     │           │
                     └─────┬─────┘
                           │
                 Motion Complete
                           │
                           ▼
                        ENABLED


Any Operational State

        │

        ▼

      FAULT

        │

   Fault Recovery

        │

        ▼

    DISABLED
```

## State Definitions

### UNINITIALIZED

The Motor object has been created but has not yet established communication with LinuxCNC.

Responsibilities

- No motion permitted
- Await initialization
- Verify communication with LinuxCNC

Transition

```
UNINITIALIZED → DISABLED
```

---

### DISABLED

The axis is intentionally disabled and available for future activation.

This is the normal safe state of an axis.

Responsibilities

- Reject motion commands
- Report disabled status
- Await enable request
- Maintain fault-free state

Transition

```
DISABLED → ENABLED
```

or

```
DISABLED → FAULT
```

---

### ENABLED

The axis is enabled and capable of accepting motion commands.

Responsibilities

- Accept commands from the Motion Coordinator
- Report current axis status
- Monitor LinuxCNC status

Transitions

```
ENABLED → CONSTANT VELOCITY
```

or

```
ENABLED → POSITION MOTION
```

or

```
ENABLED → FAULT
```

---

### CONSTANT VELOCITY

The axis is executing a continuous velocity command through LinuxCNC.

This state is primarily used during homing procedures.

Motion continues until an external event, such as a limit switch signal, requests termination.

The Motor object does not directly terminate motion. It reports the event and LinuxCNC remains responsible for stopping the commanded motion.

Transitions

```
CONSTANT VELOCITY → ENABLED
```

or

```
CONSTANT VELOCITY → FAULT
```

---

### POSITION MOTION

The axis is executing a position command through LinuxCNC.

LinuxCNC remains responsible for

- trajectory generation
- interpolation
- acceleration
- velocity limiting
- following error detection

The Motor object only monitors LinuxCNC status and determines when the commanded motion has completed.

Transitions

```
POSITION MOTION → ENABLED
```

or

```
POSITION MOTION → FAULT
```

---

### FAULT

The axis has encountered an abnormal condition preventing safe operation.

FAULT is a latched error condition and is separate from DISABLED.

Examples include

- LinuxCNC axis fault
- Following error
- Drive amplifier fault
- Communication failure
- Unexpected loss of axis state
- Invalid motion command

Responsibilities

- Reject all motion commands
- Preserve fault information
- Report fault state through telemetry
- Prevent automatic reactivation

Recovery requires an explicit reset procedure.

Transition

```
FAULT → DISABLED
```

after fault recovery has been completed.

---

The Motor state machine never performs trajectory generation or real-time control.

All motion execution is delegated to LinuxCNC, while the Motor object serves only as an abstraction of axis state, command routing, and fault reporting.

---

# LinuxCNC Interface

LinuxCNCInterface intentionally contains **no state machine**.

Its sole responsibility is translating ForgeBrain requests into LinuxCNC API calls and exposing LinuxCNC status back to the controller.

```
ForgeBrain

↓

Motion Coordinator

↓

Motor

↓

LinuxCNC Interface

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

The Motor object never commands hardware directly. Every motion request is delegated through the LinuxCNC Interface.

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
Motor
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

while each Motor object is responsible only for representing the operational state of a single axis and forwarding commands to LinuxCNC.

This separation of responsibilities guarantees deterministic execution, simplifies debugging, and provides a stable foundation for future sensor integration, adaptive process control, digital twins, FEM/MPM simulation, and autonomous forging algorithms without modifying the underlying machine control architecture.