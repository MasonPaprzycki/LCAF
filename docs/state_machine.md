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

Update Motion Coordinator

↓

Update Motor States

↓

Update Sensor States

↓

Update ForgeBrain State Machine

↓

Publish Telemetry

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

# ForgeBrain State Definitions

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

# Nested Motor State Machine

Every machine axis owns an independent Motor object.

The Motor class is **not** an independent controller.

It only exists because ForgeBrain delegated motion to the Motion Coordinator.

Each Motor maintains its own state.

```
UNKNOWN

↓

DISABLED

↓

READY

↓

MOVING

↓

LinuxCNC Executing Motion

↓

Poll LinuxCNC

↓

Motion Complete?

├─────────────── No ───────────────┐
│                                  │
▼                                  │
MOVING                             │
│                                  │
└──────────────────────────────────┘

Yes

↓

COMPLETE

↓

READY
```

Exceptional transitions

```
Any State

↓

ESTOP
```

or

```
Any State

↓

FAULT
```

Motor state transitions are driven entirely from LinuxCNC status.

The Motor class never generates trajectories and never performs real-time control.

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
- homing
- following error detection
- deterministic real-time execution

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

Information flows upward through polling.

Commands flow downward through delegation.

Every heartbeat begins in ForgeBrain.

Every command originates in ForgeBrain.

Every subsystem is synchronized by the same heartbeat.

The Motion Coordinator is responsible for expanding every toolpath operation into the same deterministic safe-motion sequence:

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

Only the target coordinates differ between operations; the execution sequence is invariant. This architecture guarantees deterministic execution, simplifies debugging, and provides a stable foundation for future sensor integration, adaptive process control, digital twins, FEM/MPM simulation, and autonomous forging algorithms without modifying the underlying machine control architecture.