# LCAF Hierarchical Finite State Machine (HFSM)

## Overview

The Low Cost Agility Forge (LCAF) control software is implemented as a **Hierarchical Finite State Machine (HFSM)**.

Unlike a traditional flat finite state machine, only **ForgeBrain** owns execution.

Every other component in the software stack exists only because ForgeBrain delegates work to it during each heartbeat.

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

Only ForgeBrain owns execution timing.

No subsystem executes independently.

No module owns its own execution loop.

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

Every subsystem reads the same LinuxCNC snapshot.

This guarantees deterministic behaviour across the entire controller.

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
│                      Motion Coordinator                                     │
│                                                                             │
│        Determines the required machine motion for the current operation.    │
│                                                                             │
│                                                                             │
│     ┌──────────────────────────────────────────────────────────────┐        │
│     │                                                              │        │
│     ▼                                                              ▼        │
│  Translate XY                                               Rotate A         │
│     │                                                              │        │
│     ▼                                                              ▼        │
│  Verify XY                                                Verify Rotation    │
│     │                                                              │        │
│     └──────────────────────┬───────────────────────────────────────┘        │
│                            ▼                                                │
│                        Execute Strike                                       │
│                            │                                                │
│                            ▼                                                │
│                        Verify Strike                                        │
│                            │                                                │
│                            ▼                                                │
│                         Retract Z                                           │
│                            │                                                │
│                            ▼                                                │
│                       Verify Retraction                                     │
│                            │                                                │
│                            ▼                                                │
│                        Motion Complete                                      │
│                                                                             │
│                                                                             │
│        Every motion request is delegated to one or more Motor objects.      │
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
- Prepare motion sequence

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

The nested motion state machine executes as follows.

```
START

↓

Move XY

↓

Wait for LinuxCNC

↓

Verify XY

↓

Rotate A

↓

Wait for LinuxCNC

↓

Verify Rotation

↓

Execute Strike

↓

Wait for LinuxCNC

↓

Verify Strike

↓

Retract Z

↓

Wait for LinuxCNC

↓

Verify Retraction

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

This architecture guarantees deterministic execution, simplifies debugging, and provides a stable foundation for future sensor integration, adaptive process control, digital twins, FEM/MPM simulation, and autonomous forging algorithms without modifying the underlying machine control architecture.