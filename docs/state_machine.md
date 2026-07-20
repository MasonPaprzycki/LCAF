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
    │       ├── Motor X                 |
    │       ├── Motor Y                 |
    │       ├── Motor Z                 |
    │       └── Motor A                 |
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

Pull from Motion Coordinator which pulls motor states and diagnostics from linuxcnc 

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
│ RETRACT_XY                                │
│   ↓                                       │
│ VERIFY_XY_RETRACTED                       │
│   ↓                                       │
│ ROTATE_A                                  │
│   ↓                                       │
│ VERIFY_ROTATION                           │
│   ↓                                       │
│ MOVE_XY                                   │
│   ↓                                       │
│ VERIFY_XY_POSITION                        │
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

- Increment to the next step in the queue and read it to get the upcoming JSON 
operation.

- Store current operation 
- Decode JSON operation
- Select operation type
- Validate target coordinates
- Pass ToolpathOperation to MotionCoordinator for expansion into motion commands. 

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
                        READY
                     ┌─────┴─────┐
                     │   MOVING  │
                     ▼           ▼
                   HOMING      MOVING
              (constant vel)  (Toolpath sub operation)
                     │           │
                     └─────┬─────┘
                           │
                        COMPLETE
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

The Motor object has been created but has not yet established communication with LinuxCNC.

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

The axis is executing a continuous velocity command through LinuxCNC to home the motor. 

Motion continues until a limit switch signal. 

The Motor object does not directly terminate motion. It reports the event and LinuxCNC remains responsible for stopping the commanded motion.

Transitions

```
HOMING → READY
```

or

```
HOMING → FAULT
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

The Motor object only monitors LinuxCNC status and determines when the commanded motion has completed.

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