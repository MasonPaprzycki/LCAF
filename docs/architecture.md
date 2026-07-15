# LCAF Software Architecture

## Overview

The Low Cost Agility Forge (LCAF) software stack is divided into hierarchical control layers.

LinuxCNC is responsible for deterministic real-time machine control, this eliminates the need for custom programmed motor drivers and makes this software scaleable and maintainable with other systems. 

ForgeBrain is responsible for high-level manufacturing execution.

The architecture intentionally separates machine control from manufacturing intelligence.


In the diagram below individual files have are denoted with .filetype
Everything else talks about a purpose a file fills in a high level description 

-------------------------------------------------------------------------------
-------------------------------------------------------------------------------

                    (Toolpath execution queue).jsonl  ◀─────────────┐
                           │                                        |
                           ▼                                        |
                    ForgeBrain.py  ─────────────────────▶ Adaptive toolpathing 
                                                            abstraction layer
          (Supervisory Manufacturing Controller)
                           │
      ┌────────────────────┼────────────────────┐
      │                    │                    │
      ▼                    ▼                    ▼
 MotionCoordinator     SensorManager      Telemetry
      │
      ▼
    Motor.py
      │
      ▼
LinuxCNCInterface.py
      │
      ▼
 LinuxCNC Python API
      │
      ▼
 HAL Pins / INI / Machine Config
      │
      ▼
      Mesa FPGA
      │
      ▼
 Servo Drives
      │
      ▼
  Mechanical Forge

--------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------
# forge_brain.py 
 
Forge brain is the highest level abstraction

It is essentially a giant state machine that ensures the machine listens to our instructions

Responsibilities: 

    - Owns the entire machine state 
    - Executes tool path 
    - Collects sensor and telemetry data 
    - Fault handeling 
    - Publish's telemetry and sensor data to a central channel that all other subsystems can subscribe to 

    - Manages Execution queue 
        - Ensures we execute a tool path step by step

        - Leaves one layer of abstraction in the queue state that allows for the collection and processing of sensor data and telemetry. This layer will execute reheating logic and adaptive tool pathing. So once it hits this state if it needs to reheat the system will wait before the next tool path operation and reheat. In adaptive tool pathing there will be another logistical chain that decides how and if the system should change the next or any proceding tool paths. 
    
    - Within every tool path we need to retract Z, retract X and Y, apply the rotation A, apply X and Y, and finally apply Z, 

# MotionCoordinator

Responsibilities: 

    - Coordinating multi-axis moves
    - Homing
    - Emergency stop requests
    - Polling motor state
    - Does not perform trajectory generation

Note: LinuxCNC performs all trajectory planning

# Motor

Represents a single machine axis.

Responsibilities: 

    - Axis abstraction
    - Axis status
    - Axis command interface
    - Axis state reporting

# LinuxCNCInterface

Thin abstraction over LinuxCNC.

Responsibilities: 

    - Machine status
    - Machine commands
    - MDI
    - HAL
    - Error channel

Note: No supervisory logic.


# LinuxCNC

Responsibilities: 

    - Motion planning
    - Trajectory generation
    - Realtime interpolation
    - Servo control
    - Safety
    - Homing
    - Limit handling
    - Following error detection

Note: ForgeBrain never replaces LinuxCNC functionality.


# Planned software modules

Already implemented: 
    - forge_brain.py (supervisory manufacturing controller)

Currently embedded in forge_brain low priority but might deserve own module: 
    - MotionCoordinator (Multi-Axis Coordination)
    - SensorManager (Sensor polling)
    - Telemetry (Publish-subscribe messaging)
    - ToolPathQueue (JSONL execution queue)

As of yet only partially completed: 
    - linuxcnc_interface.py (LinuxCNC abstraction)
    - motor.py (Axis abstraction)

Planned: 
    - ProcessModel (billet state estimation )
    - SimulationInterface (FEM/MPM integration with the simulation stuff currently stored in the LowCostAgilityForge repository)
    - GeometryEngine (Fast geometric prediction)



