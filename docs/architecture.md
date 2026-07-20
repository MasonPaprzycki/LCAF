# LCAF Software Architecture

## Overview

        main.py (intitializes and starts controller)
           |
           |
    control/controller.py (telemetry & state machine heartbeat)--------
           |                                                          |
           |                                                          |
telemetry/telemetry.py --> control/forge_brain.py                     |
                            && adaption/adaptive_planner.py           |
(publishes telemetry)                                                 |
                                                                      |
             adaption/adaptive_planner.py --------------------> forge_brain.py  
                (provides updated tool path queue)                |
                                                                  |
                                                                  |
                                                        control/motion_coordinator.py
                                                                  |
                                                                  |
                                                            control/axis.py
                                                                  |
                                                                  |
                                                        control/linuxcnc_interface.py
                              
--> means publishes data to 

utils/toolpath.py contains utils for toolpath and operation path classes used by 
motion_coordinator.py forge_brain.py and will be used by adaptive planner.py

adaption/simulation currently contains MPM and FEM simulations used to represent this forge. It is using JAX-FEM for FEM and Genesis for MPM 
        

# main.py
intiializes and starts up the entire system 

# controller.py 
Controls heart beat; initializes publishes and sets up subscribers to telemetry
Advances the control loop in forge_brain.py

# telemetry.py 
    - Collects sensor and telemetry data 
    - Publish's telemetry and sensor data to a central channel that all other subsystems can subscribe to 

# forge_brain.py 
 
Forge brain is the highest level abstraction of the state machine


Responsibilities: 

    - Owns the entire machine state 
    - Executes tool path 
    - Fault handeling 


    - Manages Execution queue 
        - Ensures we execute a tool path step by step

        - Leaves one layer of abstraction in the queue state ADAPT_OPERATION that pulls an updated queue from adaptive_planner which allows for deterministic adaptive toolpathing 
 

# motion_coordinator.py
    - Within every tool path we need to retract Z, retract X and Y, apply the rotation A, apply X and Y, and finally apply Z. MotionCoordinator makes sure we do this 

Responsibilities: 
    - Coordinating multi-axis moves
    - Homing
    - Emergency stop requests
    - Polling axis state
    - Does not perform trajectory generation

Note: LinuxCNC performs all trajectory planning

# Axis

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
