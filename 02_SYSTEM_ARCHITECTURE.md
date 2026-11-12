# 02 --- System Architecture

## 1. Architectural principle

Use a layered ROS 2 architecture. Each layer publishes structured data
or exposes explicit services/actions. The AI supervisor coordinates
high-level work but does not replace deterministic perception, motion
planning, robot drivers or safety logic.

## 2. High-level architecture

``` mermaid
flowchart TB
    OP[Operator / Task Request] --> UI[Dashboard and Task Interface]
    UI --> AI[AI Supervisor]
    AI --> ORCH[Task Orchestrator]

    CAM[Logitech C270] --> PER[Vision and AprilTag Pose]
    CAL[Saved Calibration] --> PER
    PER --> TF[Camera-to-Base Transform]
    TF --> SCENE[World / Planning Scene]

    ROBOT[xArm 5 Driver] <--> STATE[Robot State and Joint Feedback]
    TOOL[AG1010 Interface] <--> STATE

    SCENE --> GRASP[Grasp Candidate Generator]
    STATE --> KIN[Kinematics and Reachability Evaluation]
    GRASP --> KIN
    KIN --> PLAN[MoveIt 2 Motion Planning]
    PLAN --> SIM[Gazebo Digital Twin]
    SIM --> SCORE[Candidate Validation and Ranking]

    ORCH --> PER
    ORCH --> GRASP
    ORCH --> PLAN
    ORCH --> SIM
    SCORE --> GATE[Deterministic Execution Gate]
    STATE --> GATE
    GATE --> EXEC[Execution Manager]
    EXEC --> ROBOT
    EXEC --> TOOL
    EXEC --> VERIFY[Grasp and Placement Verification]
    VERIFY --> LOG[Experiment Logger]
    VERIFY --> ORCH
    GATE --> SAFETY[Safety Monitor / Stop Conditions]
    SAFETY --> EXEC
    UI <--> STATE
    UI <--> LOG
```

## 3. Subsystems

### A. Hardware and drivers

-   xArm 5 connection and status.
-   Joint-state and robot-state monitoring.
-   AG1010 command and feedback interface.
-   Logitech C270 camera driver.
-   Emergency stop remains available independently of the application.

### B. Vision and localisation

-   Image acquisition.
-   AprilTag detection.
-   Cube pose estimation from the known marker layout.
-   Pose quality and uncertainty reporting.
-   Transform from camera frame to robot base frame.

### C. Scene and calibration

-   Static table and robot-base geometry.
-   Saved camera intrinsics.
-   Saved camera-to-base extrinsics.
-   Tool centre point (TCP) and gripper geometry.
-   Calibration version, timestamp and validation status.

### D. Grasp planning

-   Generate a small, defined family of grasps for the known cube.
-   Generate pre-grasp and retreat poses.
-   Respect cube geometry, AG1010 opening and approach direction.
-   Avoid unconstrained arbitrary 6-DoF grasp generation in Version 1.

### E. Kinematics and motion planning

-   Inverse kinematics.
-   Joint-limit checks.
-   Reachability checks.
-   Singularity/manipulability assessment where meaningful for the 5-DoF
    arm.
-   Collision checking.
-   Full trajectory planning with MoveIt 2.

### F. Digital twin

-   Robot and tool model.
-   Table and cube collision geometry.
-   Scene update from perception.
-   Candidate trajectory execution in simulation.
-   Optional contact/grasp/lift simulation, subject to validated gripper
    and physics models.

### G. Candidate evaluator

-   Reject candidates that violate hard constraints.
-   Score remaining candidates using documented weights.
-   Store each candidate's score components and rejection reason.

### H. Execution and verification

-   Recheck robot and scene state before movement.
-   Execute approved trajectory through the supported robot interface.
-   Verify grasp, lift and placement.
-   Stop safely if feedback is missing or contradictory.

### I. AI supervisor

-   Convert a high-level task into a structured task request.
-   Request perception, planning, simulation and verification
    operations.
-   Explain candidate results and failure summaries.
-   Never send raw joint commands or bypass execution gates.

## 4. Core data flow

1.  `sensor_msgs/Image` from camera.
2.  Detected marker observations and estimated cube pose.
3.  `geometry_msgs/PoseWithCovarianceStamped` or an equivalent typed
    pose message.
4.  Transform into the robot base frame using a validated transform.
5.  Candidate grasp records with pose, approach direction and
    constraints.
6.  IK and trajectory results with explicit success/failure status.
7.  Simulation results and candidate score.
8.  Execution request with selected candidate ID and plan ID.
9.  Robot/gripper feedback and verification result.
10. Experiment record.

Exact custom message definitions should be created only after the first
end-to-end interface is agreed. Avoid passing unstructured dictionaries
between critical ROS nodes.

## 5. State machine

``` mermaid
stateDiagram-v2
    [*] --> INITIALISING
    INITIALISING --> READY: all required systems healthy
    INITIALISING --> FAULT: startup check failed
    READY --> PERCEIVING: task accepted
    PERCEIVING --> PLANNING: valid cube pose
    PERCEIVING --> ABORTED: pose invalid or stale
    PLANNING --> SIMULATING: feasible candidates found
    PLANNING --> ABORTED: no feasible candidate
    SIMULATING --> PRE_EXECUTION_CHECK: candidate validated
    SIMULATING --> ABORTED: no valid simulated candidate
    PRE_EXECUTION_CHECK --> EXECUTING: all gates pass
    PRE_EXECUTION_CHECK --> ABORTED: gate fails
    EXECUTING --> VERIFYING: motion completed
    EXECUTING --> FAULT: robot fault / safety stop
    VERIFYING --> COMPLETE: task verified
    VERIFYING --> RECOVERY: recoverable failure
    RECOVERY --> PERCEIVING: retry permitted
    RECOVERY --> ABORTED: retry limit reached
    COMPLETE --> READY
    ABORTED --> READY
    FAULT --> [*]
```

## 6. Architectural constraints

-   The robot driver is the only component that communicates with the
    robot controller.
-   MoveIt 2 or a validated motion-planning component produces
    trajectories; the AI layer does not.
-   A deterministic execution gate validates every physical motion
    request.
-   Calibration data must be loaded with frame names and version
    metadata.
-   Simulation and physical execution must use consistent robot, tool
    and workspace geometry.
-   Every task has a unique ID; every candidate and trajectory is
    traceable to that task.
