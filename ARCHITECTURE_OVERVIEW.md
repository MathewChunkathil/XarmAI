# Architecture Overview

``` mermaid
flowchart TB
    OP[Operator / Task UI] --> AI[AI Supervisor]
    AI --> ORCH[Task Orchestrator]
    CAM[Logitech C270] --> PER[AprilTag Detection and Cube Pose]
    CAL[Saved Intrinsic + Extrinsic Calibration] --> PER
    PER --> TF[Pose in xArm Base Frame]
    TF --> SCENE[Planning Scene]
    SCENE --> GRASP[Grasp Candidate Generator]
    GRASP --> KIN[IK / Joint Limits / Reachability]
    KIN --> MOVEIT[MoveIt 2 Trajectory Planning]
    MOVEIT --> GAZEBO[Gazebo Digital Twin Validation]
    GAZEBO --> RANK[Candidate Filtering and Ranking]
    RANK --> GATE[Deterministic Execution Gate]
    ROBOT[xArm 5 Driver and State] --> GATE
    GATE --> EXEC[Execution Manager]
    EXEC --> ROBOT
    EXEC --> TOOL[AG1010 Gripper]
    EXEC --> VERIFY[Grasp and Placement Verification]
    VERIFY --> LOG[Experiment Logger]
    VERIFY --> ORCH
    SAFETY[Robot Native Safety + Application Monitor] --> GATE
    UI[Dashboard] <--> LOG
    UI <--> ROBOT
```

## Operating sequence

1.  Load and validate saved calibration.
2.  Acquire image and detect the known cube.
3.  Transform cube pose into the xArm base frame.
4.  Generate grasp candidates.
5.  Evaluate kinematics and plan complete trajectories.
6.  Validate candidates in Gazebo.
7.  Rank feasible candidates.
8.  Pass selected plan through deterministic safety/preflight gate.
9.  Execute on the physical xArm 5 and AG1010.
10. Verify grasp and placement; log results or stop/recover.

## Boundary

The AI supervisor may interpret tasks, coordinate tools and explain
results. It must not issue raw joint commands, bypass MoveIt validation,
or override robot-native safety functions.
