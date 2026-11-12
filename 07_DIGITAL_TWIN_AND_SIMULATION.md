# 07 --- Digital Twin and Simulation

## 1. Purpose

Use Gazebo as the primary automated simulation environment for the
Version 1 workflow. The digital twin should represent the robot, tool,
tabletop and cube closely enough to test planning, collision constraints
and task sequencing.

UFACTORY Studio may be used as a supplementary vendor tool for
inspection/configuration or manual demonstrations. Do not assume its
simulation features expose a programmable API until that has been
verified.

## 2. Required simulated entities

-   xArm 5 robot model.
-   AG1010 gripper model and actuation.
-   Table/workspace.
-   50 mm cube with collision geometry.
-   Robot base and world frames.
-   Camera frame and scene object pose.
-   Relevant collision geometry.

## 3. Model fidelity levels

### Level 1 --- Planning geometry

Visual and collision models, joint limits, kinematics and TF are correct
enough for RViz/MoveIt planning.

### Level 2 --- Motion simulation

Robot joints follow trajectories in Gazebo and scene collision
constraints are represented.

### Level 3 --- Manipulation/contact simulation

Gripper fingers close on the cube, contact is modelled, and the cube can
be lifted and transported.

Level 3 is desirable but must not be claimed until contact behaviour is
validated. A visually animated gripper is not proof of realistic grasp
physics.

## 4. Synchronisation

The system must distinguish: - **Perception scene:** latest measured
cube pose. - **Planning scene:** geometry used by MoveIt. - **Gazebo
scene:** simulated robot and object state. - **Physical scene:** actual
robot and object state.

Each scene update should have a timestamp/version. Before execution,
ensure the plan was generated against the latest accepted scene and
robot state.

For physical-to-twin mode, use measured joint states to update the
virtual robot. For simulation mode, the virtual robot is driven by
planned trajectories. Avoid two controllers commanding the same
simulated joints simultaneously.

## 5. Simulation-in-the-loop process

1.  Load robot/tool/workspace models.
2.  Set cube pose from perception.
3.  Generate candidate grasp and trajectory.
4.  Reject invalid MoveIt plans.
5.  Execute remaining candidates in simulation.
6.  Check collisions and motion completion.
7.  If contact simulation is available, test close/lift/transport/place.
8.  Record simulated result and diagnostics.
9.  Rank candidates.
10. Send only the selected candidate to the physical execution gate.

## 6. Simulation limitations

-   Physics parameters may not match real friction, compliance and
    contact.
-   Gripper contact may be difficult to model accurately.
-   Cable, mounting and environmental details may be omitted.
-   Collision-free in simulation does not guarantee collision-free in
    the real environment.
-   A simulated successful grasp does not guarantee a physical
    successful grasp.

Therefore, physical trials remain necessary and must be supervised.

## 7. Simulation acceptance tests

-   Robot model loads with correct joint count and limits.
-   AG1010 geometry and motion are represented.
-   Table and cube collision geometry are present.
-   MoveIt and Gazebo agree on robot frame conventions.
-   A planned trajectory can be executed in simulation.
-   Scene updates move the cube to the expected pose.
-   Collision cases are detected.
-   Simulation outcomes and plan IDs are logged.
-   Any unsupported contact behaviour is documented rather than hidden.
