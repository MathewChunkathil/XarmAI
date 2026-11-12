# 04 --- Robot and Tool Integration

## 1. Hardware

-   Robot: UFACTORY xArm 5.
-   End effector: UFACTORY AG1010 parallel-jaw gripper.
-   Camera: Logitech C270.
-   Work surface: fixed tabletop.
-   Object: known 50 mm printed cube with AprilTag markers.

## 2. Vendor integration first

Use the official UFACTORY ROS 2 driver and robot description as the
starting point. Verify the repository's current branch, supported ROS
distribution, xArm 5 launch arguments, MoveIt configuration and Gazebo
support. Do not assume that a launch command for another xArm model
applies unchanged to xArm 5.

Initial hardware integration checklist: - \[ \] Confirm robot model and
serial/controller firmware. - \[ \] Confirm network connection and
static IP/network configuration. - \[ \] Launch official ROS 2 driver. -
\[ \] Read joint states and robot status. - \[ \] Confirm joint ordering
and units. - \[ \] Confirm motion enable and mode requirements. - \[ \]
Confirm speed/acceleration limits and reduced-speed test settings. - \[
\] Confirm stop and emergency-stop behaviour. - \[ \] Confirm AG1010
supported control method and feedback. - \[ \] Confirm tool geometry,
TCP and mounting transform. - \[ \] Confirm MoveIt robot model matches
the physical arm.

## 3. AG1010 model validation

Do not assume a generic parallel gripper model is an accurate AG1010
model.

Obtain or construct: - Visual geometry. - Collision geometry. - Mounting
transform. - TCP definition. - Finger travel and opening limits. -
Actuation mapping and command semantics. - Feedback/state
interpretation. - Mass and centre of mass if available. - Contact
geometry suitable for simulation.

Validate the model against the physical gripper: 1. Measure closed and
maximum open positions. 2. Measure finger motion direction and symmetry.
3. Confirm the commanded position maps to actual opening. 4.
Measure/confirm TCP relative to the robot flange. 5. Check that the
MoveIt planning group and Gazebo model use the same joint conventions.

If an official AG1010 model is unavailable, create a project-owned
description with documented measurements and mark approximations. Keep
the physical gripper driver separate from its URDF model.

## 4. Five-degree-of-freedom implications

The xArm 5 is a 5-DoF arm. Do not request arbitrary six-dimensional
end-effector poses as if it were a 6-DoF arm. The planner must account
for the actual kinematic constraints.

For the known cube, begin with a constrained grasp family: - Top-down
approach. - Gripper closing direction aligned with a selected cube
axis. - Limited yaw candidates around the vertical axis. - Fixed or
narrowly bounded wrist orientation. - A defined pre-grasp offset and
lift direction.

The exact feasible orientations and workspace must be established using
the robot model, MoveIt and physical tests.

## 5. Robot state contract

The application should expose, at minimum: - Connection state. - Robot
mode and error state. - Joint positions and velocities if available. -
Motion state. - Current tool/gripper state. - Active task ID. - Active
trajectory ID. - Safety gate status.

Missing, stale or invalid state must block new physical motion.

## 6. Physical commissioning sequence

1.  Verify emergency stop and safe operating area.
2.  Run driver connection and read-only state checks.
3.  Test gripper independently.
4.  Test low-speed single-joint movements under supervision.
5.  Test a known safe Cartesian/joint target.
6.  Test MoveIt plan-only mode.
7.  Test a simulated trajectory.
8.  Test a short physical trajectory at reduced speed.
9.  Test gripper open/close at a safe location.
10. Only then begin supervised pick-and-place commissioning.
