# 01 --- Project Specification

## 1. Working title

**Development of an AI-Assisted, Kinematics-Aware Digital Twin Framework
for Autonomous Object Manipulation Using UFACTORY xArm 5**

## 2. Problem statement

A robot manipulation task requires more than detecting an object. The
system must estimate its pose in robot coordinates, determine whether
the robot and gripper can reach it, generate a collision-aware motion,
assess the proposed action, execute it safely, and determine whether the
manipulation succeeded.

This project develops an integrated workflow for a known object and a
constrained tabletop environment. Its research emphasis is the
combination of robot-specific kinematic evaluation,
simulation-in-the-loop candidate validation, and feedback-based physical
execution.

## 3. Primary objective

Develop and experimentally evaluate a system that autonomously picks up
and places one known AprilTag-marked cube using a UFACTORY xArm 5 and
AG1010 gripper, with perception from a fixed Logitech C270 camera and
simulation-based validation before physical execution.

## 4. Specific objectives

1.  Integrate the xArm 5 and AG1010 with ROS 2.
2.  Acquire images from a fixed Logitech C270.
3.  Detect AprilTags on a known 50 mm cube and estimate cube pose.
4.  Transform the estimated pose into the xArm base frame.
5.  Generate a constrained set of grasp and pre-grasp candidates.
6.  Evaluate candidates for IK feasibility, joint limits, reachability,
    collision clearance and gripper compatibility.
7.  Plan complete approach, grasp, lift, transfer, place and retreat
    trajectories.
8.  Validate candidate trajectories and gripper/object interaction in
    Gazebo.
9.  Rank feasible candidates using transparent numerical criteria.
10. Execute the selected candidate on the physical robot after safety
    preflight.
11. Verify grasp and placement outcomes using available camera and
    gripper feedback.
12. Record repeatable experiments and compare simulation predictions
    with physical results.

## 5. Operating assumptions

-   The robot is mounted securely and its base frame is known.
-   The tabletop is fixed and its geometry is measured.
-   The camera is mounted rigidly and is not moved during normal Version
    1 operation.
-   The cube is a known 50 mm object with known marker layout and
    dimensions.
-   The cube is placed within a defined workspace.
-   The operator can access the robot's emergency stop and supervise
    initial physical trials.
-   Lighting is adequate for the camera and markers.
-   The AG1010 model, grasp width, TCP and control interface will be
    confirmed against the actual hardware.

## 6. Version 1 task definition

**Input:** A request to pick the detected cube and place it at a
configured target location on the tabletop.

**Output:** A completed pick-and-place attempt with a recorded outcome:
success, recoverable failure, aborted, or safety stop.

**Normal sequence:** 1. Initialise robot, gripper, camera and simulation
services. 2. Load saved calibration and validate its metadata. 3.
Capture an image and detect the cube. 4. Estimate cube pose and
uncertainty. 5. Transform cube pose into the robot base frame. 6.
Generate candidate grasps. 7. Evaluate candidates and plan trajectories.
8. Simulate and rank valid candidates. 9. Recheck robot state and scene
freshness. 10. Execute the selected trajectory. 11. Verify lift and
placement. 12. Log result and return to a known safe state.

## 7. Acceptance criteria

The project is not considered complete merely because the robot moves.
The following must be demonstrated and measured:

-   Camera images are acquired reliably at the selected resolution/frame
    rate.
-   The known cube is detected under the defined test lighting and
    workspace conditions.
-   Pose estimates can be transformed into the robot base frame with
    measured error.
-   Invalid or unreachable candidates are rejected before execution.
-   The planner produces collision-checked trajectories for the defined
    workspace.
-   Gazebo can load the robot, tool, table and cube scene.
-   The selected candidate can be executed on the physical robot under
    supervised conditions.
-   Grasp and placement outcomes are classified and logged.
-   Experiments report success rate, pose error, planning performance
    and execution time.
-   A safety stop or invalid robot state prevents motion.

Numerical performance thresholds must be set after baseline measurements
and hardware limits are known. Do not invent precision or success-rate
guarantees before testing.

## 8. Scope expansion after Version 1

Possible later extensions: - Multiple known objects. - Unknown object
dimensions and object classification. - Clutter and obstacle-rich
scenes. - Moving objects. - Automatic camera movement detection and
recalibration. - Additional sensors such as depth cameras. - More
general grasp families and task planning.

These are future work, not Version 1 deliverables.
