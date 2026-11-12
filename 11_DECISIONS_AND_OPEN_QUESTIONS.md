# 11 --- Decisions and Open Engineering Questions

## 1. Confirmed Version 1 decisions

  -----------------------------------------------------------------------
  Topic                               Decision
  ----------------------------------- -----------------------------------
  Robot                               UFACTORY xArm 5

  End effector                        UFACTORY AG1010

  Camera                              Logitech C270

  Camera mounting                     Fixed

  Calibration frequency               One-time initial setup; save and
                                      reload

  Camera moved                        Treat calibration as suspect;
                                      manual recalibration

  Calibration target                  No mandatory ChArUco board

  Object                              One known 50 mm cube

  Object marking                      AprilTag markers

  Workspace                           Fixed tabletop

  Object motion                       Static during each pick attempt

  ROS environment                     ROS 2 Jazzy

  Primary OS                          Ubuntu 24.04

  Motion planning                     MoveIt 2

  Main simulation                     Gazebo

  AI role                             High-level supervisor/orchestrator,
                                      not direct joint controller

  Initial scope                       Single-object pick-and-place
  -----------------------------------------------------------------------

## 2. Engineering items to verify before implementation

### Robot

-   Exact xArm 5 controller/firmware and supported driver version.
-   Supported official ROS 2 Jazzy branch/package instructions.
-   Joint limits and speed/acceleration limits for the actual robot.
-   Correct robot description and MoveIt configuration.

### AG1010

-   Official ROS 2 support and command interface.
-   URDF/CAD availability and licence.
-   Finger travel, opening range and actuation convention.
-   Tool mounting transform and TCP.
-   Feedback available for grasp verification.
-   Whether reliable contact simulation is practical.

### Camera and calibration

-   C270 resolution, frame rate and configurable controls on Ubuntu.
-   Marker family and marker size.
-   Calibration target/method for intrinsics.
-   Practical extrinsic calibration procedure using known robot-frame
    references.
-   Measured pose accuracy across the intended workspace.
-   Calibration invalidation procedure after camera movement.

### Simulation

-   Gazebo release and ROS 2 Jazzy integration.
-   xArm 5 simulation model availability and quality.
-   AG1010 physics/contact model quality.
-   Whether MoveIt planning scene and Gazebo collision geometry remain
    consistent.

### System

-   Actual tabletop dimensions and robot mounting geometry.
-   Safe workspace and placement target.
-   Network topology and robot IP.
-   Operator stop and commissioning procedure.
-   Storage location and retention policy for experiment data.

## 3. Calibration warning

Do not proceed to autonomous physical execution until camera-to-base
extrinsics have been measured and validated. Intrinsic camera
calibration alone is not enough to convert a camera observation into
robot-base coordinates.

## 4. Model and safety warning

A URDF model and a successful Gazebo run do not prove that the real
robot will behave identically. Validate robot limits, TCP, tool
geometry, collision geometry and stop behaviour on the actual hardware
under supervised conditions.

## 5. Change control

When a scope decision changes: 1. Update this file. 2. Update
`01_PROJECT_SPECIFICATION.md`. 3. Update the affected
architecture/implementation document. 4. Record the reason and date in
the project decision history. 5. Ensure code and configuration match the
documentation.
