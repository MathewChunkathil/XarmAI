# xArm 5 AI-Assisted Autonomous Manipulation

**Project documentation baseline --- Version 1**

This repository defines the scope, architecture, implementation plan,
interfaces, validation strategy, and development sequence for an
AI-assisted manipulation system built around a UFACTORY xArm 5 and
AG1010 gripper.

## 1. Project in one paragraph

The system uses a fixed Logitech C270 camera to detect and estimate the
pose of one known 50 mm AprilTag-marked cube on a fixed tabletop. It
transforms the cube pose into the xArm base coordinate frame, generates
feasible grasp candidates, evaluates them using robot-specific
kinematics and motion-planning constraints, validates candidate motions
in a Gazebo digital twin, ranks feasible candidates, and executes the
selected plan on the physical xArm 5. The system then verifies the grasp
and placement, records the trial, and performs bounded recovery or stops
safely when the outcome is uncertain. An AI supervisory component may
interpret high-level tasks and coordinate software tools, but it does
not directly command joint motors or bypass deterministic safety checks.

## 2. Version 1 boundaries

### Included

-   One xArm 5 robot and one AG1010 gripper.
-   One fixed Logitech C270 camera.
-   One known 50 mm cube with AprilTag markers.
-   Fixed tabletop workspace and known table geometry.
-   Fixed camera mounting; calibration performed during setup and saved.
-   Detection and pose estimation for the known cube.
-   Camera-to-robot coordinate transformation.
-   Grasp candidate generation for the known cube.
-   IK, joint-limit, reachability, collision and trajectory evaluation.
-   Gazebo simulation and digital-twin scene synchronisation.
-   Candidate ranking using explicit numerical criteria.
-   Physical execution after deterministic preflight checks.
-   Grasp/placement verification, trial logging and bounded recovery.
-   Operator dashboard or equivalent monitoring interface.

### Explicitly excluded from Version 1

-   Multiple objects or object classification.
-   Unknown object geometry or arbitrary household objects.
-   Cluttered or changing environments.
-   Moving objects or conveyor tracking.
-   Automatic camera relocation or automatic recalibration.
-   General-purpose 6-DoF grasping.
-   LLM-generated joint commands or direct LLM control of actuators.
-   Unattended operation without an operator-accessible emergency stop.

## 3. Important coordinate and calibration distinction

Camera intrinsic calibration estimates the camera's internal projection
parameters. Camera-to-robot extrinsic calibration estimates the rigid
transform between camera and robot base coordinates. These are different
calibration problems. Both required parameter sets must be established,
saved, versioned and validated. A printed cube alone does not
automatically establish the camera-to-robot transform unless a known
geometric reference or known robot poses are used.

Version 1 does not require a ChArUco board. The final practical
calibration target/method must be selected and tested during the
calibration work package. Do not fabricate or assume an extrinsic
transform.

## 4. Documentation map

  ---------------------------------------------------------------------------
  File                                    Purpose
  --------------------------------------- -----------------------------------
  `01_PROJECT_SPECIFICATION.md`           Objectives, exact scope, acceptance
                                          criteria

  `02_SYSTEM_ARCHITECTURE.md`             Subsystems, architecture diagram,
                                          data flow

  `03_SOFTWARE_AND_REPOSITORY.md`         Software stack, package layout,
                                          build conventions

  `04_ROBOT_AND_TOOL_INTEGRATION.md`      xArm 5 and AG1010 integration
                                          requirements

  `05_VISION_AND_CALIBRATION.md`          C270, AprilTag, pose estimation and
                                          calibration

  `06_GRASP_AND_MOTION_PLANNING.md`       Candidate generation, IK, planning
                                          and ranking

  `07_DIGITAL_TWIN_AND_SIMULATION.md`     Gazebo scene, synchronisation and
                                          simulation tests

  `08_EXECUTION_SAFETY_AND_RECOVERY.md`   Physical execution, interlocks,
                                          verification and recovery

  `09_EXPERIMENTS_AND_METRICS.md`         Research evaluation and trial
                                          protocol

  `10_DEVELOPMENT_ROADMAP.md`             Phased implementation plan

  `11_DECISIONS_AND_OPEN_QUESTIONS.md`    Confirmed decisions and unresolved
                                          engineering checks
  ---------------------------------------------------------------------------

## 5. Engineering rules

1.  Reuse and extend official UFACTORY ROS 2 packages wherever
    practical. Do not recreate the vendor driver or robot description
    without a documented reason.
2.  Keep perception, planning, simulation and physical execution behind
    clear interfaces.
3.  Never execute a trajectory merely because an AI component proposed
    it.
4.  Physical motion requires fresh robot state, a valid plan, collision
    checks, workspace checks and operator-accessible stop controls.
5.  Simulation is a validation aid, not a substitute for physical safety
    measures.
6.  Record configuration versions and trial data so results are
    reproducible.
7.  Keep the first working system deliberately narrow: one known cube,
    one fixed camera, one tabletop.

## 6. Suggested first milestone

Before implementing AI features, establish a reliable baseline: -
Connect to the xArm 5 through the official driver. - Read joint states
and robot status. - Operate the AG1010 using the supported interface. -
Display the robot in RViz. - Launch the robot in MoveIt 2. - Launch the
same robot model in Gazebo. - Demonstrate a conservative, manually
specified pick-and-place in simulation.

Only after this baseline works should perception-driven autonomous
grasping be added.
