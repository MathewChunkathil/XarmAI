# 03 --- Software Stack and Repository Layout

## 1. Recommended development environment

**Primary:** Ubuntu 24.04 LTS with ROS 2 Jazzy.

Use the ASUS ROG Zephyrus G16 as the primary robotics workstation if its
Linux GPU, Wi-Fi, USB and display drivers work reliably. Keep Windows
available for general use and vendor utilities. WSL2 can be useful for
secondary development but should not be the assumed final environment
for hardware-connected ROS 2, Gazebo and MoveIt integration.

The ThinkPad can be used for secondary development, documentation,
monitoring and non-GPU-heavy tasks.

## 2. Proposed stack

  ----------------------------------------------------------------------------
  Function                Preferred tool          Notes
  ----------------------- ----------------------- ----------------------------
  OS                      Ubuntu 24.04 LTS        Primary ROS environment

  Middleware              ROS 2 Jazzy             Nodes, topics, services,
                                                  actions

  Robot driver            Official UFACTORY       Reuse vendor-supported
                          `xarm_ros2`             integration

  Robot SDK               UFACTORY xArm Python    Use where required by
                          SDK                     supported interfaces

  Motion planning         MoveIt 2                IK, planning scene,
                                                  collision-aware trajectories

  Visualisation           RViz 2                  TF, robot state, planning
                                                  scene

  Simulation              Gazebo compatible with  Verify exact supported
                          Jazzy                   release and packages

  Vision                  OpenCV + AprilTag       Fixed C270 and known marker
                          detector                layout

  Language                Python and C++          Python for
                                                  orchestration/experiments;
                                                  C++ where performance or ROS
                                                  integration benefits

  Data                    SQLite + structured     Reproducible trial logging
                          CSV/JSON and image      
                          records                 

  Version control         Git                     Branches, tags,
                                                  configuration tracking

  Dashboard               Start with RViz +       Avoid building a complex UI
                          lightweight ROS         before core pipeline works
                          dashboard               
  ----------------------------------------------------------------------------

Confirm current UFACTORY compatibility and installation instructions
from the official repository before pinning package versions.

## 3. Repository layout

This is the proposed application workspace. Vendor packages should
remain external dependencies or be included as documented submodules; do
not copy and fork them casually.

``` text
xarm5_ai_manipulation/
├── README.md
├── docs/
│   ├── 01_PROJECT_SPECIFICATION.md
│   ├── 02_SYSTEM_ARCHITECTURE.md
│   ├── 03_SOFTWARE_AND_REPOSITORY.md
│   ├── 04_ROBOT_AND_TOOL_INTEGRATION.md
│   ├── 05_VISION_AND_CALIBRATION.md
│   ├── 06_GRASP_AND_MOTION_PLANNING.md
│   ├── 07_DIGITAL_TWIN_AND_SIMULATION.md
│   ├── 08_EXECUTION_SAFETY_AND_RECOVERY.md
│   ├── 09_EXPERIMENTS_AND_METRICS.md
│   ├── 10_DEVELOPMENT_ROADMAP.md
│   └── 11_DECISIONS_AND_OPEN_QUESTIONS.md
├── src/
│   ├── xarm_app_bringup/
│   ├── xarm_app_interfaces/
│   ├── camera_perception/
│   ├── calibration_manager/
│   ├── scene_manager/
│   ├── grasp_planning/
│   ├── kinematics_evaluation/
│   ├── motion_planning_interface/
│   ├── digital_twin_interface/
│   ├── candidate_evaluator/
│   ├── ai_supervisor/
│   ├── execution_manager/
│   ├── verification_recovery/
│   ├── experiment_logger/
│   └── dashboard/
├── config/
│   ├── camera/
│   ├── calibration/
│   ├── robot/
│   ├── gripper/
│   ├── workspace/
│   ├── planning/
│   └── experiment_profiles/
├── description/
│   ├── custom_tool_overrides/
│   └── workspace_models/
├── launch/
├── scripts/
├── tests/
│   ├── unit/
│   ├── integration/
│   ├── simulation/
│   └── hardware/
├── data/
│   ├── calibration/
│   ├── experiments/
│   └── sample_images/
└── results/
```

## 4. Package responsibilities

-   `xarm_app_bringup`: launch files and system startup orchestration.
-   `xarm_app_interfaces`: custom ROS messages, services and actions.
-   `camera_perception`: camera acquisition, tag detection and cube
    pose.
-   `calibration_manager`: load/save/validate intrinsics and extrinsics.
-   `scene_manager`: table, cube and obstacle representation in planning
    scene.
-   `grasp_planning`: candidate generation for the known cube.
-   `kinematics_evaluation`: IK and robot-specific feasibility metrics.
-   `motion_planning_interface`: MoveIt 2 request/response wrapper.
-   `digital_twin_interface`: Gazebo scene and simulation coordination.
-   `candidate_evaluator`: hard-constraint filtering and numerical
    ranking.
-   `ai_supervisor`: high-level task interpretation and tool
    orchestration.
-   `execution_manager`: deterministic preflight and physical execution.
-   `verification_recovery`: grasp/placement checks and bounded
    recovery.
-   `experiment_logger`: record inputs, outputs, timings and outcomes.
-   `dashboard`: operator status, candidate results and controls.

## 5. Build and code conventions

-   Use one ROS 2 workspace and standard `colcon` build.
-   Each package must have a clear README describing its inputs, outputs
    and launch/test commands.
-   Store tunable values in YAML, not scattered through source code.
-   Use ROS parameters for runtime configuration.
-   Use typed ROS messages/actions for safety-relevant interfaces.
-   Add unit tests for transforms, pose conversion, scoring and state
    transitions.
-   Add simulation tests before hardware tests.
-   Keep hardware tests opt-in and clearly labelled.
-   Pin tested dependency versions in setup documentation.

## 6. Recommended implementation order

Do not create every package as empty scaffolding on day one. Build the
smallest end-to-end vertical slice, then split/refine packages as
interfaces become clear.
