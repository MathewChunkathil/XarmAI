# Graph Report - XarmAI  (2026-11-12)

## Corpus Check
- 57 files · ~35,400 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 18 file(s) not represented in the graph (top: .stl 14, (none) 1, .csv 1)

## Summary
- 593 nodes · 1109 edges · 30 communities (27 shown, 3 thin omitted)
- Extraction: 90% EXTRACTED · 10% INFERRED · 0% AMBIGUOUS · INFERRED: 107 edges (avg confidence: 0.87)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `3466c688`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- Project Specs, Roadmap & Decisions
- UFACTORY xArm 5
- Deterministic Pre-Execution Gate
- Layered Architecture & AI Supervisor
- demo_sim_vision.py
- cube_model.yaml
- IMPLEMENTATION PLAN — xArm 5 AI-Assisted Autonomous Manipulation
- SimWorld
- ROS 2 Interfaces & State Flow
- SIM DEMO PLAN — everything in simulation (v2, 2026-11-12)
- test_pnp.py
- CLAUDE.md — XarmAI rules (MANDATORY)
- generate.py
- test_kabsch.py
- c270_sim.yaml
- ranking.yaml
- workspace.yaml
- sim_demo.py
- gripper.yaml
- Constrained Top-Down Grasp Family
- planning.yaml
- Runtime Cube Pose Estimation Pipeline
- Experimental Configurations A-D
- MoveIt 2 Motion Planning
- C270SimCamera
- calibration_tools/README.md
- Demo script - 13 Nov 2026 (about 5 minutes)
- XarmAI simulation demo - results
- test_sim_demo_smoke.py

## God Nodes (most connected - your core abstractions)
1. `SimWorld` - 28 edges
2. `Pipeline` - 20 edges
3. `rank()` - 15 edges
4. `FakeIK` - 14 edges
5. `IMPLEMENTATION PLAN — xArm 5 AI-Assisted Autonomous Manipulation` - 14 edges
6. `7. Full roadmap — work packages` - 14 edges
7. `C270SimCamera` - 13 edges
8. `estimate_cube_pose()` - 13 edges
9. `generate()` - 13 edges
10. `evaluate()` - 13 edges

## Surprising Connections (you probably didn't know these)
- `2. Architecture (45 s) - from `02_SYSTEM_ARCHITECTURE.md`` --references--> `IKBackend`  [INFERRED]
  docs/DEMO_SCRIPT.md → src/candidate_evaluator/candidate_evaluator/ik_backend.py
- `WP7 — Digital-twin validation `[SIM]`` --references--> `set_pose()`  [INFERRED]
  IMPLEMENTATION_PLAN.md → scripts/demo_sim_vision.py
- `Pipeline` --uses--> `C270SimCamera`  [INFERRED]
  scripts/sim_demo.py → sim/c270_camera.py
- `Pipeline` --uses--> `SimWorld`  [INFERRED]
  scripts/sim_demo.py → sim/pyb_world.py
- `Inputs` --references--> `IKBackend`  [INFERRED]
  src/candidate_evaluator/README.md → src/candidate_evaluator/candidate_evaluator/ik_backend.py

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Deterministic Safety and Verification Guardrails** — 02_system_architecture_ai_supervisor, 04_robot_and_tool_integration_robot_state_contract, 08_execution_safety_and_recovery_pre_execution_gate, 08_execution_safety_and_recovery_mandatory_stop_conditions, 08_execution_safety_and_recovery_grasp_verification, 08_execution_safety_and_recovery_bounded_recovery [EXTRACTED 1.00]
- **Perception-to-Ranked-Candidate Pipeline** — 05_vision_and_calibration_runtime_pose_estimation, 06_grasp_and_motion_planning_constrained_grasp_family, 06_grasp_and_motion_planning_kinematic_evaluation, 06_grasp_and_motion_planning_moveit_2, 07_digital_twin_and_simulation_in_the_loop, 06_grasp_and_motion_planning_candidate_ranking_cost [EXTRACTED 1.00]
- **ROS 2 Modular Manipulation Packages** — 03_software_and_repository_camera_perception, 03_software_and_repository_calibration_manager, 03_software_and_repository_scene_manager, 03_software_and_repository_grasp_planning, 03_software_and_repository_kinematics_evaluation, 03_software_and_repository_motion_planning_interface, 03_software_and_repository_digital_twin_interface, 03_software_and_repository_candidate_evaluator, 03_software_and_repository_execution_manager, 03_software_and_repository_verification_recovery, 03_software_and_repository_experiment_logger [EXTRACTED 1.00]

## Communities (30 total, 3 thin omitted)

### Community 0 - "Project Specs, Roadmap & Decisions"
Cohesion: 0.16
Nodes (10): 01 Project Specification, 03 Software Stack and Repository Layout, Ubuntu 24.04 and ROS 2 Jazzy Stack, 10 Development Roadmap, 16-Week Phased Development Roadmap (Phases 0-8), Confirmed Version 1 Decisions, 11 Decisions and Open Engineering Questions, Pre-Implementation Verification Backlog (+2 more)

### Community 1 - "UFACTORY xArm 5"
Cohesion: 0.19
Nodes (13): Version 1 Pick-and-Place Task, digital_twin_interface, xarm_app_bringup, Official UFACTORY xarm_ros2 Driver, UFACTORY AG1010 Gripper, 04 Robot and Tool Integration, Physical Commissioning Sequence, Robot State Contract (+5 more)

### Community 2 - "Deterministic Pre-Execution Gate"
Cohesion: 0.25
Nodes (8): execution_manager, verification_recovery, 08 Execution, Safety and Recovery, 12-Stage Physical Execution Sequence, Independent Grasp and Placement Verification, Mandatory Stop Conditions, System Operating Modes, Deterministic Pre-Execution Gate

### Community 3 - "Layered Architecture & AI Supervisor"
Cohesion: 0.25
Nodes (6): AI Supervisor, 02 System Architecture, Task Orchestrator, ai_supervisor, Architecture Overview, 10-Step End-to-End Operating Sequence

### Community 4 - "demo_sim_vision.py"
Cohesion: 0.07
Nodes (29): flange_target(), ik(), move(), set_grip(), box(), detect(), detect_all(), ee_T() (+21 more)

### Community 5 - "cube_model.yaml"
Cohesion: 0.09
Nodes (22): calib_tag, family, id, R_tcp_tag, t_tcp_tag, tag_size_m, cube_id, edge_m (+14 more)

### Community 6 - "IMPLEMENTATION PLAN — xArm 5 AI-Assisted Autonomous Manipulation"
Cohesion: 0.04
Nodes (45): 0.1 Rules for every agent / developer, 0.2 Definition of Done (per WP), 0.3 Tags used in this plan, 0. How to use this document (read first), 10. Risks and mitigations, 11. Open items (seed for `docs/OPEN_ITEMS.md`), 12. Source-doc traceability, 1. Decisions (confirmed in Q&A + research) and corrections to the source docs (+37 more)

### Community 7 - "SimWorld"
Cohesion: 0.06
Nodes (20): rot_deg(), quat_from_R(), rot_err_rad(), SimWorld, _tag_tile(), Tinv(), _write_obj(), _yaml() (+12 more)

### Community 8 - "ROS 2 Interfaces & State Flow"
Cohesion: 0.67
Nodes (3): Core ROS 2 Data Flow, Task State Machine, xarm_app_interfaces

### Community 9 - "SIM DEMO PLAN — everything in simulation (v2, 2026-11-12)"
Cohesion: 0.20
Nodes (9): 1. Goal, 2. What exists now (built 12 Nov) and the gaps, 3. Fake C270 camera (`config/camera/c270_sim.yaml`), 4. Code layout (port-ready), 5. Steps, models, checks, 6. Results table (`results/sim_demo/summary.md`), 7. Honesty labels (printed on screen and in the summary), 8. Open doubts (need a user answer before S1) (+1 more)

### Community 10 - "test_pnp.py"
Cohesion: 0.09
Nodes (30): wrap90(), CubeModel, load_cube_model(), tag_corners_in_tag_frame(), corners_from_pupil(), cube_yaw(), CubePoseResult, estimate_cube_pose() (+22 more)

### Community 11 - "CLAUDE.md — XarmAI rules (MANDATORY)"
Cohesion: 0.40
Nodes (4): 1. Talk = caveman. ALWAYS., 2. Model ladder, 3. Work rules, CLAUDE.md — XarmAI rules (MANDATORY)

### Community 12 - "generate.py"
Cohesion: 0.06
Nodes (43): generate(), _in_zone(), pose_to_matrix(), quat_xyzw_to_rot(), rot_to_quat_xyzw(), tool_down_rotation(), wrap_pi(), evaluate() (+35 more)

### Community 13 - "test_kabsch.py"
Cohesion: 0.14
Nodes (13): calibrate_eye_to_hand(), _project_so3(), rigid_error(), _rotvec(), umeyama(), umeyama_ransac(), _handeye_data(), rand_T() (+5 more)

### Community 14 - "c270_sim.yaml"
Cohesion: 0.11
Nodes (18): blur_sigma_px, brightness_jitter, diag_fov_deg, distortion, far_m, focus, height, mjpeg_quality (+10 more)

### Community 15 - "ranking.yaml"
Cohesion: 0.13
Nodes (14): clearance_d_ref_m, grasp_angle_ref_rad, grasp_cov_trace_gain, kinematic_margin_max, motion_joint_weights, profile, profile_a_k, schema_version (+6 more)

### Community 16 - "workspace.yaml"
Cohesion: 0.12
Nodes (15): cube_workspace_box, x_m, y_m, forbidden_zones, place_target, x_m, y_m, yaw_rad (+7 more)

### Community 17 - "sim_demo.py"
Cohesion: 0.09
Nodes (10): fmt(), log(), main(), Pipeline, rot_axis(), rz(), T_of(), write_outputs() (+2 more)

### Community 18 - "gripper.yaml"
Cohesion: 0.18
Nodes (10): backend, command_timeout_s, feedback_available, grasp_close_width_m, max_open_m, min_close_m, open_margin_m, schema_version (+2 more)

### Community 19 - "Constrained Top-Down Grasp Family"
Cohesion: 0.22
Nodes (7): candidate_evaluator, grasp_planning, Candidate Ranking Cost Function, Constrained Top-Down Grasp Family, 06 Grasp and Motion Planning, Grasp Candidate Record, Simulation-in-the-Loop Validation

### Community 20 - "planning.yaml"
Cohesion: 0.22
Nodes (8): approach_distance_m, grasp_height_offset_m, lift_distance_m, max_acceleration_scaling, max_velocity_scaling, schema_version, table_clearance_m, yaw_offsets_deg

### Community 21 - "Runtime Cube Pose Estimation Pipeline"
Cohesion: 0.29
Nodes (7): calibration_manager, camera_perception, 05 Vision and Calibration, Camera-to-Robot Extrinsic Calibration, Camera Intrinsic Calibration, Logitech C270 Camera, Runtime Cube Pose Estimation Pipeline

### Community 22 - "Experimental Configurations A-D"
Cohesion: 0.29
Nodes (6): dashboard, experiment_logger, Dependent Evaluation Metrics, 09 Experiments and Metrics, Experimental Configurations A-D, Trial Logging Schema

### Community 23 - "MoveIt 2 Motion Planning"
Cohesion: 0.29
Nodes (7): kinematics_evaluation, motion_planning_interface, scene_manager, 10-Step Pick-and-Place Motion Sequence, 5-DoF Kinematic Evaluation, MoveIt 2 Motion Planning, Four-Scene Synchronisation

### Community 24 - "C270SimCamera"
Cohesion: 0.14
Nodes (8): C270SimCamera, load_cfg(), cam(), load(), test_corners_lie_on_face(), test_ids_unique_and_five_faces(), test_rotations_and_offsets(), test_tag_fits_face()

### Community 26 - "Demo script - 13 Nov 2026 (about 5 minutes)"
Cohesion: 0.25
Nodes (7): 1. Problem (30 s), 2. Architecture (45 s) - from `02_SYSTEM_ARCHITECTURE.md`, 3. What the sim shows, step by step (2 min), 4. Real numbers (45 s) - seed 0, from `summary.md`, 5. What it proves and what it does NOT (60 s), 6. Next steps (30 s), Demo script - 13 Nov 2026 (about 5 minutes)

### Community 27 - "XarmAI simulation demo - results"
Cohesion: 0.33
Nodes (5): 1. Intrinsics (checkerboard, 20 views, RMS reprojection 0.186 px), 2. Extrinsics A (tag 10 on gripper, eye-to-hand, no ground truth in the solve), 3. Runs, 4. Totals, XarmAI simulation demo - results

## Knowledge Gaps
- **22 isolated node(s):** `Ubuntu 24.04 and ROS 2 Jazzy Stack`, `Confirmed Version 1 Decisions`, `Pre-Implementation Verification Backlog`, `digital_twin_interface`, `xarm_app_bringup` (+17 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 272 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **3 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `set_pose()` connect `demo_sim_vision.py` to `IMPLEMENTATION PLAN — xArm 5 AI-Assisted Autonomous Manipulation`?**
  _High betweenness centrality (0.092) - this node is a cross-community bridge._
- **Are the 2 inferred relationships involving `SimWorld` (e.g. with `Pipeline` and `world()`) actually correct?**
  _`SimWorld` has 2 INFERRED edges - model-reasoned connections that need verification._
- **What connects `Ubuntu 24.04 and ROS 2 Jazzy Stack`, `Confirmed Version 1 Decisions`, `Pre-Implementation Verification Backlog` to the rest of the system?**
  _22 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `demo_sim_vision.py` be split into smaller, more focused modules?**
  _Cohesion score 0.07049180327868852 - nodes in this community are weakly interconnected._
- **Why does `WP7 — Digital-twin validation `[SIM]`` connect `IMPLEMENTATION PLAN — xArm 5 AI-Assisted Autonomous Manipulation` to `demo_sim_vision.py`?**
  _High betweenness centrality (0.090) - this node is a cross-community bridge._
- **Are the 3 inferred relationships involving `Pipeline` (e.g. with `C270SimCamera` and `PyBulletIK`) actually correct?**
  _`Pipeline` has 3 INFERRED edges - model-reasoned connections that need verification._
- **Should `cube_model.yaml` be split into smaller, more focused modules?**
  _Cohesion score 0.08695652173913043 - nodes in this community are weakly interconnected._