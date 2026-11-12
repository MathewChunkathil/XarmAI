# Graph Report - XarmAI  (2026-11-12)

## Corpus Check
- 61 files · ~41,694 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 18 file(s) not represented in the graph (top: .stl 14, (none) 1, .csv 1)

## Summary
- 727 nodes · 1369 edges · 52 communities (35 shown, 17 thin omitted)
- Extraction: 90% EXTRACTED · 10% INFERRED · 0% AMBIGUOUS · INFERRED: 137 edges (avg confidence: 0.86)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `891e40f2`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- Candidate Generation & IK Evaluator
- Tag Generation & PnP Perception
- SimWorld
- AprilTag Cube Model & Geometry
- Sim Demo Pipeline & CLI
- Project Rules & Core Decisions
- Hand-Eye Calibration & Kabsch
- Cube Model YAML Configuration
- AprilTag Fiducial Assets
- Simulated C270 Camera Config
- Workspace & Target Zone Config
- Candidate Ranking Cost Config
- Robot Hardware & Digital Twin
- Project Specs & 16-Week Roadmap
- Simulation Camera Visual Views
- Gripper YAML Configuration
- 5-DoF Grasp Candidate Planning
- Execution, Safety & Recovery
- Motion Planning YAML Config
- Layered Architecture & AI Supervisor
- Vision & Camera Calibration
- Sim Demo Results & Plans
- Experiments, Logging & Metrics
- Motion Planning & Kinematics
- AI Supervisor & Task Packages
- PyBullet Demo Honesty & Logs
- ROS 2 Interfaces & State Flow
- C++ MTC & 5-DoF IK Decisions
- AprilTag Detector & Axes
- Sim Camera Intrinsics Calibration
- xArm 5 & Gripper URDF Joints
- Experiment SQLite & MCAP Logging
- Gripper Interface Package (WP4)
- Python Dependencies (requirements.txt)
- Sim Frame & Ground-Truth Rules
- xarm_app_interfaces Package
- App
- test_world.py
- cube_model.py
- cv2
- os
- C270SimCamera
- calib_cache.py
- demo_pick_place.py
- test_sim_demo_disconnect.py

## God Nodes (most connected - your core abstractions)
1. `SimWorld` - 35 edges
2. `App` - 33 edges
3. `Pipeline` - 33 edges
4. `AprilTag tag36h11 Family` - 19 edges
5. `rank()` - 15 edges
6. `FakeIK` - 14 edges
7. `estimate_cube_pose()` - 13 edges
8. `generate()` - 13 edges
9. `evaluate()` - 13 edges
10. `README Baseline Overview` - 13 edges

## Surprising Connections (you probably didn't know these)
- `Demo Seed 0 Results (10/10)` --semantically_similar_to--> `10-Run Sim Trials Summary (10/10, 0.32 mm mean place err)`  [INFERRED] [semantically similar]
  docs/DEMO_SCRIPT.md → results/sim_demo/summary.md
- `Pipeline` --uses--> `SimWorld`  [INFERRED]
  scripts/sim_demo.py → sim/pyb_world.py
- `write_outputs()` --calls--> `load_yaml()`  [INFERRED]
  scripts/sim_demo.py → src/candidate_evaluator/candidate_evaluator/generate.py
- `world()` --uses--> `SimWorld`  [INFERRED]
  tests/sim/test_world.py → sim/pyb_world.py
- `cam()` --uses--> `C270SimCamera`  [INFERRED]
  tests/sim/test_world.py → sim/c270_camera.py

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Deterministic Safety and Verification Guardrails** — 02_system_architecture_ai_supervisor, 04_robot_and_tool_integration_robot_state_contract, 08_execution_safety_and_recovery_pre_execution_gate, 08_execution_safety_and_recovery_mandatory_stop_conditions, 08_execution_safety_and_recovery_grasp_verification, 08_execution_safety_and_recovery_bounded_recovery [EXTRACTED 1.00]
- **Perception-to-Ranked-Candidate Pipeline** — 05_vision_and_calibration_runtime_pose_estimation, 06_grasp_and_motion_planning_constrained_grasp_family, 06_grasp_and_motion_planning_kinematic_evaluation, 06_grasp_and_motion_planning_moveit_2, 07_digital_twin_and_simulation_in_the_loop, 06_grasp_and_motion_planning_candidate_ranking_cost [EXTRACTED 1.00]
- **Port-Ready Pure-Python Sim Demo Modules** — src_cube_perception_readme_pnp, src_calibration_tools_readme_kabsch, src_calibration_tools_readme_handeye, src_calibration_tools_readme_intrinsics, src_candidate_evaluator_readme_generate, src_candidate_evaluator_readme_evaluate, src_candidate_evaluator_readme_rank [EXTRACTED 1.00]
- **ROS 2 Autonomous Manipulation Pipeline** — implementation_plan_cube_perception_pkg, implementation_plan_calibration_tools_pkg, implementation_plan_candidate_evaluator_pkg, implementation_plan_grasp_planner_mtc_pkg, implementation_plan_sim_validator_pkg, implementation_plan_execution_manager_pkg, implementation_plan_task_orchestrator_pkg, implementation_plan_ai_supervisor_pkg [EXTRACTED 1.00]
- **ROS 2 Modular Manipulation Packages** — 03_software_and_repository_camera_perception, 03_software_and_repository_calibration_manager, 03_software_and_repository_scene_manager, 03_software_and_repository_grasp_planning, 03_software_and_repository_kinematics_evaluation, 03_software_and_repository_motion_planning_interface, 03_software_and_repository_digital_twin_interface, 03_software_and_repository_candidate_evaluator, 03_software_and_repository_execution_manager, 03_software_and_repository_verification_recovery, 03_software_and_repository_experiment_logger [EXTRACTED 1.00]
- **tag36h11 Fiducial Marker Suite (Physical, Phone, Simulation)** — results_tags_tag36h11_40mm_apriltag_tag36h11_family, results_tags_tag36h11_40mm_physical_40mm_cube_markers, results_tags_sim_tag_0_simulation_fiducial_markers, results_tags_phone_id0_phone_screen_fiducial_markers, results_tags_tag36h11_40mm_sheet [EXTRACTED 1.00]
- **Sim Demo Verification & Results Artifacts** — docs_demo_script_seed0_results, results_sim_demo_summary_intrinsics, results_sim_demo_summary_extrinsics_a, results_sim_demo_summary_trials [INFERRED 0.95]

## Communities (52 total, 17 thin omitted)

### Community 0 - "Candidate Generation & IK Evaluator"
Cohesion: 0.06
Nodes (41): generate(), _in_zone(), load_yaml(), pose_to_matrix(), quat_xyzw_to_rot(), rot_to_quat_xyzw(), tool_down_rotation(), wrap_pi() (+33 more)

### Community 1 - "Tag Generation & PnP Perception"
Cohesion: 0.17
Nodes (20): box(), detect(), detect_all(), ee_T(), flange_target(), go_home(), ik(), kabsch() (+12 more)

### Community 2 - "SimWorld"
Cohesion: 0.06
Nodes (8): quat_from_R(), SimWorld, box(), zone(), _tag_tile(), _write_obj(), _yaml(), test_cube_rests_on_table()

### Community 3 - "AprilTag Cube Model & Geometry"
Cohesion: 0.11
Nodes (27): wrap90(), corners_from_pupil(), cube_yaw(), CubePoseResult, estimate_cube_pose(), fold_yaw(), median_pose(), _rt_to_T() (+19 more)

### Community 4 - "Sim Demo Pipeline & CLI"
Cohesion: 0.06
Nodes (12): fmt(), is_disconnect_error(), log(), main(), Pipeline, rot_axis(), rot_deg(), rz() (+4 more)

### Community 5 - "Project Rules & Core Decisions"
Cohesion: 0.08
Nodes (26): Caveman Communication Rule, CLAUDE.md Project Rules, Model Ladder & Graphify Update Policy, Sim-First Work Rule, calibration_tools Package (WP3), candidate_evaluator Package (WP5/WP6), cube_perception Package (WP3), Decision D4: Dual Extrinsic Calibration (A & B) (+18 more)

### Community 6 - "Hand-Eye Calibration & Kabsch"
Cohesion: 0.14
Nodes (13): calibrate_eye_to_hand(), _project_so3(), rigid_error(), _rotvec(), umeyama(), umeyama_ransac(), _handeye_data(), rand_T() (+5 more)

### Community 7 - "Cube Model YAML Configuration"
Cohesion: 0.09
Nodes (22): calib_tag, family, id, R_tcp_tag, t_tcp_tag, tag_size_m, cube_id, edge_m (+14 more)

### Community 8 - "AprilTag Fiducial Assets"
Cohesion: 0.18
Nodes (20): Phone Screen tag36h11 ID 0 Marker, Phone Screen AprilTag Fiducial Markers, Phone Screen tag36h11 ID 1 Marker, Simulation tag36h11 ID 0 Marker, Simulation AprilTag Fiducial Markers, Simulation tag36h11 ID 11 Marker, Simulation tag36h11 ID 12 Marker, Simulation tag36h11 ID 13 Marker (+12 more)

### Community 9 - "Simulated C270 Camera Config"
Cohesion: 0.11
Nodes (18): blur_sigma_px, brightness_jitter, diag_fov_deg, distortion, far_m, focus, height, mjpeg_quality (+10 more)

### Community 10 - "Workspace & Target Zone Config"
Cohesion: 0.11
Nodes (18): cube_workspace_box, x_m, y_m, forbidden_zones, place_target, x_m, y_m, yaw_rad (+10 more)

### Community 11 - "Candidate Ranking Cost Config"
Cohesion: 0.13
Nodes (14): clearance_d_ref_m, grasp_angle_ref_rad, grasp_cov_trace_gain, kinematic_margin_max, motion_joint_weights, profile, profile_a_k, schema_version (+6 more)

### Community 12 - "Robot Hardware & Digital Twin"
Cohesion: 0.19
Nodes (13): Version 1 Pick-and-Place Task, digital_twin_interface, xarm_app_bringup, Official UFACTORY xarm_ros2 Driver, UFACTORY AG1010 Gripper, 04 Robot and Tool Integration, Physical Commissioning Sequence, Robot State Contract (+5 more)

### Community 13 - "Project Specs & 16-Week Roadmap"
Cohesion: 0.16
Nodes (10): 01 Project Specification, 03 Software Stack and Repository Layout, Ubuntu 24.04 and ROS 2 Jazzy Stack, 10 Development Roadmap, 16-Week Phased Development Roadmap (Phases 0-8), Confirmed Version 1 Decisions, 11 Decisions and Open Engineering Questions, Pre-Implementation Verification Backlog (+2 more)

### Community 14 - "Simulation Camera Visual Views"
Cohesion: 0.19
Nodes (11): AprilTag-marked cube, Green-outlined detected cube AprilTag, Six floor-mounted AprilTag markers (2x3 grid), Two-finger parallel robot gripper, SIM camera | detected cube tag overlay, Blue-grey and white two-tone simulation floor, AprilTag-marked cube (close-up), Brown textured workspace surface (+3 more)

### Community 15 - "Gripper YAML Configuration"
Cohesion: 0.18
Nodes (10): backend, command_timeout_s, feedback_available, grasp_close_width_m, max_open_m, min_close_m, open_margin_m, schema_version (+2 more)

### Community 16 - "5-DoF Grasp Candidate Planning"
Cohesion: 0.22
Nodes (7): candidate_evaluator, grasp_planning, Candidate Ranking Cost Function, Constrained Top-Down Grasp Family, 06 Grasp and Motion Planning, Grasp Candidate Record, Simulation-in-the-Loop Validation

### Community 17 - "Execution, Safety & Recovery"
Cohesion: 0.25
Nodes (8): execution_manager, verification_recovery, 08 Execution, Safety and Recovery, 12-Stage Physical Execution Sequence, Independent Grasp and Placement Verification, Mandatory Stop Conditions, System Operating Modes, Deterministic Pre-Execution Gate

### Community 18 - "Motion Planning YAML Config"
Cohesion: 0.22
Nodes (8): approach_distance_m, grasp_height_offset_m, lift_distance_m, max_acceleration_scaling, max_velocity_scaling, schema_version, table_clearance_m, yaw_offsets_deg

### Community 19 - "Layered Architecture & AI Supervisor"
Cohesion: 0.25
Nodes (6): AI Supervisor, 02 System Architecture, Task Orchestrator, ai_supervisor, Architecture Overview, 10-Step End-to-End Operating Sequence

### Community 20 - "Vision & Camera Calibration"
Cohesion: 0.29
Nodes (7): calibration_manager, camera_perception, 05 Vision and Calibration, Camera-to-Robot Extrinsic Calibration, Camera Intrinsic Calibration, Logitech C270 Camera, Runtime Cube Pose Estimation Pipeline

### Community 21 - "Sim Demo Results & Plans"
Cohesion: 0.25
Nodes (7): DEMO_SCRIPT.md (13 Nov 2026), Demo Seed 0 Results (10/10), IMPLEMENTATION_PLAN.md (v1.0), Sim Extrinsics A Result (0.4 mm / 0.08 deg), Sim Intrinsics Result (0.192 px RMS), 10-Run Sim Trials Summary (10/10, 0.32 mm mean place err), SIM_DEMO_PLAN.md (v2)

### Community 22 - "Experiments, Logging & Metrics"
Cohesion: 0.29
Nodes (6): dashboard, experiment_logger, Dependent Evaluation Metrics, 09 Experiments and Metrics, Experimental Configurations A-D, Trial Logging Schema

### Community 23 - "Motion Planning & Kinematics"
Cohesion: 0.29
Nodes (7): kinematics_evaluation, motion_planning_interface, scene_manager, 10-Step Pick-and-Place Motion Sequence, 5-DoF Kinematic Evaluation, MoveIt 2 Motion Planning, Four-Scene Synchronisation

### Community 24 - "AI Supervisor & Task Packages"
Cohesion: 0.33
Nodes (6): ai_supervisor Package (WP9), Decision D10: YASMIN Task FSM, Decision D11: Foxglove & RViz Dashboard, Decision D3: Pluggable LLM Provider, Experiment Profiles A-D (WP10), task_orchestrator Package (WP8)

### Community 25 - "PyBullet Demo Honesty & Logs"
Cohesion: 0.40
Nodes (5): Demo Proven vs Not Proven Claims, pybullet==3.2.7, xArm6 Stand-in Pick-Place Log (1.9 mm error), Simulation Honesty Labels, PyBullet All-Sim Demo Pipeline

### Community 26 - "ROS 2 Interfaces & State Flow"
Cohesion: 0.67
Nodes (3): Core ROS 2 Data Flow, Task State Machine, xarm_app_interfaces

### Community 27 - "C++ MTC & 5-DoF IK Decisions"
Cohesion: 0.67
Nodes (3): Decision D6: Python Nodes + C++ MTC Planner, Decision D9: pick_ik 5-DoF IK, grasp_planner_mtc C++ Package (WP6)

### Community 29 - "Sim Camera Intrinsics Calibration"
Cohesion: 0.67
Nodes (3): Fake Logitech C270 Camera Model (c270_sim.yaml), Sim Checkerboard Intrinsics Calibration, intrinsics.calibrate_checkerboard

### Community 38 - "App"
Cohesion: 0.07
Nodes (11): App, prog(), solve(), button_theme(), Cancel, f1(), main(), make_theme() (+3 more)

### Community 43 - "test_world.py"
Cohesion: 0.16
Nodes (9): rot_err_rad(), cam(), cube_frame(), _tag_R_world(), test_calib_tag_visible(), test_checkerboard_found(), test_ik_top_down(), test_tag_frames_match_cube_model() (+1 more)

### Community 44 - "cube_model.py"
Cohesion: 0.18
Nodes (8): CubeModel, load_cube_model(), tag_corners_in_tag_frame(), load(), test_corners_lie_on_face(), test_ids_unique_and_five_faces(), test_rotations_and_offsets(), test_tag_fits_face()

### Community 45 - "cv2"
Cohesion: 0.17
Nodes (3): load_cfg(), calibrate_checkerboard(), test_detect_ids_only_model_ids()

### Community 48 - "calib_cache.py"
Cohesion: 0.31
Nodes (4): config_hash(), load(), save(), _ser()

### Community 49 - "demo_pick_place.py"
Cohesion: 0.15
Nodes (6): flange_target(), ik(), move(), set_grip(), T_from(), Tinv()

## Knowledge Gaps
- **61 isolated node(s):** `SIM_DEMO_PLAN.md (v2)`, `digital_twin_interface`, `xarm_app_bringup`, `Physical Commissioning Sequence`, `Simulation Fidelity Levels 1-3` (+56 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 290 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **17 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `SimWorld` connect `SimWorld` to `demo_pick_place.py`, `test_world.py`, `Sim Demo Pipeline & CLI`, `cv2`?**
  _High betweenness centrality (0.053) - this node is a cross-community bridge._
- **Are the 2 inferred relationships involving `SimWorld` (e.g. with `Pipeline` and `world()`) actually correct?**
  _`SimWorld` has 2 INFERRED edges - model-reasoned connections that need verification._
- **What connects `SIM_DEMO_PLAN.md (v2)`, `digital_twin_interface`, `xarm_app_bringup` to the rest of the system?**
  _61 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Candidate Generation & IK Evaluator` be split into smaller, more focused modules?**
  _Cohesion score 0.06126126126126126 - nodes in this community are weakly interconnected._
- **Why does `Pipeline` connect `Sim Demo Pipeline & CLI` to `SimWorld`, `App`?**
  _High betweenness centrality (0.052) - this node is a cross-community bridge._
- **Should `SimWorld` be split into smaller, more focused modules?**
  _Cohesion score 0.06431372549019608 - nodes in this community are weakly interconnected._
- **Should `AprilTag Cube Model & Geometry` be split into smaller, more focused modules?**
  _Cohesion score 0.11336032388663968 - nodes in this community are weakly interconnected._