# Graph Report - XarmAI  (2026-11-12)

## Corpus Check
- 21 files · ~18,428 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 1 file(s) not represented in the graph (top: .obj 1)

## Summary
- 210 nodes · 290 edges · 13 communities (12 shown, 1 thin omitted)
- Extraction: 97% EXTRACTED · 3% INFERRED · 0% AMBIGUOUS · INFERRED: 9 edges (avg confidence: 0.87)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- Project Specs, Roadmap & Decisions
- Robot Hardware & Digital Twin
- Execution, Safety & Recovery
- Layered Architecture & AI Supervisor
- demo_sim_vision.py
- cube_model.yaml
- IMPLEMENTATION PLAN — xArm 5 AI-Assisted Autonomous Manipulation
- 7. Full roadmap — work packages
- ROS 2 Interfaces & State Flow
- 5. Configuration schemas and core algorithms
- test_cube_model.py
- CLAUDE.md — XarmAI rules (MANDATORY)

## God Nodes (most connected - your core abstractions)
1. `IMPLEMENTATION PLAN — xArm 5 AI-Assisted Autonomous Manipulation` - 14 edges
2. `7. Full roadmap — work packages` - 14 edges
3. `README Baseline Overview` - 13 edges
4. `5. Configuration schemas and core algorithms` - 12 edges
5. `UFACTORY xArm 5` - 9 edges
6. `Gazebo Digital Twin` - 9 edges
7. `Version 1 Pick-and-Place Task` - 7 edges
8. `Runtime Cube Pose Estimation Pipeline` - 7 edges
9. `calib_tag` - 6 edges
10. `load()` - 6 edges

## Surprising Connections (you probably didn't know these)
- `WP7 — Digital-twin validation `[SIM]`` --references--> `set_pose()`  [INFERRED]
  IMPLEMENTATION_PLAN.md → scripts/demo_sim_vision.py
- `Constrained Top-Down Grasp Family` --semantically_similar_to--> `5-DoF Kinematic Constraints`  [INFERRED] [semantically similar]
  06_GRASP_AND_MOTION_PLANNING.md → 04_ROBOT_AND_TOOL_INTEGRATION.md
- `Vertical Slice Implementation Order` --semantically_similar_to--> `First Baseline Milestone`  [INFERRED] [semantically similar]
  03_SOFTWARE_AND_REPOSITORY.md → README.md
- `First Baseline Milestone` --semantically_similar_to--> `Schedule Protection Priority Policy`  [INFERRED] [semantically similar]
  README.md → 10_DEVELOPMENT_ROADMAP.md
- `README Baseline Overview` --references--> `05 Vision and Calibration`  [EXTRACTED]
  README.md → 05_VISION_AND_CALIBRATION.md

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Deterministic Safety and Verification Guardrails** — 02_system_architecture_ai_supervisor, 04_robot_and_tool_integration_robot_state_contract, 08_execution_safety_and_recovery_pre_execution_gate, 08_execution_safety_and_recovery_mandatory_stop_conditions, 08_execution_safety_and_recovery_grasp_verification, 08_execution_safety_and_recovery_bounded_recovery [EXTRACTED 1.00]
- **Perception-to-Ranked-Candidate Pipeline** — 05_vision_and_calibration_runtime_pose_estimation, 06_grasp_and_motion_planning_constrained_grasp_family, 06_grasp_and_motion_planning_kinematic_evaluation, 06_grasp_and_motion_planning_moveit_2, 07_digital_twin_and_simulation_in_the_loop, 06_grasp_and_motion_planning_candidate_ranking_cost [EXTRACTED 1.00]
- **ROS 2 Modular Manipulation Packages** — 03_software_and_repository_camera_perception, 03_software_and_repository_calibration_manager, 03_software_and_repository_scene_manager, 03_software_and_repository_grasp_planning, 03_software_and_repository_kinematics_evaluation, 03_software_and_repository_motion_planning_interface, 03_software_and_repository_digital_twin_interface, 03_software_and_repository_candidate_evaluator, 03_software_and_repository_execution_manager, 03_software_and_repository_verification_recovery, 03_software_and_repository_experiment_logger [EXTRACTED 1.00]

## Communities (13 total, 1 thin omitted)

### Community 0 - "Project Specs, Roadmap & Decisions"
Cohesion: 0.11
Nodes (16): 01 Project Specification, dashboard, 03 Software Stack and Repository Layout, experiment_logger, Ubuntu 24.04 and ROS 2 Jazzy Stack, Dependent Evaluation Metrics, 09 Experiments and Metrics, Experimental Configurations A-D (+8 more)

### Community 1 - "Robot Hardware & Digital Twin"
Cohesion: 0.12
Nodes (20): Version 1 Pick-and-Place Task, calibration_manager, camera_perception, digital_twin_interface, xarm_app_bringup, Official UFACTORY xarm_ros2 Driver, UFACTORY AG1010 Gripper, 04 Robot and Tool Integration (+12 more)

### Community 2 - "Execution, Safety & Recovery"
Cohesion: 0.12
Nodes (15): candidate_evaluator, execution_manager, grasp_planning, verification_recovery, Candidate Ranking Cost Function, Constrained Top-Down Grasp Family, 06 Grasp and Motion Planning, Grasp Candidate Record (+7 more)

### Community 3 - "Layered Architecture & AI Supervisor"
Cohesion: 0.13
Nodes (13): AI Supervisor, 02 System Architecture, Task Orchestrator, ai_supervisor, kinematics_evaluation, motion_planning_interface, scene_manager, 10-Step Pick-and-Place Motion Sequence (+5 more)

### Community 4 - "demo_sim_vision.py"
Cohesion: 0.11
Nodes (25): flange_target(), ik(), move(), set_grip(), box(), detect(), detect_all(), ee_T() (+17 more)

### Community 5 - "cube_model.yaml"
Cohesion: 0.09
Nodes (22): calib_tag, family, id, R_tcp_tag, t_tcp_tag, tag_size_m, cube_id, edge_m (+14 more)

### Community 6 - "IMPLEMENTATION PLAN — xArm 5 AI-Assisted Autonomous Manipulation"
Cohesion: 0.10
Nodes (19): 0.1 Rules for every agent / developer, 0.2 Definition of Done (per WP), 0.3 Tags used in this plan, 0. How to use this document (read first), 10. Risks and mitigations, 11. Open items (seed for `docs/OPEN_ITEMS.md`), 12. Source-doc traceability, 1. Decisions (confirmed in Q&A + research) and corrections to the source docs (+11 more)

### Community 7 - "7. Full roadmap — work packages"
Cohesion: 0.14
Nodes (14): 7. Full roadmap — work packages, WP0 — Workspace, tooling, facts register `[SIM]`, WP10 — Experiments A–D `[SIM]` + `[HW]`, WP11 — Analysis and dissertation artefacts, WP12 — Experiment logger and dashboard `[SIM]`, WP1 — Interfaces and common utilities `[SIM]`, WP2 — Simulation baseline: description + Gazebo + MoveIt `[SIM]`, WP3 — Vision and calibration `[CAM]` + `[HW]` for extrinsics (+6 more)

### Community 8 - "ROS 2 Interfaces & State Flow"
Cohesion: 0.67
Nodes (3): Core ROS 2 Data Flow, Task State Machine, xarm_app_interfaces

### Community 9 - "5. Configuration schemas and core algorithms"
Cohesion: 0.17
Nodes (12): 5.10 Verification (`execution_manager.verify`), 5.11 Recovery (bounded), 5.1 Camera (`config/camera/c270.yaml`), 5.2 Extrinsics file (`config/calibration/extrinsics_<date>_<method>.yaml`, symlinked as `extrinsics_active.yaml`), 5.3 Cube model (`config/cube/cube_model.yaml`), 5.4 Workspace (`config/workspace/workspace.yaml`), 5.5 Gripper (`config/gripper/gripper.yaml`), 5.6 Candidate generation (`candidate_evaluator`, pure Python, unit-tested) (+4 more)

### Community 10 - "test_cube_model.py"
Cohesion: 0.36
Nodes (5): load(), test_corners_lie_on_face(), test_ids_unique_and_five_faces(), test_rotations_and_offsets(), test_tag_fits_face()

### Community 11 - "CLAUDE.md — XarmAI rules (MANDATORY)"
Cohesion: 0.40
Nodes (4): 1. Talk = caveman. ALWAYS., 2. Model ladder, 3. Work rules, CLAUDE.md — XarmAI rules (MANDATORY)

## Knowledge Gaps
- **22 isolated node(s):** `Ubuntu 24.04 and ROS 2 Jazzy Stack`, `Confirmed Version 1 Decisions`, `Pre-Implementation Verification Backlog`, `digital_twin_interface`, `xarm_app_bringup` (+17 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 102 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **1 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `7. Full roadmap — work packages` connect `7. Full roadmap — work packages` to `IMPLEMENTATION PLAN — xArm 5 AI-Assisted Autonomous Manipulation`?**
  _High betweenness centrality (0.138) - this node is a cross-community bridge._
- **What connects `Ubuntu 24.04 and ROS 2 Jazzy Stack`, `Confirmed Version 1 Decisions`, `Pre-Implementation Verification Backlog` to the rest of the system?**
  _22 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Project Specs, Roadmap & Decisions` be split into smaller, more focused modules?**
  _Cohesion score 0.10952380952380952 - nodes in this community are weakly interconnected._
- **Why does `IMPLEMENTATION PLAN — xArm 5 AI-Assisted Autonomous Manipulation` connect `IMPLEMENTATION PLAN — xArm 5 AI-Assisted Autonomous Manipulation` to `5. Configuration schemas and core algorithms`, `7. Full roadmap — work packages`?**
  _High betweenness centrality (0.119) - this node is a cross-community bridge._
- **Should `Robot Hardware & Digital Twin` be split into smaller, more focused modules?**
  _Cohesion score 0.12121212121212122 - nodes in this community are weakly interconnected._
- **Why does `set_pose()` connect `demo_sim_vision.py` to `7. Full roadmap — work packages`?**
  _High betweenness centrality (0.119) - this node is a cross-community bridge._
- **Should `Execution, Safety & Recovery` be split into smaller, more focused modules?**
  _Cohesion score 0.11764705882352941 - nodes in this community are weakly interconnected._