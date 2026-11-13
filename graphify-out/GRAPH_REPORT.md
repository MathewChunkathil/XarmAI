# Graph Report - XarmAI  (2026-11-14)

## Corpus Check
- 95 files · ~90,799 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 23 file(s) not represented in the graph (top: .stl 14, .csv 3, (none) 2)

## Summary
- 1443 nodes · 3079 edges · 115 communities (70 shown, 45 thin omitted)
- Extraction: 91% EXTRACTED · 9% INFERRED · 0% AMBIGUOUS · INFERRED: 291 edges (avg confidence: 0.86)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `71129193`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- rank
- demo_sim_vision.py
- SimWorld
- intent.py
- kin_log.py
- Project Rules & Core Decisions
- Hand-Eye Calibration & Kabsch
- Cube Model YAML Configuration
- AprilTag Fiducial Assets
- Simulated C270 Camera Config
- workspace.yaml
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
- tts.py
- cv2
- TaskRunner
- Conveyor
- generate
- parse
- report_pdf.py
- trajectory.py
- test_kin_log.py
- Pipeline
- VoiceInput
- .cube_pose_true
- sim_demo.py
- task_runner.py
- .submit_command
- ai.yaml
- candidate_evaluator/__init__.py
- conveyor
- cube_perception/__init__.py
- pyb_world.py
- candidate_evaluator/__init__.py
- quat_from_R
- math
- place_zones
- main
- .take_shots
- .interactive_loop
- test_candidates.py
- math
- ._run_one
- .pump
- .calib_worker
- xArm 5 AI-Assisted Autonomous Manipulation
- cubes
- place_zones
- evaluate
- BeltOps
- .run_calibrations
- install.ps1
- sim_app.py
- test_cube_model.py
- PyBulletIK
- pyb_world.py
- .tee
- place_target
- test_pnp_single_tag_gate.py
- load_limits
- analytic_cartesian
- lane_box
- cube_workspace_box
- place_zone
- table_box
- Tint measurement (SIMULATION ONLY)
- test_cube_set.py
- analytic_twist
- edge_pose_gate.md
- conveyor_summary.md
- sim_conveyor/summary.md
- time
- .build_ui
- .transcribe
- ._load
- test_voice.py

## God Nodes (most connected - your core abstractions)
1. `App` - 70 edges
2. `SimWorld` - 53 edges
3. `Pipeline` - 44 edges
4. `parse()` - 32 edges
5. `TaskRunner` - 29 edges
6. `generate()` - 23 edges
7. `VoiceInput` - 22 edges
8. `main()` - 21 edges
9. `C270SimCamera` - 20 edges
10. `conveyor` - 19 edges

## Surprising Connections (you probably didn't know these)
- `test_identify_groups_by_id_set()` --calls--> `identify_cubes()`  [INFERRED]
  tests/unit/test_cube_set.py → src/cube_perception/cube_perception/cube_model.py
- `Demo Seed 0 Results (10/10)` --semantically_similar_to--> `10-Run Sim Trials Summary (10/10, 0.32 mm mean place err)`  [INFERRED] [semantically similar]
  docs/DEMO_SCRIPT.md → results/sim_demo/summary.md
- `main()` --calls--> `load_cube_set()`  [INFERRED]
  scripts/measure_edge_poses.py → src/cube_perception/cube_perception/cube_model.py
- `main()` --calls--> `corners_from_pupil()`  [INFERRED]
  scripts/measure_edge_poses.py → src/cube_perception/cube_perception/pnp.py
- `main()` --calls--> `cube_yaw()`  [INFERRED]
  scripts/measure_edge_poses.py → src/cube_perception/cube_perception/pnp.py

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

## Communities (115 total, 45 thin omitted)

### Community 0 - "rank"
Cohesion: 0.22
Nodes (12): FakeIK, _clip01(), rank(), make(), _q_fn(), test_deterministic_and_tiebreak(), test_format_table(), test_missing_terms_zero_with_note() (+4 more)

### Community 1 - "demo_sim_vision.py"
Cohesion: 0.13
Nodes (21): box(), detect(), detect_all(), ee_T(), flange_target(), go_home(), ik(), kabsch() (+13 more)

### Community 2 - "SimWorld"
Cohesion: 0.11
Nodes (6): SimWorld, _yaml(), world(), world(), test_run_log_files(), world()

### Community 3 - "intent.py"
Cohesion: 0.06
Nodes (35): work(), _as_cube(), _as_int(), build_system_prompt(), classify_reply(), Context, fuzzy_correct(), IntentEngine (+27 more)

### Community 4 - "kin_log.py"
Cohesion: 0.09
Nodes (11): axis_R(), _f(), _f3(), git_short(), IndepFK, parse_urdf(), rot_angle(), rpy_of() (+3 more)

### Community 5 - "Project Rules & Core Decisions"
Cohesion: 0.08
Nodes (26): Caveman Communication Rule, CLAUDE.md Project Rules, Model Ladder & Graphify Update Policy, Sim-First Work Rule, calibration_tools Package (WP3), candidate_evaluator Package (WP5/WP6), cube_perception Package (WP3), Decision D4: Dual Extrinsic Calibration (A & B) (+18 more)

### Community 6 - "Hand-Eye Calibration & Kabsch"
Cohesion: 0.11
Nodes (14): calibrate_eye_to_hand(), _project_so3(), calibrate_checkerboard(), rigid_error(), _rotvec(), umeyama(), umeyama_ransac(), _handeye_data() (+6 more)

### Community 7 - "Cube Model YAML Configuration"
Cohesion: 0.09
Nodes (22): calib_tag, family, id, R_tcp_tag, t_tcp_tag, tag_size_m, cube_id, edge_m (+14 more)

### Community 8 - "AprilTag Fiducial Assets"
Cohesion: 0.18
Nodes (20): Phone Screen tag36h11 ID 0 Marker, Phone Screen AprilTag Fiducial Markers, Phone Screen tag36h11 ID 1 Marker, Simulation tag36h11 ID 0 Marker, Simulation AprilTag Fiducial Markers, Simulation tag36h11 ID 11 Marker, Simulation tag36h11 ID 12 Marker, Simulation tag36h11 ID 13 Marker (+12 more)

### Community 9 - "Simulated C270 Camera Config"
Cohesion: 0.11
Nodes (18): blur_sigma_px, brightness_jitter, diag_fov_deg, distortion, far_m, focus, height, mjpeg_quality (+10 more)

### Community 10 - "workspace.yaml"
Cohesion: 0.22
Nodes (8): forbidden_zones, max_stack_height, place_slot_spacing_m, place_tol_m, ready_joint_state_rad, schema_version, stack_release_gap_m, table_top_z_m

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
Cohesion: 0.12
Nodes (16): backend, command_timeout_s, feedback_available, finger_footprint, finger_bottom_above_tcp_m, finger_half_width_m, finger_outer_half_m, open_inner_half_gap_m (+8 more)

### Community 16 - "5-DoF Grasp Candidate Planning"
Cohesion: 0.22
Nodes (7): candidate_evaluator, grasp_planning, Candidate Ranking Cost Function, Constrained Top-Down Grasp Family, 06 Grasp and Motion Planning, Grasp Candidate Record, Simulation-in-the-Loop Validation

### Community 17 - "Execution, Safety & Recovery"
Cohesion: 0.25
Nodes (8): execution_manager, verification_recovery, 08 Execution, Safety and Recovery, 12-Stage Physical Execution Sequence, Independent Grasp and Placement Verification, Mandatory Stop Conditions, System Operating Modes, Deterministic Pre-Execution Gate

### Community 18 - "Motion Planning YAML Config"
Cohesion: 0.14
Nodes (13): approach_distance_m, grasp_height_offset_m, lift_distance_m, max_acceleration_scaling, max_velocity_scaling, schema_version, sim_cartesian_peak_speed_m_s, sim_gripper_time_s (+5 more)

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

### Community 38 - "tts.py"
Cohesion: 0.38
Nodes (3): available(), speak(), stop()

### Community 43 - "cv2"
Cohesion: 0.15
Nodes (6): main(), load_cfg(), test_headless_command_run(), test_unknown_cube_in_command_exits_2(), test_sim_demo_disconnect_partial(), test_sim_demo_headless_two_runs()

### Community 45 - "Conveyor"
Cohesion: 0.08
Nodes (3): Conveyor, ConveyorCfg, world()

### Community 46 - "generate"
Cohesion: 0.19
Nodes (13): generate(), filter_neighbors(), Obstacle, reasons(), test_evaluate_skips_neighbor_rejected(), test_extra_boxes_allow_zone_pick(), test_far_neighbor_never_hit(), test_generate_marks_neighbor_collision() (+5 more)

### Community 47 - "parse"
Cohesion: 0.06
Nodes (37): _cube_list_join(), is_cube_end(), extract_speed(), finalize(), _interpret(), _is_num(), normalize(), _order() (+29 more)

### Community 48 - "report_pdf.py"
Cohesion: 0.07
Nodes (26): main(), build_report(), _Doc, _floats(), _fmt(), git_short(), _grab(), load_calib() (+18 more)

### Community 49 - "trajectory.py"
Cohesion: 0.16
Nodes (14): dds_q(), ds_q(), Is_q(), joint_steps(), plan_gripper(), plan_joint(), plan_line(), fallback() (+6 more)

### Community 50 - "test_kin_log.py"
Cohesion: 0.14
Nodes (9): test_tracking_segment_fails_cleanly_when_unreachable(), test_tracking_segment_matches_its_equation_and_qd_is_consistent(), test_ik_top_down(), _rand_q(), test_cartesian_line_is_straight_and_within_limits(), test_cartesian_qd_qdd_analytic_and_smooth(), test_forward_dynamics_round_trip(), test_limits_from_urdf_and_planning() (+1 more)

### Community 53 - "VoiceInput"
Cohesion: 0.19
Nodes (3): VoiceInput, test_no_microphone_is_handled(), test_status_text_before_load()

### Community 54 - ".cube_pose_true"
Cohesion: 0.16
Nodes (7): test_attached_cube_is_not_carried_by_the_belt(), test_stopped_belt_does_not_move_cubes_and_cleats_stay(), test_each_cube_shows_only_its_own_ids(), test_legacy_single_cube_unchanged(), test_stack_rests_on_lower_cube(), test_three_cubes_coexist_and_rest_on_table(), test_cube_rests_on_table()

### Community 55 - "sim_demo.py"
Cohesion: 0.18
Nodes (14): fmt(), is_disconnect_error(), log(), main(), parse_cubes_arg(), parse_layout_arg(), print_task_table(), run_conveyor() (+6 more)

### Community 56 - "task_runner.py"
Cohesion: 0.17
Nodes (6): lim_dt(), Tinv(), rz(), T_of(), TaskError, wrap90()

### Community 58 - "ai.yaml"
Cohesion: 0.29
Nodes (6): confirm_text_rules, confirm_voice, model, provider, timeout, url

### Community 60 - "candidate_evaluator/__init__.py"
Cohesion: 0.24
Nodes (7): _axes(), cube_rect(), finger_rects(), neighbor_hit(), rects_overlap(), test_finger_boxes_follow_closing_axis(), test_rects_overlap_basic()

### Community 61 - "conveyor"
Cohesion: 0.11
Nodes (18): conveyor, belt_height_m, cleat_pitch_m, direction, end_stop_thickness_m, feed_cross_m, feed_s_m, hover_s (+10 more)

### Community 62 - "cube_perception/__init__.py"
Cohesion: 0.07
Nodes (36): measure(), wrap90(), CubeModel, identify_cubes(), load_cube_model(), load_cube_set(), load_cubes_cfg(), tag_corners_in_tag_frame() (+28 more)

### Community 63 - "pyb_world.py"
Cohesion: 0.15
Nodes (4): _tag_tile(), _tint_tile(), _write_obj(), test_tint_is_pastel_quiet_zone_full_colour_bottom()

### Community 64 - "candidate_evaluator/__init__.py"
Cohesion: 0.24
Nodes (6): format_table(), motion_raw(), RankResult, _raw_terms(), GraspCandidate, ScoreBreakdown

### Community 65 - "quat_from_R"
Cohesion: 0.20
Nodes (3): quat_from_R(), box(), zone()

### Community 66 - "math"
Cohesion: 0.22
Nodes (8): _in_zone(), load_yaml(), pose_to_matrix(), quat_xyzw_to_rot(), rot_to_quat_xyzw(), tool_down_rotation(), wrap_pi(), Pose

### Community 67 - "place_zones"
Cohesion: 0.12
Nodes (10): get_zone(), min_cube_spacing(), place_zones(), zone_of_point(), zone_slots(), tool_yaw_of(), test_random_layout_spacing(), test_zone_slots() (+2 more)

### Community 68 - "main"
Cohesion: 0.24
Nodes (3): main(), C270SimCamera, test_lane_visible_with_margin_in_the_true_camera()

### Community 72 - "test_candidates.py"
Cohesion: 0.28
Nodes (11): gen(), test_evaluate_no_ik_and_joint_limit(), test_gripper_width_rejected(), test_ids_and_count(), test_ok_cube_generated_with_widths(), test_out_of_workspace_rejected(), test_pregrasp_above_grasp(), test_pregrasp_below_table_rejected() (+3 more)

### Community 73 - "math"
Cohesion: 0.20
Nodes (9): rot_deg(), rot_err_rad(), cam(), cube_frame(), _tag_R_world(), test_calib_tag_visible(), test_checkerboard_found(), test_detect_ids_only_model_ids() (+1 more)

### Community 75 - ".pump"
Cohesion: 0.16
Nodes (3): refine(), solve(), main()

### Community 77 - "xArm 5 AI-Assisted Autonomous Manipulation"
Cohesion: 0.17
Nodes (11): 1. Project in one paragraph, 2. Version 1 boundaries, 3. Important coordinate and calibration distinction, 4. Documentation map, 5. Engineering rules, 6. Suggested first milestone, 7. Simulation app quick start, Conveyor (moving-cube pick, simulation only) (+3 more)

### Community 78 - "cubes"
Cohesion: 0.20
Nodes (9): cubes, color_rgb, face_ids, name, number, schema_version, tint, edge_band_frac (+1 more)

### Community 79 - "place_zones"
Cohesion: 0.20
Nodes (10): place_zones, color_rgb, name, number, target, x_m, y_m, x_m (+2 more)

### Community 80 - "evaluate"
Cohesion: 0.22
Nodes (3): evaluate(), IKBackend, joint_margin()

### Community 82 - ".run_calibrations"
Cohesion: 0.15
Nodes (4): config_hash(), load(), save(), _ser()

### Community 86 - "test_cube_model.py"
Cohesion: 0.48
Nodes (5): load(), test_corners_lie_on_face(), test_ids_unique_and_five_faces(), test_rotations_and_offsets(), test_tag_fits_face()

### Community 89 - "pyb_world.py"
Cohesion: 0.17
Nodes (14): circ_mean_yaw(), fit_track(), ray_plane_xy(), boxes_overlap(), synth(), test_belt_moves_cube_exactly_v_dt_per_step_and_keeps_yaw(), test_cleats_step_with_the_belt(), test_config_lane_is_separate_from_all_zones() (+6 more)

### Community 92 - "place_target"
Cohesion: 0.50
Nodes (4): place_target, x_m, y_m, yaw_rad

### Community 93 - "test_pnp_single_tag_gate.py"
Cohesion: 0.31
Nodes (5): _central(), _cube_in_cam(), _project(), _setup(), test_gate_accepts_good_and_rejects_bad_single_tag()

### Community 96 - "lane_box"
Cohesion: 0.67
Nodes (3): lane_box, x_m, y_m

### Community 97 - "cube_workspace_box"
Cohesion: 0.67
Nodes (3): cube_workspace_box, x_m, y_m

### Community 98 - "place_zone"
Cohesion: 0.67
Nodes (3): place_zone, x_m, y_m

### Community 99 - "table_box"
Cohesion: 0.67
Nodes (3): table_box, center_xyz_m, size_xyz_m

### Community 101 - "test_cube_set.py"
Cohesion: 0.27
Nodes (8): make_T(), project(), rot(), test_estimate_cubes_per_cube_pose(), test_geometry_shared_across_cubes(), test_identify_groups_by_id_set(), test_missing_cube_not_reported(), test_to_base_stack_level_snapping()

### Community 106 - "time"
Cohesion: 0.24
Nodes (5): flange_target(), ik(), move(), set_grip(), move()

### Community 107 - ".build_ui"
Cohesion: 0.28
Nodes (3): button_theme(), make_theme(), sc()

## Knowledge Gaps
- **61 isolated node(s):** `SIM_DEMO_PLAN.md (v2)`, `digital_twin_interface`, `xarm_app_bringup`, `Physical Commissioning Sequence`, `Simulation Fidelity Levels 1-3` (+56 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 568 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **45 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `App` connect `.submit_command` to `.handle`, `.take_shots`, `.build_ui`, `.pump`, `.__init__`, `sim_app.py`, `.tee`, `.calibrated`?**
  _High betweenness centrality (0.070) - this node is a cross-community bridge._
- **Are the 5 inferred relationships involving `SimWorld` (e.g. with `Pipeline` and `Conveyor`) actually correct?**
  _`SimWorld` has 5 INFERRED edges - model-reasoned connections that need verification._
- **What connects `SIM_DEMO_PLAN.md (v2)`, `digital_twin_interface`, `xarm_app_bringup` to the rest of the system?**
  _61 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `demo_sim_vision.py` be split into smaller, more focused modules?**
  _Cohesion score 0.12923076923076923 - nodes in this community are weakly interconnected._
- **Why does `Pipeline` connect `Pipeline` to `SimWorld`, `main`, `.handle`, `.interactive_loop`, `._run_one`, `.run_calibrations`, `sim_demo.py`, `.tee`, `.run_logged`?**
  _High betweenness centrality (0.055) - this node is a cross-community bridge._
- **Should `SimWorld` be split into smaller, more focused modules?**
  _Cohesion score 0.11494252873563218 - nodes in this community are weakly interconnected._
- **Why does `SimWorld` connect `SimWorld` to `quat_from_R`, `main`, `math`, `cv2`, `Conveyor`, `test_kin_log.py`, `Pipeline`, `.cube_pose_true`, `sim_demo.py`, `pyb_world.py`, `pyb_world.py`?**
  _High betweenness centrality (0.043) - this node is a cross-community bridge._