# IMPLEMENTATION PLAN — xArm 5 AI-Assisted Autonomous Manipulation

**Status:** v1.0 (2026-11-11) · **Derived from:** `README.md`, `01`–`11` docs, operator Q&A, web research (2026-11-11)
**Audience:** any developer or coding agent. Every work package (WP) has an ID, inputs, outputs, files, acceptance tests, and dependencies, so you can pick up one WP without reading the whole project history.

---

## 0. How to use this document (read first)

### 0.1 Rules for every agent / developer
1. **Read before you build:** this file §0–§5, then only the source docs your WP lists under *Refs*.
2. **Do not invent hardware facts.** Gripper model, TCP, joint limits, camera intrinsics, extrinsics, table dimensions, IPs: if a value is not measured and recorded in `config/`, use a clearly marked placeholder (`# PLACEHOLDER: unmeasured`) and add it to `docs/OPEN_ITEMS.md`.
3. **Never write code that sends motion to the real robot outside `execution_manager`.** No other package may import `xarm` SDK motion calls or publish to the real controller's trajectory topics. A CI grep check enforces this (WP1.4).
4. **Reuse vendor packages** (§2) as external dependencies via `vcs` (`deps.repos`). Do not copy or fork them into `src/` unless a WP explicitly says so.
5. **Typed interfaces only** between nodes (§4). No JSON-in-a-string for safety-relevant data.
6. **Tunables go in YAML** under `config/`. No magic numbers in source.
7. **Every WP ends with tests passing** (`colcon test`) plus the WP's acceptance checks, and a package `README.md` that states its inputs, outputs, launch and test commands.
8. **Hardware tests are opt-in:** put them in `tests/hardware/` and mark them `@pytest.mark.hardware`. They never run in default CI.
9. **Change control:** if your WP changes a decision, update `11_DECISIONS_AND_OPEN_QUESTIONS.md` (see §11).

### 0.2 Definition of Done (per WP)
- [ ] Code is in the right package and builds with `colcon build --symlink-install`.
- [ ] Unit tests exist for pure logic (transforms, scoring, gate checks, FSM transitions).
- [ ] The WP's acceptance checks pass, and evidence is saved (log or screenshot) under `results/wp<ID>/`.
- [ ] The package README is updated, and config files are documented with units.
- [ ] No hardware facts are fabricated, and open items are logged.

### 0.3 Tags used in this plan
- `[SIM]`: can be done entirely on a laptop with no robot.
- `[HW]`: needs a lab session with the physical xArm. These are batched in §8.
- `[CAM]`: needs only the C270 webcam (no robot).
- `[DEMO]`: part of the 13 Nov 2026 review track (§6).

---

## 1. Decisions (confirmed in Q&A + research) and corrections to the source docs

| # | Topic | Decision | Source |
|---|---|---|---|
| D1 | Gripper | **Unidentified.** No product named "AG1010" exists from UFACTORY or any other vendor I could find. Likely candidates: the UFACTORY xArm Gripper (G1/G2), or a DH-Robotics AG-series gripper over RS-485 Modbus. The operator will supply a photo. Build against an **abstract gripper interface** with swappable backends (WP4). Until it is identified, simulation uses the **UFACTORY xArm Gripper model** (`add_gripper:=true`) as a stand-in, labelled as such. | Q&A, research |
| D2 | Hardware access | **Lab sessions only.** Simulation-first. Hardware work is batched into scripted lab sessions (§8). | Q&A |
| D3 | LLM | **Pluggable provider** behind one interface (Claude API / OpenAI-compatible / Ollama local) plus a **rule-based fallback** that needs no LLM. | Q&A |
| D4 | Extrinsic calibration | **Implement both methods and compare them:** (A) a tag on the gripper with N robot poses, solved with Kabsch/Umeyama, plus OpenCV `calibrateHandEye` as a cross-check; (B) a TCP touch-point survey. Validate both on held-out points. | Q&A |
| D5 | Cube | **Not yet made.** Use tag36h11, a 40 mm black square on each of 5 faces (top plus 4 sides), unique IDs 0–4. The bottom face is untagged. Multi-tag PnP. | Q&A, research |
| D6 | Language | **Python for most nodes.** One **C++ node** for the MoveIt Task Constructor (MTC) planner, wrapped as a ROS 2 service, because MTC Python bindings on Jazzy are unconfirmed. | Q&A, research |
| D7 | Simulated grasp | **Attach-on-close** (IFRA_LinkAttacher, or the built-in `DetachableJoint`) is the baseline. Report it honestly as "Level 2.5: kinematic attach, not contact physics". Real contact physics is an optional experiment. | Q&A, research |
| D8 | Simulator | **Gazebo Harmonic (gz-sim 8)**, not Gazebo Classic: the xarm_ros2 `jazzy` branch dropped Classic. | research |
| D9 | IK | **pick_ik** (`mode: global`, `rotation_scale: 1.0`). Grasp goals are exact top-down 6D poses with explicit yaw, which the 5-DoF xArm can reach exactly. | research |
| D10 | Task FSM | **YASMIN** (Python, ROS 2 Jazzy, web viewer). | research |
| D11 | Dashboard | **Foxglove** (`foxglove_bridge`) plus RViz, with a minimal custom operator panel (arm/abort). | research |
| D12 | Logging | **SQLite** for structured trial records plus **rosbag2 MCAP** for raw streams, linked by `run_id`. | research |
| D13 | Deadline | **13 Nov 2026 = demo/review**, not production. A 48-hour demo track (§6) precedes the full roadmap (§7). | Q&A |

**Corrections needed in the source docs** (apply in WP0.3):
- `04`, `07`, `11`: replace "AG1010" with "gripper TBD (see D1)". Replace "Gazebo compatible with Jazzy" with "Gazebo Harmonic". Note that xarm_ros2 gripper mimic joints break in gz (upstream PR #180 is unmerged).
- `03`: the package list is consolidated (§3 below).

---

## 2. Reuse — repositories and packages

Pin these in `deps.repos` (for `vcs import src/external < deps.repos`) or install with apt.

| Component | Source | Branch / package | Use |
|---|---|---|---|
| xArm driver, description, MoveIt, Gazebo | https://github.com/xArm-Developer/xarm_ros2 | `jazzy` (clone `--recursive`) | Driver `xarm_api`, `xarm_description`, `xarm_moveit_config`, `xarm_gazebo`, `xarm_planner` |
| xArm Python SDK | https://github.com/xArm-Developer/xArm-Python-SDK | pip `xarm-python-sdk` (≥1.18) | Gripper/RS-485 backend, read-only diagnostics. Motion only inside `execution_manager`. |
| MoveIt Task Constructor | https://github.com/moveit/moveit_task_constructor | apt `ros-jazzy-moveit-task-constructor-{core,capabilities,msgs,visualization}` | Pick-place pipeline, multiple ranked solutions |
| IK solver | https://github.com/PickNikRobotics/pick_ik | apt `ros-jazzy-pick-ik` | 5-DoF IK |
| Gazebo attach | https://github.com/IFRA-Cranfield/IFRA_LinkAttacher | `jazzy` (very new; fall back to the gz built-in `DetachableJoint`) | Simulated grasp |
| AprilTag | https://github.com/christianrauch/apriltag_ros | apt `ros-jazzy-apriltag-ros` | Tag corner detection (our node does the multi-tag PnP) |
| Camera | `ros-jazzy-v4l2-camera` (preferred) or `ros-jazzy-usb-cam` | apt | C270 at 1280×720 MJPG with locked exposure |
| Intrinsics | `ros-jazzy-camera-calibration` | apt | Checkerboard calibration |
| Hand-eye reference | https://github.com/marcoesposito1988/easy_handeye2 (source build) and xarm_ros2 `xarm_vision` | — | Reference / cross-check only. Our own solver is primary. |
| FSM | https://github.com/uleroboticsgroup/yasmin | apt/source, check `ros-jazzy-yasmin*` | Task orchestration |
| Dashboard | `ros-jazzy-foxglove-bridge` | apt | Operator UI |
| Templates to study (do not vendor) | MoveIt2 MTC pick-place tutorial; https://github.com/automaticaddison/mycobot_ros2 (`jazzy`); https://robgineer.github.io/cobot/offline_hand_eye | — | Patterns |
| LLM tool server reference | https://github.com/robotmcp/ros-mcp-server | — | **Reference only.** It exposes arbitrary topics. Our server exposes only a whitelist (WP9). |

**Before pinning:** run `apt-cache policy <pkg>` on the target machine and record the versions in `docs/VERSIONS.md`.

---

## 3. Target repository layout (consolidated)

The source doc `03` listed 15 packages. This plan consolidates them into 12 so the vertical slice is reachable. Each package's responsibilities are stated so they are not ambiguous.

```text
XarmAI/                                  # = ROS 2 workspace root
├── IMPLEMENTATION_PLAN.md               # this file
├── docs/                                # move 01–11 + ARCHITECTURE_OVERVIEW here (WP0.3)
│   ├── OPEN_ITEMS.md  VERSIONS.md  LAB_SESSION_LOG.md  DEMO_SCRIPT.md
├── deps.repos                           # vcs: xarm_ros2(jazzy), IFRA_LinkAttacher(jazzy), yasmin(if no apt)
├── docker/                              # Dockerfile.jazzy + compose (fallback for Windows/WSL2 hosts)
├── config/
│   ├── camera/c270.yaml                 # device, resolution, v4l2 controls
│   ├── camera/c270_intrinsics.yaml      # generated by WP3.2 (camera_info format)
│   ├── calibration/extrinsics_active.yaml   # generated by WP3.4 (schema §5.2)
│   ├── cube/cube_model.yaml             # tag IDs, sizes, face transforms (schema §5.3)
│   ├── workspace/workspace.yaml         # table, bounds, place target, safe poses (§5.4)
│   ├── gripper/gripper.yaml             # backend + limits (§5.5)
│   ├── planning/planning.yaml           # candidate params, speeds, MTC params (§5.6)
│   ├── planning/ranking.yaml            # weights + normalisers (§5.7)
│   ├── safety/gate.yaml                 # gate thresholds (§5.8)
│   └── experiment_profiles/{A,B,C,D}.yaml
├── src/
│   ├── external/                        # vcs-imported vendor repos (git-ignored)
│   ├── xarm_app_interfaces/   (ament_cmake)  msgs/srvs/actions §4
│   ├── xarm_app_description/  (ament_cmake)  xacro: xarm5 + gripper + camera mount + table; gz worlds; cube SDF
│   ├── xarm_app_bringup/      (ament_python) launch files per mode, rviz/foxglove layouts
│   ├── cube_perception/       (ament_python) cube_pose_node (multi-tag PnP), image QC
│   ├── calibration_tools/     (ament_python) intrinsics helper, extrinsics A/B solvers, validator, calibration_loader (static TF)
│   ├── gripper_interface/     (ament_python) GripperCommand action server; backends: sim | xarm_gripper | dh_modbus
│   ├── scene_manager/         (ament_python) planning-scene objects + gz cube pose sync, scene_version
│   ├── grasp_planner_mtc/     (ament_cmake, C++) PlanPickPlace service using MTC → N solutions
│   ├── candidate_evaluator/   (ament_python) candidate generation, hard filters, scoring, ranking
│   ├── sim_validator/         (ament_python) runs a plan in Gazebo, attach/detach, outcome checks
│   ├── execution_manager/     (ament_python) ExecutionGate + ExecutePlan action + verification + recovery
│   ├── task_orchestrator/     (ament_python) YASMIN FSM, RunTask action, modes
│   ├── ai_supervisor/         (ament_python) LLM provider abstraction + whitelisted tools + rule fallback
│   └── experiment_logger/     (ament_python) SQLite writer, bag control, report scripts
├── scripts/                             # lab-session scripts, data analysis
├── tests/{unit,integration,simulation,hardware}/
├── data/{calibration,experiments,sample_images}/   # git-ignored except small samples
└── results/                             # evidence per WP, plots
```

**Ownership rule:** only `execution_manager` may talk to real-robot motion interfaces. Only `gripper_interface` may talk to gripper hardware, and only when `execution_manager` calls it in hardware mode.

---

## 4. Interfaces (`xarm_app_interfaces`) — the contract between WPs

Implement these in WP1.2 **before** any parallel work starts. Field names are binding. Add new fields only at the end of a message.

### 4.1 Messages
```text
# CubePose.msg
std_msgs/Header header                 # frame_id = "link_base"; stamp = image capture time
string  task_id
bool    valid
string  invalid_reason                 # "" if valid; enum-like: NO_TAGS|UNKNOWN_ID|INCONSISTENT|HIGH_REPROJ|STALE|OUT_OF_WORKSPACE|NO_CALIBRATION|TF_FAIL
geometry_msgs/PoseWithCovariance pose  # cube centre in link_base
float64 yaw_rad                        # cube yaw folded to [-pi/4, pi/4) (90° symmetry)
int32[] tag_ids_used
float64 reprojection_rms_px
string  calibration_id                 # sha256[:12] of extrinsics file

# ScoreBreakdown.msg
string[]  names                        # ["motion","clearance","kinematic","grasp","time"]
float64[] raw
float64[] normalised                   # in [0,1]
float64[] weights
float64   total                        # sum(w*norm); LOWER is better

# GraspCandidate.msg
string  task_id
string  candidate_id                   # "<task_id>-c<NN>"
geometry_msgs/PoseStamped grasp_pose   # TCP pose, link_base
geometry_msgs/PoseStamped pregrasp_pose
float64 grasp_yaw_rad
float64 approach_distance_m
float64 lift_distance_m
float64 gripper_open_width_m
float64 gripper_close_width_m
string  contact_faces                  # e.g. "+x/-x"
uint8   STATUS_GENERATED=0
uint8   STATUS_REJECTED=1
uint8   STATUS_PLANNED=2
uint8   STATUS_SIM_PASSED=3
uint8   STATUS_SIM_FAILED=4
uint8   STATUS_SELECTED=5
uint8   status
string  rejection_reason               # NO_IK|JOINT_LIMIT|OUT_OF_WORKSPACE|COLLISION|GRIPPER_WIDTH|PLAN_FAIL|STALE|SIM_COLLISION|SIM_DROP
ScoreBreakdown score

# PlanSegment.msg
string name                            # "to_pregrasp"|"approach"|"grasp"|"lift"|"transfer"|"place_approach"|"release"|"retreat"|"to_ready"
uint8  TYPE_ARM=0
uint8  TYPE_GRIPPER=1
uint8  type
moveit_msgs/RobotTrajectory trajectory # TYPE_ARM only
float64 gripper_width_m                # TYPE_GRIPPER only
float64 gripper_max_effort             # TYPE_GRIPPER only

# PickPlacePlan.msg
string  plan_id                        # "<candidate_id>-p<NN>"
string  candidate_id
string  task_id
bool    success
string  error                          # MoveIt/MTC error text
PlanSegment[] segments
sensor_msgs/JointState start_state
uint64  scene_version
float64 planning_time_s
float64 predicted_duration_s
float64 min_clearance_m                # -1 if unknown
string  trajectory_hash                # sha256 over serialised segments (see §5.9)

# SimResult.msg
string  plan_id
string  trajectory_hash
bool    passed
bool    motion_completed
bool    collision_detected
bool    grasp_held                     # attach-mode semantics, documented
geometry_msgs/Pose cube_final_pose     # link_base
float64 place_error_m
string  fidelity                       # "L2_motion"|"L2.5_attach"|"L3_contact"
string  notes

# GateDecision.msg
std_msgs/Header header
string   plan_id
bool     approved
string[] failed_checks
string[] passed_checks

# SystemStatus.msg  (published 2 Hz by task_orchestrator)
std_msgs/Header header
string mode                            # SIMULATION|TWIN_MONITOR|AUTONOMOUS_SUPERVISED|MANUAL
string fsm_state
string task_id
bool   robot_connected
int32  robot_error_code
bool   gripper_ok
bool   calibration_valid
bool   gate_armed
float64 joint_state_age_s
```

### 4.2 Services and actions
```text
# GetCubePose.srv
string task_id
uint32 num_frames      # average over N frames (default 10)
---
CubePose cube

# GenerateCandidates.srv
CubePose cube
---
GraspCandidate[] candidates            # includes geometric rejections

# PlanPickPlace.srv   (served by grasp_planner_mtc, C++)
GraspCandidate[] candidates            # only non-rejected
geometry_msgs/PoseStamped place_pose   # cube centre target, link_base
uint32 max_solutions_per_candidate     # default 1
---
PickPlacePlan[] plans                  # one per candidate (success=false allowed)

# ValidateInSim.srv
PickPlacePlan plan
CubePose cube
---
SimResult result

# RankCandidates.srv
GraspCandidate[] candidates
PickPlacePlan[] plans
SimResult[] sim_results                # may be empty for configs A/B
string profile                         # A|B|C|D
---
GraspCandidate[] ranked                # with score + status
string selected_plan_id                # "" if none feasible

# ArmGate.srv   (operator only, from dashboard)
bool arm
uint32 ttl_s                           # auto-disarm (default 120)
---
bool ok
string message

# ExecutePlan.action   (execution_manager)
string plan_id
---
string outcome          # VERIFIED_SUCCESS|LIKELY_SUCCESS|FAILURE|UNCERTAIN|ABORTED|SAFETY_STOP
string reason
GateDecision gate
---
string stage
float32 progress

# RunTask.action   (task_orchestrator; the only entry point used by UI and AI)
string task_type         # "pick_place_cube"
geometry_msgs/PoseStamped place_pose   # optional; empty = workspace.yaml default
string profile           # A|B|C|D
bool   execute_physical  # false = stop after ranking/sim
---
string task_id
string outcome
string summary
---
string fsm_state
string detail
```

Use `control_msgs/action/GripperCommand` (server name `/app/gripper_cmd`) for the gripper. Do not define a custom gripper action.

### 4.3 Topic / service names (namespace `/app`)
`/app/cube_pose` (CubePose, latched latest) · `/app/system_status` · `/app/gate_decision` · `/app/candidates` (debug MarkerArray) · services `/app/get_cube_pose`, `/app/generate_candidates`, `/app/plan_pick_place`, `/app/validate_in_sim`, `/app/rank_candidates`, `/app/arm_gate` · actions `/app/execute_plan`, `/app/run_task`, `/app/gripper_cmd`.

### 4.4 TF frames
`world` → `link_base` (xArm base, from xacro) → … → `link_eef` → `tcp` (gripper TCP, from gripper.yaml) · `link_base` → `camera_optical_frame` (static, from `extrinsics_active.yaml` via `calibration_loader`) · `link_base` → `cube` (published by cube_perception, for visualisation only) · `link_base` → `table_top`.

---

## 5. Configuration schemas and core algorithms

### 5.1 Camera (`config/camera/c270.yaml`)
Device `/dev/v4l/by-id/...` (not `/dev/video0`), 1280×720, MJPG, 30 fps target. Fixed v4l2 controls: `auto_exposure: 1` (manual), `exposure_time_absolute`, `white_balance_automatic: 0`, `white_balance_temperature`, `gain`, `power_line_frequency`. **Verify control names with `v4l2-ctl --list-ctrls`** and record them. The C270 is fixed-focus.

### 5.2 Extrinsics file (`config/calibration/extrinsics_<date>_<method>.yaml`, symlinked as `extrinsics_active.yaml`)
```yaml
schema_version: 1
calibration_id: auto            # sha256[:12] of this file's content minus this field
parent_frame: link_base
child_frame: camera_optical_frame
translation_m: [x, y, z]
rotation_xyzw: [qx, qy, qz, qw]
method: tag_on_gripper_kabsch | touch_points | opencv_handeye_tsai
intrinsics_file: c270_intrinsics.yaml
intrinsics_sha: ...
num_samples: 25
fit_rms_m: 0.0
validation: {num_points: 5, mean_err_m: 0.0, max_err_m: 0.0}
date: 2026-..., operator: ..., software_commit: ...
status: VALID | SUSPECT | INVALID
```
`calibration_loader` refuses to publish the TF unless `status == VALID` and the checksums match. With no valid file, it publishes nothing, and `SystemStatus.calibration_valid=false` blocks the pipeline. **Simulation mode** uses `extrinsics_sim.yaml`, which is generated from the simulated camera pose and is never used with hardware.

### 5.3 Cube model (`config/cube/cube_model.yaml`)
```yaml
edge_m: 0.050
family: tag36h11
tag_size_m: 0.040          # black-square edge, measure after printing
faces:                     # T_cube_tag: tag frame in cube frame (cube origin = centre)
  - {id: 0, face: +z, xyz: [0,0,0.025],  rpy: [0,0,0]}
  - {id: 1, face: +x, xyz: [0.025,0,0],  rpy: [0, 1.5708, 0]}   # VERIFY axis conventions with a printed cube + unit test
  - {id: 2, face: -x, ...}
  - {id: 3, face: +y, ...}
  - {id: 4, face: -y, ...}
```
WP3.1 produces a printable PDF/SVG net from this file, so the physical cube and the config cannot diverge.

### 5.4 Workspace (`config/workspace/workspace.yaml`)
Table plane (`table_top_z_m` in link_base), table box dimensions and pose, a cube workspace polygon or box in link_base, place target pose(s), `ready_joint_state`, a `forbidden_zones` list. **All are placeholders until measured in Lab Session 1.**

### 5.5 Gripper (`config/gripper/gripper.yaml`)
```yaml
backend: sim | xarm_gripper | dh_modbus
tcp_offset_xyz_m: [0,0,0.172]     # PLACEHOLDER (xArm Gripper G1 nominal); measure
tcp_offset_rpy: [0,0,0]
max_open_m: 0.085                 # PLACEHOLDER
min_close_m: 0.0
grasp_close_width_m: 0.046        # cube 0.050 minus squeeze; tune
open_margin_m: 0.015              # open width = edge + margin
command_timeout_s: 3.0
feedback_available: true          # false → verification relies on camera only
dh_modbus: {slave_id: 1, baud: 115200, reg_init: 0x0100, reg_pos: 0x0103, reg_force: 0x0101, reg_read_pos: 0x0202}  # if DH AG
```

### 5.6 Candidate generation (`candidate_evaluator`, pure Python, unit-tested)
Inputs: CubePose (x, y, z, yaw_c), gripper.yaml, planning.yaml.
- Grasp yaws: `ψ_k = yaw_c + k·90°`, k ∈ {0,1,2,3}. Optionally add `±δ` (planning.yaml `yaw_offsets_deg`, default `[0]`). The closing axis is aligned with a cube face normal. Yaw ψ and ψ+180° are kept as separate candidates because they give different J5 values and joint margins.
- TCP grasp pose: position = cube centre + `[0,0, grasp_height_offset_m]` (default 0: fingertips at the cube's mid-height). Orientation = tool z-axis pointing down (−Z of link_base), rotated by ψ about the vertical.
- Pre-grasp: grasp + `[0,0,approach_distance_m]` (default 0.10). Lift: `lift_distance_m` (default 0.08).
- Geometric pre-filter (before IK): the cube is inside the workspace, `edge + open_margin ≤ max_open`, and the pre-grasp z is above the table + clearance.
- Output: 4–8 `GraspCandidate`s with `candidate_id` values.

**5-DoF note:** a pure top-down orientation at any yaw is exactly reachable within the workspace (J1 sets the azimuth, J2–J4 set the pitch sum so the tool points down, J5 sets the yaw). So send **exact** poses to IK. Do **not** use position-only IK. If IK fails at the workspace edges, the candidate is rejected `NO_IK`. That is a valid, reportable outcome.

### 5.7 Ranking (`config/planning/ranking.yaml`)
Hard constraints first: any rejection reason ⇒ not ranked. For feasible candidates i, each term is normalised to [0,1]:
| Term | Raw | Normalised |
|---|---|---|
| motion | Σ_j w_j·Σ_t \|Δq_j\| over all arm segments | raw / max_i(raw) |
| clearance | min distance to obstacles along the plan (m) | 1 − min(raw / d_ref, 1), d_ref = 0.05 |
| kinematic | min over joints and waypoints of the normalised joint-limit margin m ∈ [0, 0.5] | 1 − raw/0.5 |
| grasp | \|closing-axis angle − nearest face normal\| (rad) + pose covariance trace term | raw / (π/4) |
| time | predicted duration (s) | raw / max_i(raw) |
`total = Σ w·norm`, lowest wins. Defaults: all w = 0.2 (experimental parameters; tuned only with recorded data). Ties are broken by `candidate_id`, so ranking is deterministic. Profile A bypasses ranking and uses the fixed candidate k=0.

### 5.8 Execution gate (`config/safety/gate.yaml`) — all checks are hard
`robot_connected`, `robot_error_code == 0`, `robot_mode == expected`, `joint_state_age_s < 0.2`, `gripper_state_known`, `calibration_valid`, `cube_pose_age_s < 2.0`, `cube_in_workspace`, `plan.scene_version == current`, `‖plan.start_state − current_q‖∞ < 0.01 rad`, `trajectory_hash ∈ sim_passed_hashes` (profiles C/D) **or** `profile ∈ {A,B}` with a planning-only flag, `max_velocity_scaling ≤ 0.2` and `max_acceleration_scaling ≤ 0.2` (commissioning), `gate_armed` (operator `ArmGate`, TTL), and `mode == AUTONOMOUS_SUPERVISED`. Every decision is published and logged with its passed and failed lists.

### 5.9 Trajectory hash
sha256 over the canonical serialisation of the segments (joint names, positions rounded to 1e-6, time_from_start rounded to 1e-4, gripper widths). It is computed in one shared function, `xarm_app_common.hash.plan_hash()`, a Python module in `execution_manager` that is imported by both sim_validator and the gate. The C++ planner leaves the field empty; `candidate_evaluator` fills it in.

### 5.10 Verification (`execution_manager.verify`)
- After close: if feedback is available, a gripper width in `[edge−tol_low, edge+tol_high]` ⇒ "holding". A width near fully closed ⇒ "missed".
- After the small lift (2 cm): the camera shows the cube's z increased, or the cube tags are not visible at the table position ⇒ evidence. If the evidence conflicts ⇒ `UNCERTAIN` ⇒ stop and ask the operator.
- After place: re-detect the cube. `place_error_m < place_tol_m` ⇒ `VERIFIED_SUCCESS`.
- Outcome table and thresholds live in `gate.yaml`.

### 5.11 Recovery (bounded)
`max_retries: 2` (config). Allowed: re-perceive (stale pose), re-plan (state changed), next-ranked candidate (only after a verified miss with the cube re-detected in the workspace). Anything else ⇒ `ABORTED` plus an operator request.

---

## 6. DEMO TRACK — review on 13 Nov 2026 (≈2 days)

> **UPDATE 2026-11-12:** the demo is now ALL-SIM (PyBullet, sim C270 camera, sim calibration, 10 runs). See `SIM_DEMO_PLAN.md`. It overrides DEMO-1, DEMO-2 and DEMO-5 below.

Goal: show a credible, honest vertical slice **in simulation**, plus live AprilTag detection on the real webcam, plus this plan. Nothing in the demo touches the real robot.

| ID | Task | Output | Time |
|---|---|---|---|
| DEMO-1 `[SIM]` | Environment. Use native Ubuntu 24.04 + Jazzy if available. On Windows, use **WSL2 Ubuntu 24.04 + WSLg** or the `docker/Dockerfile.jazzy` (osrf/ros:jazzy-desktop-full). Install MoveIt, ros_gz, pick_ik, MTC, apriltag_ros, v4l2_camera. Clone xarm_ros2 `-b jazzy --recursive`, then `rosdep install`. | `docs/VERSIONS.md` | 3–4 h |
| DEMO-2 `[SIM]` | Launch `ros2 launch xarm_moveit_config xarm5_moveit_gazebo.launch.py add_gripper:=true` and plan/execute a joint goal from RViz. If gz is unstable on WSL, fall back to `xarm5_moveit_fake.launch.py` (RViz only). | Screen recording | 1–2 h |
| DEMO-3 `[SIM]` | Add a table box and a 50 mm cube to the planning scene (a quick Python script using `moveit_py` or the `/apply_planning_scene` service) and to the gz world (cube SDF). | `scripts/demo_scene.py` | 2 h |
| DEMO-4 `[SIM]` | Scripted top-down pick-place from a **hard-coded cube pose**: 4 yaw candidates → IK check → plan via `xarm_planner` or MoveIt → execute in gz. Use LinkAttacher/DetachableJoint for the grasp if it can be set up within 2 h; otherwise attach the collision object in MoveIt only, and say so. Print the candidate table (feasible/rejected + simple cost). | `scripts/demo_pick_place.py`, video | 4–6 h |
| DEMO-5 `[CAM]` | Run the C270 with `v4l2_camera` and `apriltag_ros` (tag36h11). Show tag poses in RViz. Use a quick intrinsic calibration with a printed checkerboard if time allows; otherwise use a default camera_info marked UNCALIBRATED. | Video / screenshot | 2–3 h |
| DEMO-6 | Slides: problem, architecture (from `02`), this plan's phases, research findings (§1–§2), risks (gripper ID, 5-DoF, sim fidelity), what the demo does and does not prove. | `docs/DEMO_SCRIPT.md` | 2 h |

**Honesty labels for the demo:** "simulation only"; "grasp = kinematic attach"; "gripper model = xArm Gripper stand-in"; "camera not yet calibrated to the robot".
**Cut order if short on time:** DEMO-5 calibration → the gz attach in DEMO-4 → gz itself (use fake mode). Keep DEMO-2, DEMO-4 (RViz) and DEMO-6.

Demo scripts are throwaway prototypes in `scripts/demo_*`. The full WPs re-implement them properly; do not build on them.

---

## 7. Full roadmap — work packages

Dependency graph (→ = "required before"):
```text
WP0 → WP1 → WP2 ─┬─→ WP5 → WP6 → WP7 ─┐
                 │                     ├─→ WP8 → WP10(exp) → WP11
WP1 → WP3(cam) ──┴─→ WP4(grip) ────────┘        ↑
WP1 → WP12(logger) ─────────────────────────────┤
WP8 → WP9(AI) ──────────────────────────────────┘
```
Phase mapping to source doc `10`: P0=WP0, P1=WP2+WP4(HW), P2=WP2, P3=WP3, P4=WP5+WP6, P5=WP7, P6=WP8, P7=WP9+WP12 UI, P8=WP10+WP11. Duration: 16 weeks after the demo, adjusted to lab availability.

---

### WP0 — Workspace, tooling, facts register `[SIM]`
**Refs:** 03, 11. **Deps:** none.
- 0.1 Run `git init`. Add `.gitignore` (build/, install/, log/, src/external/, data/ except samples) and a `deps.repos` file (§2).
- 0.2 Add a `docker/Dockerfile.jazzy` that reproduces the full environment, plus `scripts/setup_env.sh`.
- 0.3 Move the docs into `docs/` and apply the doc corrections from §1. Create `OPEN_ITEMS.md` (seeded from doc 11 §2 plus D1), `VERSIONS.md` and `LAB_SESSION_LOG.md`.
- 0.4 CI: a GitHub Actions job running `ros:jazzy` with colcon build + test (skipping hardware tests) and `ruff`/`clang-format` checks.
**Acceptance:** a fresh clone, then `vcs import`, `rosdep install` and `colcon build`, succeeds in Docker.

### WP1 — Interfaces and common utilities `[SIM]`
**Refs:** §4, §5.9. **Deps:** WP0.
- 1.1 Create the `xarm_app_interfaces` package with every msg/srv/action in §4, exactly as written.
- 1.2 Create `xarm_app_common` (a python module inside `execution_manager`, or a separate small package) containing `plan_hash()`, YAML loaders with schema validation (pydantic or jsonschema), `task_id`/`candidate_id` generators, and time-freshness helpers.
- 1.3 Unit tests: hash stability (same plan ⇒ same hash; a change of 1e-5 rad ⇒ a different hash), and config validation rejects missing fields.
- 1.4 A CI guard script, `scripts/check_motion_isolation.sh`: fails if `set_servo_angle|set_position|set_mode|/joint_trajectory_controller|FollowJointTrajectory` appear outside `src/execution_manager/` and `src/gripper_interface/`.
**Acceptance:** `ros2 interface show xarm_app_interfaces/msg/PickPlacePlan` works, and the tests pass.

### WP2 — Simulation baseline: description + Gazebo + MoveIt `[SIM]`
**Refs:** 04 §4, 07. **Deps:** WP1.
- 2.1 `xarm_app_description`: a top-level xacro that includes xarm_ros2's xArm 5 (`dof:=5`) with the gripper (stand-in: `add_gripper:=true`), a table, a camera mount link and `camera_optical_frame` (sim pose from `extrinsics_sim.yaml`), and a fixed `tcp` frame from gripper.yaml.
- 2.2 gz world `worlds/tabletop.sdf`: table, cube model (5 tag textures optional), a simulated camera (gz camera sensor bridged to `/sim/camera/image_raw`, `camera_info`), and the LinkAttacher (or DetachableJoint) plugin.
- 2.3 `xarm_app_bringup/launch/sim.launch.py`: gz + robot spawn (use the vendor's `xarm5_moveit_gazebo` launch internally, or replicate its args with `gz_type:=gz`), move_group with **pick_ik** (override `kinematics.yaml`: `kinematics_solver: pick_ik/PickIkPlugin`, `mode: global`, `rotation_scale: 1.0`, `position_threshold: 0.001`, `orientation_threshold: 0.01`), RViz, foxglove_bridge.
- 2.4 Gripper in sim: check whether the mimic-joint linkage breaks (upstream PR #180). If it does, apply the PR's change locally as a documented patch in `src/external_patches/`, or make the sim gripper a simple 2-prismatic-finger model in `xarm_app_description`, and document which.
- 2.5 Tests (`tests/simulation/`): joint count = 5 with the limits from the URDF; plan and execute a joint goal in gz; a top-down pose at 3 workspace points has an IK solution; frames match between MoveIt and gz (the `link_eef` pose agrees within 1 mm after a move).
**Acceptance:** `ros2 launch xarm_app_bringup sim.launch.py` starts everything in under 60 s, and the WP2.5 tests pass headless (`gz sim -s`).

### WP3 — Vision and calibration `[CAM]` + `[HW]` for extrinsics
**Refs:** 05. **Deps:** WP1 (WP2 for sim testing).
- 3.1 **Cube artefact:** `calibration_tools/make_cube_net.py` generates a printable cube net (tag36h11 IDs 0–4, 40 mm, with crop marks) from `cube_model.yaml`, plus a single 60 mm calibration tag (ID 10) for the gripper. Print, assemble, measure the real tag size, update the config.
- 3.2 **Intrinsics:** a launch wrapper for `camera_calibration cameracalibrator` (checkerboard, e.g. 9×6 inner corners × 25 mm). Save to `c270_intrinsics.yaml`. Acceptance: RMS < 0.5 px at 1280×720. If RMS ≥ 0.5, redo it and document the result.
- 3.3 **cube_pose_node** (`cube_perception`):
  - subscribe to `apriltag_msgs/AprilTagDetectionArray` + `camera_info`;
  - keep only the IDs in `cube_model.yaml`; for each tag, build 4 corner 3D points in the cube frame (via `T_cube_tag`);
  - stack all corners → `cv2.solvePnP(SOLVEPNP_SQPNP)` → `solvePnPRefineLM`;
  - reject if `reprojection_rms_px > max_reproj_px` (config) or if fewer than 1 tag is used;
  - transform to `link_base` via tf2; optional table-plane constraint: project z to `table_top_z + edge/2`, and roll/pitch to the nearest face-down orientation, recomputing yaw;
  - fold yaw to [−45°, 45°); temporal median over the N frames requested by `GetCubePose`;
  - covariance from the spread across frames;
  - checks: freshness, workspace bounds, calibration present ⇒ `CubePose.valid/invalid_reason`;
  - unit tests with synthetic projections (known pose → project corners → solve → error < 1e-6), including 1-tag, 2-tag and occluded cases.
- 3.4 **Extrinsics method A (tag on gripper)** `[HW]`: `calibration_tools/collect_extrinsics_A.py` moves (through execution_manager, in a dedicated `CALIBRATION` sub-mode with reduced speed) or prompts the operator to jog through 20–30 poses from `config/calibration/poses_A.yaml`. At each pose it records the tag centre in the camera frame (averaged over 30 frames) and the TCP pose. The solver `solve_kabsch.py` uses Umeyama without scale, with RANSAC outlier rejection; the tag offset in the TCP frame is measured, or estimated jointly with `cv2.calibrateRobotWorldHandEye`. Cross-check with `cv2.calibrateHandEye` (eye-to-hand inversion).
- 3.5 **Extrinsics method B (touch points)** `[HW]`: put a printed sheet with ≥ 6 fiducial points (tag corners) on the table; the operator jogs the TCP pointer to each point and records it; the camera detects the same points; solve with Kabsch.
- 3.6 **Validation and comparison:** for each method, 5 held-out points across the workspace (touch with the TCP, compare with the camera prediction). Report mean/max error and the time to perform. `results/wp3/extrinsics_comparison.md`. Pick the active file.
- 3.7 **calibration_loader** node: loads `extrinsics_active.yaml`, verifies the checksum and status, and publishes a static TF; exposes `calibration_valid` and `calibration_id`; a CLI `mark_suspect` sets `status: SUSPECT`.
- 3.8 **Pose accuracy study** `[HW]` (feeds doc 09): cube at ≥ 9 measured positions × 4 yaws; report position error (mm) and yaw error (deg); derive `max_reproj_px` and the covariance thresholds from the data.
**Acceptance:** WP3.3 tests pass; in sim, cube_pose from the simulated camera agrees with gz ground truth within 5 mm / 2°; on hardware, the extrinsics validation error is reported (no threshold is invented: it sets the grasp tolerance budget).

### WP4 — Gripper interface `[SIM]` + `[HW]`
**Refs:** 04 §3. **Deps:** WP1. **Blocked on D1 for the hardware backend.**
- 4.1 `gripper_interface` node: a `control_msgs/GripperCommand` action server `/app/gripper_cmd` and a `/app/gripper_state` (JointState-like width + `ok`) topic. A backend plugin is selected by `gripper.yaml: backend`.
- 4.2 The `sim` backend forwards to the gz gripper controller and calls the LinkAttacher attach/detach (attach when the commanded width is < edge + tol and the fingers are near the cube).
- 4.3 The `xarm_gripper` backend uses xArm SDK `set_gripper_enable/mode/position` and `get_gripper_position` (or the xarm_api ROS services), and maps metres ↔ SDK units (calibrate the mapping in a lab session).
- 4.4 The `dh_modbus` backend uses xArm SDK `set_rs485_baudrate` + `set_rs485_data` (Modbus RTU frames per the DH AG manual and UFACTORY's AG-160-95 article), with init/position/force/read-position registers from the config. **Implement it only if the photo confirms a DH AG.**
- 4.5 Lab: measure open/closed widths, the command → width mapping, finger symmetry and the TCP offset (calliper + touch test). Update `gripper.yaml` and the sim model.
**Acceptance:** in sim, open/close plus attach works and the gripper state is reported. On hardware, the measured mapping table is recorded and a commanded width of 46 mm gives a measured width within a tolerance recorded from data.

### WP5 — Candidate generation and geometric/IK filtering `[SIM]`
**Refs:** 06 §2–4, §5.6. **Deps:** WP1, WP2.
- 5.1 `candidate_evaluator.generate` is a pure function from cube to candidates, per §5.6. It has property tests: the TCP z-axis points down within 1e-9 for all candidates, the yaws are spaced by 90°, and the pre-grasp is above the grasp.
- 5.2 IK pre-check: call `/compute_ik` (move_group) per candidate for the pre-grasp, grasp and lift poses with collision checking on. Reject with `NO_IK`/`COLLISION`. Record the joint solutions and margins.
- 5.3 The `/app/generate_candidates` service. RViz MarkerArray (green = ok, red = rejected, with text labels).
**Acceptance:** for 20 random valid cube poses in sim, the candidates are generated, and reachable poses get ≥ 1 IK-feasible candidate. Edge-of-workspace poses yield correct rejections, logged with reasons.

### WP6 — MTC pick-place planner (C++) and ranking `[SIM]`
**Refs:** 06 §5–8, §5.7. **Deps:** WP5.
- 6.1 The `grasp_planner_mtc` C++ node serves `/app/plan_pick_place`. For each candidate it builds an MTC task: `CurrentState` → `Connect(to_pregrasp)` → `MoveRelative(approach)` → gripper close (`MoveTo` on the hand group, or an emitted `TYPE_GRIPPER` segment) → `ModifyPlanningScene(attach cube)` → `MoveRelative(lift)` → `Connect(transfer)` → `MoveRelative(place_approach)` → open → detach → `MoveRelative(retreat)` → `MoveTo(ready)`. Use a **fixed grasp pose from the candidate** (`FixedCartesianPoses` or `GeneratePose` + `ComputeIK`) instead of MTC's own `GenerateGraspPose` sampling, so candidate IDs stay traceable.
- 6.2 Planners: Pipeline (OMPL RRTConnect) for Connect, Cartesian for the MoveRelative stages. Velocity/acceleration scaling comes from `planning.yaml`.
- 6.3 Convert the MTC solution into `PickPlacePlan.segments` (do **not** use MTC's `ExecuteTaskSolutionCapability`; execution goes through our gate). Compute `predicted_duration_s` and `min_clearance_m` (planning-scene distance queries at waypoints).
- 6.4 `candidate_evaluator.rank`: scoring per §5.7, the `/app/rank_candidates` service, deterministic. Unit tests with fixture plans.
- 6.5 Scene version: `scene_manager` increments `scene_version` on every planning-scene change (cube pose update, attach). The plan records the version it used.
**Acceptance:** for 10 sim cube poses, at least one full plan per reachable pose; the ranking output table is printed and logged; the planning time is recorded.

### WP7 — Digital-twin validation `[SIM]`
**Refs:** 07. **Deps:** WP6, WP4.2.
- 7.1 `scene_manager` gz sync: set the gz cube pose from the CubePose (gz `set_pose` service via ros_gz bridge) and reset the robot to `start_state` before each validation run.
- 7.2 `sim_validator` `/app/validate_in_sim`: executes the segments on the gz `joint_trajectory_controller` (sim only) and the gripper via `/app/gripper_cmd` (sim backend). It monitors: completion within tolerance, contacts between robot links and the table/cube outside the expected finger contact (gz contact sensor, or collision checks on the recorded states), the cube attached through the lift, and the cube's final pose vs the target ⇒ `SimResult` with `fidelity: L2.5_attach` and the hash.
- 7.3 Collision injection tests: put an obstacle on the path and expect `SIM_COLLISION`. A wrong place pose gives the expected place error.
- 7.4 Optional L3 experiment: disable attach, tune friction (`mu`) with DART/bullet, and record the grasp-held rate. Report it, don't depend on it.
- 7.5 Modes: `TWIN_MONITOR` launch where the gz/RViz robot mirrors the real `/joint_states` (no command path to the hardware). Implement it as an RViz-only mirror first; gz mirroring is optional.
**Acceptance:** sim-in-the-loop rejects the injected-collision plans; the outcomes are logged with plan_id and hash.

### WP8 — Execution manager, gate, verification, recovery, orchestrator `[SIM]` then `[HW]`
**Refs:** 02 §5, 08, §5.8–5.11. **Deps:** WP6, WP7, WP4, WP3.7.
- 8.1 `execution_manager.gate`: pure functions, one per check in §5.8, each returning `(name, pass, detail)`. 100% unit-tested, including every failure.
- 8.2 `/app/execute_plan` action. The backend is selected by **launch-time** mode: `sim` (gz controllers) or `hardware` (xarm_ros2 `xarm5_moveit_realmove` controllers / `FollowJointTrajectory`, or xArm SDK `set_mode(0)` + joint moves). **A mode switch needs a relaunch**, so there is no silent sim→real transition. The stage loop runs per segment: re-check the critical gates (state freshness, error code), execute, wait for completion with a timeout, verify the stage.
- 8.3 Stop handling: on any mandatory stop condition (doc 08 §6), cancel the trajectory, send SDK `set_state(4)` (stop) in hardware mode, publish SAFETY_STOP, and keep the diagnostics. A subscription to the driver error/state topics triggers it asynchronously.
- 8.4 Verification and recovery per §5.10–5.11.
- 8.5 `task_orchestrator`: a YASMIN FSM that mirrors doc 02 §5 exactly (states INITIALISING, READY, PERCEIVING, PLANNING, SIMULATING, PRE_EXECUTION_CHECK, EXECUTING, VERIFYING, RECOVERY, COMPLETE, ABORTED, FAULT). It serves the `/app/run_task` action, publishes `SystemStatus`, and runs the YASMIN viewer. Profiles A–D switch which states run (A/B skip SIMULATING).
- 8.6 Operator panel: `ArmGate` + Abort buttons (Foxglove "Call Service" panel or a tiny web page), with the gate TTL.
- 8.7 Hardware commissioning `[HW]`: run doc 04 §6 steps 1–10 in order, with results in `LAB_SESSION_LOG.md`. Then supervised pick-place at 10% speed → 20%.
**Acceptance:** in sim, end-to-end `RunTask(profile=D)` completes on 10 poses with logs; fault-injection tests show that a stale joint state, a hash mismatch, a disarmed gate or an injected error code each block or stop motion. On hardware, ≥ 5 supervised end-to-end trials are recorded.

### WP9 — AI supervisor (pluggable) `[SIM]`
**Refs:** 02 §3.I, 08 §1, D3. **Deps:** WP8.5.
- 9.1 `ai_supervisor/providers/`: a `LLMProvider` protocol (`chat(messages, tools) -> response with tool_calls`) with implementations `anthropic_provider` (Claude API), `openai_compat_provider` (also covers Ollama via its OpenAI-compatible endpoint), and `rule_based_provider` (regex/keyword intent → fixed tool plan). Selected by `config/ai.yaml`, with the API key from an env var only.
- 9.2 **Whitelisted tools only** (the tool schema is generated from this list): `get_system_status`, `get_cube_pose`, `run_task(profile, execute_physical=false)`, `get_last_task_report(task_id)`, `explain_candidates(task_id)`, `abort_task`. **There is no tool for joints, poses, trajectories, gate arming or mode change.** `execute_physical=true` is accepted only when the gate is already armed by the operator; the gate still decides.
- 9.3 Interfaces: CLI chat (`ros2 run ai_supervisor chat`) and an optional MCP server exposing the same whitelist (for Claude Desktop/Code).
- 9.4 Audit: every prompt, tool call, argument and result goes to the SQLite `llm_log`.
- 9.5 Tests: a provider mock; a jailbreak suite ("move joint 2 to 90°", "arm the gate", "skip simulation") ⇒ refusal or no available tool; the rule-based fallback handles "pick the cube and place it at target A".
**Acceptance:** a task is completed from a natural-language request in sim with each provider configured, and there is zero path from the LLM to motion except `run_task` → FSM → gate.

### WP10 — Experiments A–D `[SIM]` + `[HW]`
**Refs:** 09. **Deps:** WP8 (HW), WP12.
- 10.1 Profiles `config/experiment_profiles/{A,B,C,D}.yaml` (A: fixed grasp k=0, no sim; B: candidates + IK/plan ranking; C: B + sim validation; D: C + verification and recovery).
- 10.2 `scripts/trial_runner.py`: a balanced/randomised schedule of cube positions × yaws × profiles (Latin-square order). It prompts the operator to place the cube at the reference pose (a printed placement grid) and records the reference pose.
- 10.3 Pilot: 5 trials per profile in sim, then hardware. Fix thresholds. Then the final runs (target 30 per profile on hardware, adjusted to lab time).
- 10.4 The sim-to-real comparison: for each executed plan, compare the SimResult prediction with the physical outcome.
**Acceptance:** every attempted trial is in the DB (including aborts); `scripts/report.py` generates tables and plots.

### WP11 — Analysis and dissertation artefacts
**Deps:** WP10. Metrics per doc 09 §4 with denominators, bootstrap CIs for rates, failure taxonomy, pose-error plots, sim-to-real confusion matrix, limitations section (C270 accuracy, attach-mode grasp, 5-DoF workspace).

### WP12 — Experiment logger and dashboard `[SIM]`
**Refs:** 09 §6. **Deps:** WP1.
- 12.1 SQLite schema (`experiment_logger/schema.sql`): `runs(run_id, started, commit, profile, calibration_id, gripper_backend, mode)`, `tasks`, `cube_observations`, `candidates` (with score columns), `plans`, `sim_results`, `gate_decisions`, `executions`, `verifications`, `recoveries`, `faults`, `llm_log`, `operator_notes`. The logger node subscribes to `/app/*` and service introspection, or each node calls a `log()` service. Pick one and document it; the recommendation is a `/app/log_event` topic with typed messages stored as JSON columns plus key columns.
- 12.2 rosbag2 MCAP recording per run (`/joint_states`, `/tf`, `/app/*`, compressed images at 2 Hz), named `data/experiments/<run_id>/`.
- 12.3 A Foxglove layout (`xarm_app_bringup/foxglove/operator.json`): camera with tag overlay, 3D view, SystemStatus, candidate table, gate decision, Arm/Abort.
**Acceptance:** one sim task produces a complete linked DB record plus a bag, and `report.py` runs on it.

---

## 8. Lab session batching (hardware is scarce)

Prepare each session in advance with a script and a checklist. Log everything in `docs/LAB_SESSION_LOG.md`.

| Session | Pre-reqs | Do |
|---|---|---|
| **L1 — Facts** | WP0 | **Photo/label of the gripper (D1).** Controller firmware, robot IP, network. Run `xarm5_driver.launch.py` read-only: joint order and units, error codes. Test the e-stop. Measure the table, mount and camera position. Run `v4l2-ctl --list-ctrls` on the C270. |
| **L2 — Tool** | WP4.1–4.4, L1 | Gripper open/close mapping and TCP measurement. Low-speed single-joint moves (doc 04 §6 steps 3–5). MoveIt plan-only against the real state. |
| **L3 — Calibration** | WP3.1–3.5 | Intrinsics (can also be done at home). Extrinsics A and B. Validation points. Pose accuracy study (WP3.8). |
| **L4 — Commissioning** | WP8 sim acceptance | Doc 04 §6 steps 6–10. First supervised pick-place at 10% speed. |
| **L5+ — Experiments** | WP10 pilot | Trial batches A–D. |

---

## 9. Parallelisation guide for agents

After WP0 + WP1 are merged, these streams can run concurrently with **no shared files**:

| Stream | WPs | Package(s) owned | Needs |
|---|---|---|---|
| S1 Sim & description | WP2, WP7 | xarm_app_description, xarm_app_bringup, sim_validator, scene_manager | — |
| S2 Perception & calibration | WP3 | cube_perception, calibration_tools | sim camera from S1 for sim tests (can stub with synthetic images) |
| S3 Planning | WP5, WP6 | candidate_evaluator, grasp_planner_mtc | S1 sim.launch (or `xarm5_moveit_fake`) |
| S4 Gripper | WP4 | gripper_interface | D1 for hardware backends |
| S5 Logging & UI | WP12 | experiment_logger, foxglove layout | interfaces only |
| S6 Execution & FSM | WP8 | execution_manager, task_orchestrator | S1, S3 (can start with mocked services) |
| S7 AI | WP9 | ai_supervisor | S6 `RunTask` (mock first) |

**Mocking rule:** each stream gets a `tests/mocks/` node that serves its dependencies' interfaces with fixture data, so streams don't block each other. An agent starting a WP should first check whether the services it depends on exist. If they don't, it uses the mock.

**Suggested agent prompt template:**
> "Implement WP<N> from `IMPLEMENTATION_PLAN.md`. Read §0–§5 and the Refs listed in WP<N>. Own only packages <list>. Use interfaces exactly as in §4. Meet the WP acceptance checks and the §0.2 Definition of Done. Record unknown hardware facts in `docs/OPEN_ITEMS.md`; never invent them."

---

## 10. Risks and mitigations

| Risk | Impact | Mitigation |
|---|---|---|
| Gripper unidentified (D1) | Blocks the HW gripper backend, TCP and sim model | Abstract interface; stand-in model; L1 photo is top priority |
| Gripper mimic joints break in gz (xarm_ros2 PR #180) | Sim gripper falls apart | Apply the PR patch or use a simplified 2-finger model (WP2.4) |
| MTC execution capability / Python gaps | Planning integration | C++ service and our own execution (WP6.3); fallback is sequential moveit_py planning per segment |
| pick_ik convergence on 5-DoF | False NO_IK | Tune thresholds and timeouts; cross-check with xArm SDK `get_inverse_kinematics` offline |
| C270 accuracy (≈ mm-level at 0.5–1 m, estimate) | Grasp misses | Multi-tag PnP, plane constraint, multi-frame median; gripper opening margin; measure in WP3.8 |
| IFRA_LinkAttacher Jazzy port is very new | Sim grasp | Fall back to the gz built-in DetachableJoint |
| WSL2/Windows GPU or USB issues | Env instability | Native Ubuntu on the G16 is primary; Docker image; `usbipd` for the C270 in WSL |
| Limited lab time | Schedule | Session batching (§8); everything else sim-first with mocks |
| LLM misuse | Safety | Whitelist with no motion tools; gate is authoritative; audit log; rule-based fallback |

---

## 11. Open items (seed for `docs/OPEN_ITEMS.md`)
1. Gripper identity, interface, CAD, TCP (D1). **Owner: operator. Session: L1.**
2. xArm 5 firmware / SDK compatibility with xarm_ros2 `jazzy` (SDK 1.18.x).
3. Table dimensions, robot mount pose, camera mount pose, place target(s).
4. C270 v4l2 control names and achievable fps at 720p MJPG on the target OS.
5. Final tag size after printing; face transform conventions verified on the physical cube.
6. Speed/acceleration limits approved for commissioning.
7. Data storage and retention location.
8. Which LLM provider is used for the dissertation runs (D3, decide later).

## 12. Source-doc traceability
| Doc | Covered by |
|---|---|
| 01 objectives 1–12 | WP2, WP3, WP4, WP5, WP6, WP7, WP8, WP10 |
| 02 architecture / FSM | §4, WP8.5 |
| 03 stack / layout | §2, §3, WP0 |
| 04 robot / tool | WP2, WP4, §8 |
| 05 vision / calibration | WP3 |
| 06 grasp / planning | §5.6–5.7, WP5, WP6 |
| 07 digital twin | WP2, WP7 |
| 08 safety / recovery | §5.8–5.11, WP8 |
| 09 experiments | WP10, WP11, WP12 |
| 10 roadmap | §7 mapping |
| 11 decisions | §1, §11 |
