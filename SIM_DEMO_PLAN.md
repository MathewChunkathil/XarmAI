# SIM DEMO PLAN — everything in simulation (v2, 2026-11-12)

**Replaces** the camera/hardware parts of `IMPLEMENTATION_PLAN.md` §6 for the 13 Nov review.
**Rule:** no real camera, no real robot. Every sim module is written so it ports to ROS 2 later without being rewritten.

---

## 1. Goal

One command opens a sim window. Inside it:
1. The xArm 5 sits on a table, with a fake C270 webcam on a stand.
2. The sim calibrates the camera to the robot (tag on the gripper, Kabsch).
3. 10 times: the cube spawns at a random pose → the camera finds its tags → the cube pose goes into the robot frame → 4–8 grasp candidates → IK check + score → pick → place.
4. A results table and a camera image with tag boxes are saved.

```
.venv\Scripts\python -I scripts\sim_demo.py --gui --runs 10 --seed 0
```

## 2. What exists now (built 12 Nov) and the gaps

| Exists in `scripts/demo_sim_vision.py` | Gap to fix |
|---|---|
| PyBullet world, IK, kinematic attach | Uses the **xArm 6** from pybullet_data, not the **xArm 5** |
| Rendered camera, pupil_apriltags | Ideal 1280×960 pinhole. The C270 is 1280×720 and has distortion, noise and blur |
| Calibration A and B, error vs truth | The mount alignment uses the ground-truth camera pose (a cheat, though it is labelled) |
| One cube pose, one run | Needs 10 random runs and a results table |
| Cube textured with tag ID 0 on all faces | Must use `config/cube/cube_model.yaml`: IDs 0–4, bottom blank, multi-tag PnP |
| One big script | Split into port-ready modules (§4) |

## 3. Fake C270 camera (`config/camera/c270_sim.yaml`)

Public Logitech C270 specs are used; anything not published is marked PLACEHOLDER.

| Item | Value | Source |
|---|---|---|
| Resolution | 1280×720 | Logitech spec (720p) |
| Diagonal FOV | 55° | Logitech spec. HFOV ≈ 48.4°, VFOV ≈ 28.0° are derived from it |
| Focus | fixed | Logitech spec |
| Lens distortion k1, k2 | PLACEHOLDER (small barrel) | Replaced by the real intrinsics calibration later |
| Pixel noise σ | PLACEHOLDER (≈3 gray levels, signal-dependent) | Tuned to look like a C270 |
| Blur | Gaussian σ ≈ 0.8 px (soft fixed-focus lens) | PLACEHOLDER |
| Compression | MJPEG quality ≈ 80 | Simulates the USB MJPG stream |
| Exposure / light | ±15 % brightness change per frame, light direction varies per run | PLACEHOLDER |
| Mount | Stand at about (0.65, 0, 0.60) m, looking at the table | PLACEHOLDER; becomes `extrinsics_sim.yaml` |

The pipeline first runs a **sim intrinsics calibration** (a checkerboard at 20 poses → `cv2.calibrateCamera`). It uses those results, **not** the true intrinsics, exactly as the real pipeline will.

## 4. Code layout (port-ready)

Pure-Python logic goes in `src/` packages named as in `IMPLEMENTATION_PLAN.md` §3. Nothing in these packages imports pybullet, so the same code runs in the ROS nodes later.

| Module | Job | Later ROS node |
|---|---|---|
| `src/cube_perception/cube_perception/pnp.py` | Multi-tag PnP from cube_model.yaml, yaw fold, reprojection checks | `cube_pose_node` |
| `src/calibration_tools/calibration_tools/kabsch.py`, `intrinsics.py` | Kabsch/Umeyama, hand-eye cross-check, checkerboard calibration | calibration CLIs |
| `src/candidate_evaluator/candidate_evaluator/generate.py`, `rank.py` | Candidates per §5.6, score per §5.7 | `/app/generate_candidates`, `/app/rank_candidates` |
| `sim/pyb_world.py` | xArm 5 + table + cube + camera stand in PyBullet | replaced by Gazebo |
| `sim/c270_camera.py` | Render + distortion + noise + blur + JPEG | replaced by `v4l2_camera` |
| `scripts/sim_demo.py` | Runs the whole flow, writes results | replaced by `task_orchestrator` |

Config is read from `config/` (cube, camera, gripper, workspace, ranking). There are no magic numbers in the code.

## 5. Steps, models, checks

| # | Step | Model | Done when |
|---|---|---|---|
| S0 | git init, `.gitignore`, create the public GitHub repo `xarm-ai` | Haiku | Repo exists, first commit pushed |
| S1 | Get the xArm 5 + gripper URDF: clone xarm_ros2 `jazzy` and run `xacro` (pip) to produce `xarm5_with_gripper.urdf` with mesh paths fixed | Haiku → Sonnet if xacro fails | The URDF loads in PyBullet with 5 arm joints |
| S2 | `sim/pyb_world.py`: xArm 5, table, cube from cube_model.yaml (5 tag textures, right faces/orientation) | Sonnet | A screenshot shows the right IDs on the right faces |
| S3 | `sim/c270_camera.py` + `c270_sim.yaml` + sim intrinsics calibration | Sonnet | Calibration RMS is reported; tags are still detected through the noise |
| S4 | `cube_perception/pnp.py` + unit tests (synthetic 1-, 2- and 3-tag cases) | Sonnet | Tests pass; sim pose error vs truth is printed |
| S5 | Calibration A as a clean module (tag ID 10 mount from cube_model.yaml, no ground-truth cheat), plus B as a comparison | Sonnet | Error table for A vs B |
| S6 | `candidate_evaluator` generate + rank, IK via PyBullet behind a small `IKBackend` interface (MoveIt later) | Sonnet | The candidate table is printed for each run |
| S7 | `scripts/sim_demo.py`: GUI window + live camera window (cv2) + 10 runs + `results/sim_demo/summary.md` | Sonnet | Runs end to end and the table is saved |
| S8 | `docs/DEMO_SCRIPT.md` (what is proven and what is not) + push | Sonnet / Haiku | File written, pushed |
| — | After every step: `graphify update .` | Haiku | Graph is current |
| — | Hard bug or design doubt only | Opus | — |

**Cut order if time runs short:** calibration B → the sim intrinsics step (use the known K, labelled) → the runs drop from 10 to 5. Always keep S1, S2, S4, S6 and S7.

## 6. Results table (`results/sim_demo/summary.md`)

Per run: cube true pose, estimated pose, position error (mm), yaw error (deg), tags used, feasible/rejected candidates, chosen yaw, place error (mm), and the outcome.
Totals: success rate n/10, mean/max pose error, the calibration A and B errors, and the intrinsics RMS.

## 7. Honesty labels (printed on screen and in the summary)

SIMULATION ONLY · camera = C270 model from public spec + PLACEHOLDER noise · grasp = kinematic attach (L2.5) · gripper = xArm Gripper stand-in · IK = PyBullet (MoveIt/pick_ik later).

## 8. Open doubts (need a user answer before S1)

1. Is the xArm 5 URDF source OK: xarm_ros2 `jazzy` + pip `xacro` on Windows? (Fallback: hand-write the URDF from the UFACTORY DH table.)
2. The C270 values marked PLACEHOLDER: OK to tune them by eye?
3. Should the cube show 4 side tags with "up" toward the top (as in cube_model.yaml)?
