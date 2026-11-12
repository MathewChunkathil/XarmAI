# Demo script - 13 Nov 2026 (about 5 minutes)

**Say this first (honesty banner):** SIMULATION ONLY. The camera is a C270 model built from public specs plus PLACEHOLDER noise.
The grasp is a kinematic attach (no contact physics). The gripper is the xArm Gripper as a stand-in. IK is PyBullet, not MoveIt.
No real robot and no real camera are used in this demo.

Command: `.venv\Scripts\python -I scripts\sim_demo.py --gui --runs 10 --seed 0` (headless: leave out `--gui`, about 2.5 min).
All numbers below come from `results/sim_demo/summary.md` (seed 0). Re-run to refresh them.

## 1. Problem (30 s)
A UFACTORY xArm 5 must pick a 50 mm cube from a random place and put it at a target, using a cheap USB webcam (Logitech C270)
and AprilTags on the cube. The hard parts are: knowing where the camera is relative to the robot, getting the cube pose
accurately from tags, choosing a grasp the 5-DOF arm can reach, and doing all this without trusting anything blindly.

## 2. Architecture (45 s) - from `02_SYSTEM_ARCHITECTURE.md`
Layers: camera -> vision/AprilTag pose -> camera-to-base transform -> grasp candidate generator -> kinematics/reachability
-> motion planning (MoveIt 2) -> digital twin (Gazebo) -> ranking -> deterministic execution gate -> execution manager -> robot,
with verification and logging after the move. The AI supervisor sits above and cannot bypass the gate.
What this sim exercises: the **pure-Python modules that will be reused unchanged in the ROS 2 nodes**:
`cube_perception` (multi-tag PnP), `calibration_tools` (checkerboard + hand-eye), `candidate_evaluator` (generate + rank, behind an `IKBackend`
interface). Only the simulator pieces (`sim/`) and the script itself are throw-away.

## 3. What the sim shows, step by step (2 min)
1. **Window opens**: xArm 5 on a table, a C270 on a stand, a second window "C270 sim" with what the camera sees.
2. **Intrinsics**: a checkerboard is shown at 20 poses; `cv2.calibrateCamera` gives K. The pipeline uses this K, not the true one.
3. **Extrinsics (method A)**: a tag (ID 10) is mounted on the gripper at a mount the solver does not know. The arm visits about 20
   poses; for each, the tag pose in the camera and the arm's forward kinematics go into an eye-to-hand solver (A X = Y B). It returns camera-to-base and the tag mount.
4. **Ten pick-and-place runs**. Each run: cube spawns at a random x, y and yaw -> 3 camera frames -> tags (IDs 0-4) detected -> multi-tag PnP
   -> median over frames -> cube pose in the robot frame, constrained to the table -> 4 grasp candidates (90 degree yaw steps)
   -> IK check -> ranking table printed -> best candidate executes: pregrasp, grasp (gripper closes), lift, move, place, release, retreat.
5. **Result files**: annotated camera frame per run (tag outlines, IDs, estimated cube axes) and a results table.

## 4. Real numbers (45 s) - seed 0, from `summary.md`
| quantity | result |
|---|---|
| Pick-and-place success (place error < 10 mm and cube upright) | **10 / 10** |
| Intrinsics: RMS reprojection | 0.19 px; fx, fy within 0.06 %, cx 1.2 px, cy 3.4 px off |
| Intrinsics: model difference vs true distortion | 4.4 px mean, 19 px max (image corners; the cube stays near the centre) |
| Camera-to-base calibration error (vs truth, scoring only) | 0.8 mm / 0.21 deg (19 of 20 poses used, 1 outlier dropped) |
| Cube position error (xy) | mean 0.72 mm, max 0.97 mm |
| Cube yaw error (mod 90 deg) | mean 0.08 deg, max 0.13 deg |
| Grasp offset TCP to cube centre | mean 0.80 mm, max 0.96 mm |
| Place error | mean 0.82 mm, max 1.05 mm |

## 5. What it proves and what it does NOT (60 s)
**Proves:** the software chain is consistent end to end: frame conventions (cube model, tag corners, hand-eye, base frame), the multi-tag PnP,
the calibration that does not use ground truth, the candidate generation + IK + ranking logic, and the yaw handling of a 90-degree-symmetric cube.
Errors are measured against ground truth that the pipeline never reads. Reproducible with `--seed`.

**Does NOT prove:**
- Real accuracy. The C270 noise, blur, distortion, JPEG quality and lighting are **PLACEHOLDER**, tuned by eye. The sub-millimetre numbers are a best case: a noise-free pinhole world with a perfect robot model. Real numbers will likely be worse.
- Grasping. The grasp is a **kinematic attach**: the cube is rigidly tied to the TCP. Friction, slip, finger collision and force are not tested.
- The real gripper. It is unidentified (photo pending); the xArm Gripper is a stand-in, and its TCP offset is not measured.
- Motion planning. IK is **PyBullet** with joint-space interpolation and no collision checking along the path. MoveIt, Gazebo and the execution gate are not exercised.
- Safety, recovery and timing. Nothing here touches the real robot.
- The arm must be parked aside while the camera looks (at the ready pose it would block the view). That is a layout finding for the real cell.
- Place tolerance (10 mm) and the cube workspace are PLACEHOLDERs until measured in the lab.
- Calibration method B was cut to save time.

## 6. Next steps (30 s)
1. **ROS 2 port** (`IMPLEMENTATION_PLAN.md` work packages): WP2 description + Gazebo + MoveIt, WP3 vision/calibration nodes, WP4 gripper interface, WP5/WP6 candidate generation and MTC planner with ranking, WP7 digital-twin validation, WP8 execution manager, gate, verification and orchestrator.
2. **Lab sessions**: L1 facts (gripper photo/label, controller, workspace and camera position measurements), L2 tool (gripper mapping, TCP, low-speed moves), L3 calibration (real intrinsics, extrinsics A and B, accuracy study).
3. **Identify the gripper** (decision D1) and replace the stand-in.
4. Replace PLACEHOLDER camera values with real C270 measurements, and repeat these 10 runs with them to see how the numbers change.
