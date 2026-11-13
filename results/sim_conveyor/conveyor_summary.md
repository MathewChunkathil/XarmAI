# XarmAI simulation - conveyor batch (moving-cube pick)

**SIMULATION ONLY | camera = C270 model from public spec + PLACEHOLDER noise | grasp = kinematic attach (L2.5) | gripper = xArm Gripper stand-in | IK = PyBullet (MoveIt/pick_ik later)**

Honesty: SIMULATION ONLY; belt = kinematic translation of the cube, geometry / speeds / timing = PLACEHOLDER; the robot estimates the cube velocity from 5 camera frames (true belt speed only scores the estimate); grasp = kinematic attach; GRASP_MISS = closing-axis offset above the open-finger margin (fingers would miss). seed 0, runtime 165 s.

| speed cm/s | runs | SUCCESS | speed est err mean/max abs mm/s | sigma_v mean mm/s | tool-vs-cube at contact mean/max mm | grasp offset mean/max mm | place err mean/max mm | outcomes |
|---|---|---|---|---|---|---|---|---|
| 2.0 | 10 | 10/10 | 0.04 / 0.14 | 0.06 | 0.50 / 0.69 | 0.51 / 0.77 | 0.51 / 0.74 | SUCCESS 10 |
| 4.0 | 10 | 10/10 | 0.11 / 0.22 | 0.11 | 0.40 / 0.47 | 0.42 / 0.56 | 0.42 / 0.55 | SUCCESS 10 |
| 6.0 | 10 | 10/10 | 0.28 / 0.77 | 0.23 | 0.66 / 1.45 | 0.79 / 1.88 | 0.78 / 1.87 | SUCCESS 10 |

Place error is over SUCCESS runs only. Per-run table: conveyor_runs.csv.
