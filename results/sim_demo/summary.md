# XarmAI simulation demo - results

**SIMULATION ONLY | camera = C270 model from public spec + PLACEHOLDER noise | grasp = kinematic attach (L2.5) | gripper = xArm Gripper stand-in | IK = PyBullet (MoveIt/pick_ik later)**

Command: `python -I scripts/sim_demo.py --runs 10 --seed 0` (calib poses 20, board poses 20). Runtime 184 s. Ground truth is used for scoring only.

Honesty labels: SIMULATION ONLY; C270 noise/blur/distortion = PLACEHOLDER (tuned by eye); grasp = kinematic attach, no contact physics; gripper = xArm Gripper stand-in (real gripper unidentified); IK = PyBullet, no collision checking in planning; no MoveIt, no real robot, no real camera. `place_tol_m` = 10 mm is a PLACEHOLDER. Calibration B was cut (plan cut order).

## 1. Intrinsics (checkerboard, 20 views, RMS reprojection 0.186 px)

| param | estimated | true | error |
|---|---|---|---|
| fx | 1411.28 | 1410.58 | +0.05 % (+0.69 px) |
| fy | 1411.40 | 1410.58 | +0.06 % (+0.82 px) |
| cx | 638.82 | 640.00 | -0.18 % (-1.18 px) |
| cy | 356.64 | 360.00 | -0.93 % (-3.36 px) |
| k1 | -0.0385 | -0.0500 | +0.0115 |
| k2 | -0.2769 | 0.0100 | -0.2869 |
| p1 | -0.0004 | 0.0000 | -0.0004 |
| p2 | -0.0002 | 0.0000 | -0.0002 |
| k3 | 2.0431 | 0.0000 | +2.0431 |

Effective accuracy: undistorting a 33x19 pixel grid with the estimated vs the true model differs by 4.38 px mean / 19.30 px max (k2/k3 are strongly correlated, so judge the model, not single terms). The pipeline uses the estimated K/dist only.

## 2. Extrinsics A (tag 10 on gripper, eye-to-hand, no ground truth in the solve)

| item | value |
|---|---|
| poses used / tries / outliers dropped | 19 / 177 / 1 (IK fail 22, tag not detected 135) |
| residual (mean) | 0.26 mm / 0.128 deg |
| **T_base_cam error vs truth** | **0.8 mm / 0.21 deg** |
| estimated tag mount (T_tcp_tag) error vs truth | 0.2 mm / 0.12 deg |

## 3. Runs

| run | true x,y (m) / yaw (deg) | est x,y (m) / yaw (deg) | pos err (mm) | yaw err (deg, mod 90) | tags | feasible | chosen yaw (deg) | grasp offset (mm) | place err (mm) | outcome |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 0.426, -0.065 / -4.9 | 0.426, -0.066 / -4.8 | 0.7 | 0.12 | 0,3 | 4/4 | -5 | 0.6 | 0.7 | SUCCESS |
| 2 | 0.445, 0.014 / -24.3 | 0.445, 0.014 / -24.3 | 0.7 | 0.05 | 0,2 | 4/4 | -24 | 0.8 | 0.8 | SUCCESS |
| 3 | 0.457, 0.122 / -37.4 | 0.456, 0.122 / -37.3 | 0.8 | 0.06 | 0,1 | 4/4 | 53 | 0.9 | 0.9 | SUCCESS |
| 4 | 0.443, 0.040 / -44.6 | 0.442, 0.039 / -44.6 | 0.7 | -0.02 | 0,3 | 4/4 | 45 | 0.9 | 0.8 | SUCCESS |
| 5 | 0.442, 0.126 / -41.2 | 0.441, 0.126 / -41.1 | 0.8 | 0.05 | 0,3 | 4/4 | 49 | 0.9 | 0.9 | SUCCESS |
| 6 | 0.445, 0.055 / -2.3 | 0.444, 0.054 / -2.2 | 0.7 | 0.07 | 0,4 | 4/4 | -2 | 0.8 | 0.9 | SUCCESS |
| 7 | 0.331, -0.018 / 35.4 | 0.331, -0.019 / 35.4 | 0.5 | -0.04 | 0,2,4 | 4/4 | 35 | 0.5 | 0.5 | SUCCESS |
| 8 | 0.394, -0.035 / 41.7 | 0.394, -0.035 / 41.7 | 0.4 | -0.04 | 0,1,3 | 4/4 | -48 | 0.3 | 0.4 | SUCCESS |
| 9 | 0.443, 0.042 / 31.3 | 0.443, 0.042 / 31.3 | 0.5 | 0.05 | 0,1,3 | 4/4 | 31 | 0.7 | 0.6 | SUCCESS |
| 10 | 0.345, 0.128 / -16.5 | 0.344, 0.128 / -16.3 | 1.0 | 0.14 | 0,1 | 4/4 | -16 | 1.0 | 1.2 | SUCCESS |

## 4. Totals

| metric | value |
|---|---|
| **success** | **10 / 10** |
| cube position error (mm) mean / max | 0.68 / 1.00 |
| cube yaw error (deg, mod 90) mean / max | 0.06 / 0.14 |
| grasp offset TCP->cube centre (mm) mean / max | 0.73 / 1.02 |
| place error (mm) mean / max | 0.76 / 1.16 |
| outcomes | SUCCESS=10 |
| intrinsics RMS | 0.186 px |
| T_base_cam error | 0.8 mm / 0.21 deg |
