# XarmAI simulation demo - results

**SIMULATION ONLY | camera = C270 model from public spec + PLACEHOLDER noise | grasp = kinematic attach (L2.5) | gripper = xArm Gripper stand-in | IK = PyBullet (MoveIt/pick_ik later)**

Command: `python -I scripts/sim_demo.py --runs 10 --seed 0` (calib poses 20, board poses 20). Runtime 128 s. Ground truth is used for scoring only.

Honesty labels: SIMULATION ONLY; C270 noise/blur/distortion = PLACEHOLDER (tuned by eye); grasp = kinematic attach, no contact physics; gripper = xArm Gripper stand-in (real gripper unidentified); IK = PyBullet, no collision checking in planning; no MoveIt, no real robot, no real camera. `place_tol_m` = 10 mm is a PLACEHOLDER. Calibration B was cut (plan cut order).

## 1. Intrinsics (checkerboard, 20 views, RMS reprojection 0.192 px)

| param | estimated | true | error |
|---|---|---|---|
| fx | 1410.17 | 1410.58 | -0.03 % (-0.41 px) |
| fy | 1410.14 | 1410.58 | -0.03 % (-0.44 px) |
| cx | 639.94 | 640.00 | -0.01 % (-0.06 px) |
| cy | 360.89 | 360.00 | +0.25 % (+0.89 px) |
| k1 | -0.0526 | -0.0500 | -0.0026 |
| k2 | 0.0151 | 0.0100 | +0.0051 |
| p1 | 0.0001 | 0.0000 | +0.0001 |
| p2 | 0.0002 | 0.0000 | +0.0002 |
| k3 | 0.0920 | 0.0000 | +0.0920 |

Effective accuracy: undistorting a 33x19 pixel grid with the estimated vs the true model differs by 0.92 px mean / 1.86 px max (k2/k3 are strongly correlated, so judge the model, not single terms). The pipeline uses the estimated K/dist only.

## 2. Extrinsics A (tag 10 on gripper, eye-to-hand, no ground truth in the solve)

| item | value |
|---|---|
| poses used / tries / outliers dropped | 19 / 177 / 1 (IK fail 22, tag not detected 135) |
| residual (mean) | 0.15 mm / 0.048 deg |
| **T_base_cam error vs truth** | **0.4 mm / 0.08 deg** |
| estimated tag mount (T_tcp_tag) error vs truth | 0.0 mm / 0.05 deg |

## 3. Runs

| run | true x,y (m) / yaw (deg) | est x,y (m) / yaw (deg) | pos err (mm) | yaw err (deg, mod 90) | tags | feasible | chosen yaw (deg) | grasp offset (mm) | place err (mm) | outcome |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 0.426, -0.163 / -4.9 | 0.426, -0.163 / -5.0 | 0.2 | -0.09 | 0,2,3 | 4/4 | -5 | 0.5 | 0.1 | SUCCESS |
| 2 | 0.445, -0.051 / -24.3 | 0.445, -0.051 / -24.2 | 0.1 | 0.14 | 0,2,4 | 4/4 | -24 | 0.5 | 0.1 | SUCCESS |
| 3 | 0.457, 0.102 / -37.4 | 0.456, 0.102 / -37.4 | 0.3 | -0.03 | 0,1 | 4/4 | 53 | 0.5 | 0.4 | SUCCESS |
| 4 | 0.443, -0.015 / -44.6 | 0.443, -0.015 / -44.6 | 0.4 | -0.06 | 0,2,3 | 4/4 | -45 | 0.7 | 0.3 | SUCCESS |
| 5 | 0.442, 0.108 / -41.2 | 0.442, 0.108 / -41.3 | 0.3 | -0.15 | 0,3 | 4/4 | 49 | 0.6 | 0.2 | SUCCESS |
| 6 | 0.445, 0.006 / -2.3 | 0.444, 0.007 / -2.4 | 0.3 | -0.05 | 0,4 | 4/4 | -2 | 0.6 | 0.4 | SUCCESS |
| 7 | 0.331, -0.097 / 35.4 | 0.330, -0.097 / 35.3 | 0.9 | -0.05 | 0,4 | 4/4 | -55 | 0.9 | 0.8 | SUCCESS |
| 8 | 0.394, -0.120 / 41.7 | 0.394, -0.120 / 41.5 | 0.2 | -0.17 | 0,3 | 4/4 | -48 | 0.4 | 0.2 | SUCCESS |
| 9 | 0.443, -0.011 / 31.3 | 0.443, -0.011 / 31.4 | 0.1 | 0.13 | 0,3 | 4/4 | 31 | 0.5 | 0.1 | SUCCESS |
| 10 | 0.345, 0.110 / -16.5 | 0.344, 0.111 / -16.4 | 0.5 | 0.02 | 0,1 | 4/4 | -16 | 0.7 | 0.7 | SUCCESS |

## 4. Totals

| metric | value |
|---|---|
| **success** | **10 / 10** |
| cube position error (mm) mean / max | 0.34 / 0.86 |
| cube yaw error (deg, mod 90) mean / max | 0.09 / 0.17 |
| grasp offset TCP->cube centre (mm) mean / max | 0.58 / 0.87 |
| place error (mm) mean / max | 0.32 / 0.83 |
| outcomes | SUCCESS=10 |
| intrinsics RMS | 0.192 px |
| T_base_cam error | 0.4 mm / 0.08 deg |
