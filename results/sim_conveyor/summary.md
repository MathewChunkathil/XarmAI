# XarmAI simulation - multi-cube command run

**SIMULATION ONLY | camera = C270 model from public spec + PLACEHOLDER noise | grasp = kinematic attach (L2.5) | gripper = xArm Gripper stand-in | IK = PyBullet (MoveIt/pick_ik later)**

Command: `feed the red cube and pick the red cube from the belt and place it in zone 2`  ->  parsed: 1) feed red (cube 1) onto the conveyor; 2) pick red (cube 1) from the conveyor, place in zone 2  (seed 0, rule-based parser, no LLM). Runtime 5 s.

Honesty labels: SIMULATION ONLY; C270 noise = PLACEHOLDER; grasp = kinematic attach; gripper = xArm Gripper stand-in; cube colours / tint, zones 2-3, slot spacing, min cube spacing, finger footprint margin, stack release gap = PLACEHOLDER; poses of cubes outside the camera view come from the last COMMANDED pose (stated per task).

| # | task | pick pose from | place / stack err mm | stack dz mm | neighbours considered / rejected | outcome |
|---|---|---|---|---|---|---|
| 1 | feed red (cube 1) onto the conveyor | - | - | - | 0/0 | SUCCESS |
| 2 | pick red (cube 1) from the conveyor, place in zone 2 | perception | - | - | 0/0 | MISSED |

Target pose sources:

- task 1: -
- task 2: zone 2 slot (0.350, 0.315) from workspace.yaml place_zones
