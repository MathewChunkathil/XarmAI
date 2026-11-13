# Single-tag outlier gate (SIMULATION ONLY)

150 poses per cube (3 cubes) spread over and beyond the pick zone (view edges included), simulated C270 (noise PLACEHOLDER), true intrinsics / extrinsics. Outlier = pos error > 5 mm or yaw error > 5 deg vs true pose (scoring only).

| gate | valid poses | 1-tag valid | outliers (all) | outliers (1-tag) | pos err mm mean / max (valid) |
|---|---|---|---|---|---|
| off | 331 | 60 | 4 | 3 | 1.14 / 18.75 |
| on | 328 | 57 | 1 | 0 | 1.05 / 5.41 |

rejected by the gate (reason: count over all frames of the 'on' pass): {'HIGH_REPROJ': 2, 'SINGLE_TAG_NOT_RESTING': 1, 'SINGLE_TAG_RMS': 2}

