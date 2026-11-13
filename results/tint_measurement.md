# Tint measurement (SIMULATION ONLY)

20 random cube poses in the pick zone per cell, simulated C270 (noise/blur/JPEG = PLACEHOLDER), pose by PnP with the TRUE intrinsics (isolates the texture effect), error vs true pose (scoring only). Strength 0 = plain white quiet zone.

| cube | strength | tags seen / pose (mean, min) | cube valid | pos err mm mean / max | yaw err deg mean / max | reproj rms px |
|---|---|---|---|---|---|---|
| red | 0.00 | 2.00, 0 | 19 / 20 | 1.00 / 1.51 | 0.04 / 0.15 | 0.172 |
| red | 0.20 | 1.95, 0 | 19 / 20 | 0.96 / 1.52 | 0.04 / 0.14 | 0.158 |
| red | 0.35 | 1.95, 0 | 19 / 20 | 0.94 / 1.59 | 0.04 / 0.11 | 0.157 |
| red | 0.50 | 2.05, 0 | 19 / 20 | 0.86 / 1.36 | 0.04 / 0.12 | 0.168 |
| red | 0.70 | 2.10, 0 | 19 / 20 | 0.75 / 1.12 | 0.07 / 0.56 | 0.278 |
| green | 0.00 | 2.00, 0 | 19 / 20 | 1.03 / 1.56 | 0.04 / 0.14 | 0.170 |
| green | 0.20 | 2.05, 0 | 19 / 20 | 0.93 / 1.43 | 0.04 / 0.13 | 0.162 |
| green | 0.35 | 2.15, 0 | 19 / 20 | 0.85 / 1.41 | 0.04 / 0.13 | 0.181 |
| green | 0.50 | 2.15, 0 | 19 / 20 | 0.82 / 1.19 | 0.05 / 0.16 | 0.178 |
| green | 0.70 | 2.20, 0 | 18 / 20 | 0.73 / 0.98 | 0.05 / 0.17 | 0.187 |
| blue | 0.00 | 2.05, 0 | 19 / 20 | 0.98 / 1.44 | 0.04 / 0.16 | 0.179 |
| blue | 0.20 | 1.95, 1 | 20 / 20 | 4.11 / 46.65 | 1.90 / 30.12 | 0.240 |
| blue | 0.35 | 1.95, 0 | 19 / 20 | 0.96 / 1.47 | 0.04 / 0.13 | 0.158 |
| blue | 0.50 | 2.10, 0 | 19 / 20 | 0.88 / 1.33 | 0.03 / 0.12 | 0.174 |
| blue | 0.70 | 2.20, 0 | 19 / 20 | 0.78 / 1.22 | 0.05 / 0.16 | 0.238 |

## Conclusion (SIMULATION ONLY, PLACEHOLDER camera noise)

- Tint strength 0.35 (config/cube/cubes.yaml `tint.margin_strength`) is kept: detection rate (tags seen per pose, valid cube poses 19/20 for every
  strength - the same one pose lies at the edge of the camera view for all variants) and pose error (about 0.9-1.0 mm mean, 1.4-1.6 mm max,
  yaw 0.04 deg mean) are the same as with a plain white quiet zone. Reprojection RMS stays at about 0.16-0.18 px.
- Strength 0.7 starts to cost: red reprojection RMS rises 0.17 -> 0.28 px and blue to 0.24 px, so it is not used.
- One outlier cell (blue, strength 0.20: one pose with a single marginal tag -> single-tag ambiguity, 46 mm / 30 deg) is a property of
  `min_tags: 1` in perception at the very edge of the view, not of the tint (the plain cube loses that tag entirely in the same pose).
  Not fixed here; a `min_tags: 2` rule would remove it at the cost of rejecting more edge poses.
- Cube bottom face and a thin edge band are full colour (visible in the 3D view); the tag cells themselves are untouched.
