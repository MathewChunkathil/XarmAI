# cube_perception
Inputs: AprilTag detections `[(id, corners 4x2 TL,TR,BR,BL)]`, intrinsics K/dist, `config/cube/cube_model.yaml`.
Outputs: `CubePoseResult` (T_cam_cube, tag_ids_used, rms_px, valid, invalid_reason in
NO_TAGS|UNKNOWN_ID|INCONSISTENT|HIGH_REPROJ); `to_base` (optional table-plane constraint),
`fold_yaw`, `median_pose`, `corners_from_pupil` (pupil_apriltags order [TR,TL,BL,BR] -> OpenCV).
Thresholds: `DEFAULT_CFG` in `pnp.py`, overridable via cfg dict. No simulator dependencies.
