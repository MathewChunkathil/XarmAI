# sim/ - PyBullet world + fake C270 (SIMULATION ONLY)

Honesty labels: SIMULATION ONLY | camera = C270 model from public spec + PLACEHOLDER noise/distortion |
grasp = kinematic attach (L2.5, `attach_cube`) | gripper = xArm Gripper stand-in | IK = PyBullet.

## Frames
- World = robot base frame, origin at link_base, z up. Table top z = `config/workspace/workspace.yaml: table_top_z_m`
  (0.0 => the base sits on the table top). Floor is 0.75 m below.
- Cube frame/tag frames: exactly as in `config/cube/cube_model.yaml` (tag z out of face, x right, y up seen from outside;
  side tags' up = cube top). Textures are built from `R_cube_tag`; verified against OpenCV (`cv2.aruco` + IPPE) within ~0.5 deg.
- Camera frame = OpenCV (x right, y down, z forward). `T_world_cam_true` maps cam -> world.
- Calib tag (ID 10) frame = ArUco convention; mount `T_tcp_tag` is set in `SimWorld.spawn_calib_tag`
  (default: beside the gripper, normal = -z_tcp). The solver must NOT read it.
- Checkerboard: 9x6 inner corners, 25 mm, 1-square white margin; board frame z out, x right, y up, origin at centre.

## Ground truth - SCORING ONLY (pipeline must never read)
`C270SimCamera.K_true / dist_true / T_world_cam_true`, `SimWorld.cube_pose_true()`, `SimWorld.T_tcp_tag`.

## Notes
- Camera image = true-pinhole render -> lens distortion (inverse map of the OpenCV model) -> blur -> brightness jitter
  -> shot+read noise -> JPEG. All non-spec numbers in `config/camera/c270_sim.yaml` are `PLACEHOLDER: tuned by eye`.
- `pupil_apriltags` pose axes are NOT the ArUco convention here (observed: R_aruco = R_pupil @ diag(-1,1,-1) in this
  setup); use cv2.aruco/solvePnP or convert explicitly.
- Textured meshes/atlases are cached in `sim/_cache/` (generated, safe to delete).
- `python -I sim/preview.py` renders one frame to `results/sim_demo/preview_cam.png`.
