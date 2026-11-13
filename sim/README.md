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

## Running the sim
- **GUI (use this for demos): `.venv\Scripts\python -I scripts\sim_app.py`** - Dear PyGui control window (right) + clean PyBullet 3D
  view (left, view only: no debug sliders/text, green PICK zone, orange PLACE zone + target disc as flat visual-only boxes).
  Sections: Cube pose (x/y/yaw sliders within the PICK zone minus 30 mm margin, Random pose, Centre), Actions (Pick & Place,
  Home, Recalibrate, speed 0.25x-4x (playback only)), Last run + history, Calibration (source, RMS, T_base_cam error, progress bar),
  C270 camera view with tag outlines / cube axes, Kinematics (J1..J5 angle, speed, limits, TCP xyz/rpy, gripper, phase,
  10 s joint plot), Log. Closing either window exits cleanly.
- **Calibration cache**: `results/sim_calib/sim_calib.json` (K, dist, T_base_cam, scores, timestamp, hash of camera + cube +
  workspace yaml and calibration settings; SIMULATION ONLY). Valid cache -> app is Ready in ~1 s. Missing/stale -> calibrates in
  a background thread (headless world, progress bar, ~2.5 min), Pick disabled until done. "Recalibrate" reruns it.
- Test hooks: `XARM_APP_TEST=1` (sets sliders, presses Pick & Place, saves screenshots to `XARM_APP_SHOTS`, exits 0 on SUCCESS);
  `XARM_APP_TEST_CLOSE=dpg|pb` simulates closing a window mid-run.
- **Scored headless batch: `.venv\Scripts\python -I scripts\sim_demo.py --runs 10 --seed 0`** (random cubes, always calibrates
  for real, writes `results/sim_demo/`). Optional `--use-calib-cache` loads/saves the cache instead (default off).
- Legacy `sim_demo.py --gui` (PyBullet sliders + OpenCV window) and `--gui --auto` still work but are superseded by `sim_app.py`.
- Zones: `config/workspace/workspace.yaml` (`cube_workspace_box` = pick, `place_zone`, `place_target`; PLACEHOLDER values).
- Legacy test hook: `XARM_SIM_TEST_AUTOPRESS=N` (sim_demo.py --gui) auto-presses the button N times, then exits.

## Motion timing, straight moves, kin logs (SIMULATION ONLY, limits are PLACEHOLDER)
- Every segment is a quintic `q(t) = q0 + D*(10 tau^3 - 15 tau^4 + 6 tau^5)` sampled at dt = 1/240 s (`sim/trajectory.py`);
  sim time = steps x dt. Limits: URDF joint velocity (3.14 rad/s) x `sim_velocity_scale` 0.5 = 1.57 rad/s, `sim_joint_acc_limit_rad_s2` 2.5,
  segment floor 0.3 s, gripper 0.6 s, Cartesian peak speed 0.1 m/s (all in `config/planning/planning.yaml`, all PLACEHOLDER).
  The GUI Speed slider only changes playback (sleep), never logged times.
- Transit moves (park, ready, pregrasp, lift -> place-pre) are joint quintics. Pregrasp -> grasp, grasp -> lift, place-pre <-> place are straight
  Cartesian lines (IK at every step on the hidden aux world, verified < 0.5 mm; falls back to the joint quintic and logs it).
- URDF: `link_eef` and `link_tcp` carry a negligible inertial (1e-6 kg) so PyBullet does not add a default 1 kg there.
- Camera mount: `config/camera/c270_sim.yaml` is the 891e40f mount (eye [0.65, 0, 0.60]); the later higher mount saw only 1 tag on mid-zone cubes.
- **Per-run validation logs**: every run (app and headless, also failed ones) writes `results/kin_logs/<YYYYMMDD_HHMMSS>_runNN_x<mm>_y<mm>_yaw<deg>_<OUTCOME>.txt`
  and `.csv` (`sim/kin_log.py`; folder is git-ignored; `--no-kin-log` / `--kin-dir` on `sim_demo.py`). TXT: header, robot model, IK errors,
  FK check against an independent numpy FK, trajectory equations and coefficients, peak speed/acceleration vs limits, model dynamics
  (inverse dynamics from URDF inertials + point-mass payload, forward-dynamics round trip; model values, not measured), summary.
  CSV: one row per sim step (t, q, qd, qdd, tau, TCP pose, gripper, attached).
