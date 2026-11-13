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

## Multi-cube commands (SIMULATION ONLY)
- Three identified cubes (red 1, green 2, blue 3; tag IDs in `config/cube/cubes.yaml`) and three place zones. Type into the Commands box of
  `scripts\sim_app.py` and press Run / Enter. Rule-based parser (`sim/commands.py`, no LLM), tasks run one after another, each with its own kin log.
- Examples: `place the red cube in zone 1` - `put the green cube in zone 2 and stack red on green` - `move all cubes to zone 3` -
  `stack all cubes in zone 1` - `put blue on top of red` - `home`. Errors (unknown cube, cube not in scene, zone out of range, stack > 3 high) are shown, nothing runs.
- A single-tag cube pose is only used when it passes a quality gate (`cube_perception/pnp.py`: reprojection RMS, tag size, image border,
  resting-cube z / tilt); otherwise the cube is PERCEPTION_FAIL with the reason. `scripts\measure_edge_poses.py` measures the outliers gate off / on.

## Voice input (push-to-talk, offline) + free-form understanding
- Hold the **Hold to talk** button (or hold **SPACE** while the window is focused and the text box is not) -> release -> faster-whisper
  (`small.en`, CUDA float16, CPU int8 fallback) -> text appears in the command box ("Heard: ...") -> `sim/intent.py`. Max 10 s per recording, live level meter.
- **Speak freely.** Order: (1) rule parser; (2) if it cannot parse, a local LLM (Ollama `qwen2.5:3b`, `config/ai.yaml`) proposes tasks;
  (3) if Ollama is offline, fuzzy spelling correction + rules. The LLM only PROPOSES whitelisted actions; the proposal goes through the same
  validation as the parser (cubes, zones, scene, stack height <= 3) and nothing moves until you confirm.
- **Confirm.** Voice commands always show and speak (Windows TTS, checkbox **Speak replies**) `Did you mean: ...?` with **Yes, do it / No** buttons.
  Answer by voice (`yes`, `yeah do it`, `no`, `cancel`) or text; anything else is treated as a new / corrected command. Typed commands the rules fully
  understood run directly; typed text that needed the LLM / fuzzy fix also asks. A line under the voice meter shows `Brain: Ollama qwen2.5:3b ready` or `Ollama offline - rules only`.
- Setup: install Ollama, `ollama pull qwen2.5:3b` (first reply is slower while the model loads; the app warms it at start).
- First start downloads the model (~480 MB) into `models/whisper/` (git-ignored); the status shows `Voice: loading...` -> `Voice: ready (GPU)`.
  GPU needs `nvidia-cublas-cu12` + `nvidia-cudnn-cu12` (in `requirements.txt`; their DLL folders are registered by `sim/voice.py`).
- Disabled (tooltip says why) when there is no microphone, the model is still loading, or a task is running.
- Example phrases: `place the red cube in zone one` - `stack red on green` - `move all cubes to zone two` - `put cube two in zone three` - `go home`.
- Test without speaking: `XARM_APP_TEST=1 XARM_APP_TEST_VOICE=<file.wav> XARM_APP_SHOTS=<dir> .venv\Scripts\python -I scripts\sim_app.py`
  feeds the wav as if spoken, runs the command, saves dpg + 3D screenshots (no desktop capture). `tests/unit/test_voice.py` (model test skipped when not downloaded).

## PDF report
- `.venv\Scripts\python -I scripts\make_report.py --last N` builds `results/reports/<stamp>_xarm_sim_report.pdf` from the N newest kin logs
  (`--since <stamp>`, `--logs`, `--runs-csv`, `--out` also available).

## Conveyor: pick a MOVING cube (SIMULATION ONLY, all numbers PLACEHOLDER)
- **Finding it in the app:** the left column scrolls (mouse wheel) - the CONVEYOR panel is at the bottom, with a `(?) how it works` tooltip.
  Start belt -> set speed -> `Feed cube` (puts the chosen cube at the belt start) -> tick `Pick from belt to zone N` to auto-catch it.
  Or say: `put the red cube on the belt and catch it into zone 2`.
- **Belt** (`sim/conveyor.py`, config `conveyor:` in `config/workspace/workspace.yaml`): a straight lane along world x at y = -0.17 m,
  x 0.18..0.47 m, cubes flow toward the robot (-x), end stop at x = 0.18 m. It overlaps no pick / place zone (static cubes never sit on it),
  is inside reach and inside the C270 view with margin (`tests/sim/test_conveyor.py` projects every possible cube footprint with the TRUE camera: >= 30 px
  from the image border). Dark rubber belt box, side rails, rollers, orange end stop and light cleat stripes that move with the belt; the
  app also draws the yellow pick window. The belt is only built when asked for (`world.add_conveyor()`), so the calibration and the scored single-cube batch never see it.
- **Kinematic belt**: every sim step an unattached cube resting on the belt is translated by `v*dt` along the flow (yaw kept, linear velocity set to
  `v`, the physics step's own drift removed); a cube that reaches the end stop stops there. Belt speed 0..6 cm/s (default 3 cm/s, app slider /
  `set conveyor speed to 4`). Belt state, cleat phase and cube poses are mirrored into the hidden camera world, so frames show the cube at the right sim time.
- **Vision tracking** (`sim/belt_task.py`): the robot never reads the belt speed. It renders N = 5 camera frames 0.1 s apart (sim time passes between
  frames), every frame -> AprilTags -> PnP (K_est) -> centre ray intersected with the known belt-top plane (T_base_cam estimate only) and fits
  `s(t) = s0 + v (t - t_mean)` along the lane by least squares (leave-one-out rejection of frames > 2.5 mm off the line). It logs v_est +- sigma_v next to the TRUE speed (scoring only).
  |v_est| below max(3 mm/s, 2 sigma) = "stationary".
- **Intercept planning**: earliest grasp time t* = join (park -> pregrasp over the predicted cube, joint quintic) + hover 0.2 s + tracking descent;
  candidates / neighbour filter / IK / place feasibility run on the PREDICTED pose. Execution = joint move -> `cartesian-tracking` hover (belt velocity weight
  ramps 0 -> 1) -> tracking descent -> close the gripper while tracking -> kinematic attach -> lift with the belt velocity blended to zero -> normal place.
  Segment type `cartesian-tracking` (`trajectory.plan_tracking`): `p(t) = p0 + v_est*Ib(t) + dz*s(tau)*e_z`, `Ib = w0 t + (w1-w0) T Is(tau)`, `Is = 2.5 tau^4 - 3 tau^5 + tau^6`,
  q by per-step IK, qd / qdd analytic; the kin log prints the equation, p0, v_est, weights, dz and peaks vs limits.
- Outcomes: SUCCESS, MISSED (pick window passed / cube would hit the end stop before the grip closed / IK or limits), GRASP_MISS (closing-axis
  offset above the open-finger margin 7.5 mm), PERCEPTION_FAIL, PLACE_FAIL.
- **App**: Conveyor panel (Start/Stop belt, speed slider 0-6 cm/s, Feed cube + Feed, `Pick from belt to zone N` auto-pick after the feed). Commands (voice too):
  `start the conveyor` - `stop the belt` - `set conveyor speed to 4 cm per second` - `feed the red cube onto the conveyor` -
  `pick the green cube from the belt and place it in zone 2` - `catch the blue cube on the conveyor and put it in zone 3`.
- **Headless**: `.venv\Scripts\python -I scripts\sim_demo.py --conveyor --command "feed the red cube and pick the red cube from the belt and place it in zone 2" --use-calib-cache [--conveyor-speed 0.03]`;
  batch `--conveyor-runs 10 --conveyor-speed 0.02,0.04,0.06` writes `results/sim_conveyor/conveyor_summary.md` + `conveyor_runs.csv`.
  Test hook: `XARM_APP_TEST=1 XARM_APP_NO_SCREEN=1 XARM_APP_CUBES=1 XARM_APP_COMMAND="..." XARM_APP_SHOTS=<dir>` (dpg + 3D shots, incl. a mid-tracking shot).
