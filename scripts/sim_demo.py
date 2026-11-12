"""XarmAI all-simulation demo (SIM_DEMO_PLAN.md).  SIMULATION ONLY.

    .venv\\Scripts\\python -I scripts\\sim_demo.py --gui --runs 10 --seed 0

Flow: sim intrinsics calibration -> extrinsics A (tag on gripper, hand-eye) -> N x
(random cube -> tags -> cube pose in base frame -> candidates -> IK + rank -> pick -> place).
Ground truth (K_true, T_world_cam_true, true cube pose, true tag mount) is used ONLY for scoring
and for placing props in the scene; the pipeline works from K_est/dist_est and T_base_cam_est.
"""
import argparse
import csv
import math
import os
import sys
import time

import cv2
import numpy as np
import pybullet as pb

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (ROOT, *(os.path.join(ROOT, "src", k) for k in
                  ("cube_perception", "calibration_tools", "candidate_evaluator"))):
    if p not in sys.path:
        sys.path.insert(0, p)

from pupil_apriltags import Detector  # noqa: E402

from calibration_tools import calibrate_checkerboard, calibrate_eye_to_hand  # noqa: E402
from candidate_evaluator import generate, rank, format_table, evaluate, load_yaml  # noqa: E402
from candidate_evaluator.generate import pose_to_matrix  # noqa: E402
from candidate_evaluator.ik_backend import joint_margin  # noqa: E402
from cube_perception import (corners_from_pupil, estimate_cube_pose, fold_yaw, cube_yaw,  # noqa: E402
                             load_cube_model, median_pose, to_base)
from cube_perception.cube_model import tag_corners_in_tag_frame  # noqa: E402
from sim import calib_cache  # noqa: E402
from sim.c270_camera import C270SimCamera, load_cfg  # noqa: E402
from sim.ik_adapter import PyBulletIK  # noqa: E402
from sim.pyb_world import SimWorld, T_from, Tinv, rot_err_rad  # noqa: E402

BANNER = ("SIMULATION ONLY | camera = C270 model from public spec + PLACEHOLDER noise | "
          "grasp = kinematic attach (L2.5) | gripper = xArm Gripper stand-in | IK = PyBullet (MoveIt/pick_ik later)")

# ---- script-level settings (demo parameters, not hardware facts) ----
S = {
    "board_inner": (9, 6), "board_square_m": 0.025,
    "board_depth_m": (0.50, 0.62),      # board distance from camera
    "board_lateral_m": 0.04,            # board centre offset in camera x/y
    "board_tilt_deg": 35.0,             # max random tilt
    "board_roll_deg": 20.0,             # max in-plane rotation
    "calib_xy_m": ((0.30, 0.52), (-0.16, 0.16)),   # calib-pose TCP xy sampling box
    "calib_z_m": (0.08, 0.30),
    "calib_tilt_deg": (-55.0, 20.0),    # tool tilt in the radial plane (5-DOF reachable set)
    "frames_per_pose": 3,
    "handeye_outlier_trans_mm": 5.0,    # drop poses whose residual exceeds this after first solve
    "cube_margin_m": 0.03,
    "cube_min_dist_to_place_m": 0.10,
    "settle_steps": 240,
    "move_steps": 40,
    "cube_upright_cos": math.cos(math.radians(10)),
    "min_valid_frames": 1,
    # Perception/intrinsics "park" pose: the ready pose puts the gripper in front of the camera (occludes the
    # table), so the arm is swung aside (J1 +1.5 rad) while the camera looks. Candidate IK still seeds from ready.
    "park_q": [1.5, -0.5, -0.5, 0.0, 0.0],
}


# ----------------------------------------------------------------- helpers
def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def rot_axis(axis, a):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


def T_of(R, t):
    M = np.eye(4)
    M[:3, :3], M[:3, 3] = R, t
    return M


def rot_deg(Ra, Rb):
    return math.degrees(rot_err_rad(Ra, Rb))


def wrap90(a_rad):
    return fold_yaw(a_rad)


def log(*a):
    print(*a, flush=True)


WIN = "C270 sim"


class UserStop(Exception):
    """GUI window closed (PyBullet or OpenCV) -> stop cleanly."""


def is_disconnect_error(e):
    return isinstance(e, pb.error) and "not connected" in str(e).lower()


class Pipeline:
    def __init__(self, args):
        self.args = args
        self.rng = np.random.default_rng(args.seed)
        self.interactive = bool(args.gui and not getattr(args, "auto", False))
        self.app = bool(getattr(args, "app", False))           # sim_app.py drives the UI (Dear PyGui), no OpenCV UI
        self.cv_ui = bool(args.gui and not self.app)           # legacy OpenCV window + PyBullet debug panel
        self.world = SimWorld(gui=args.gui, gui_panel=args.gui, clean=self.app,
                               window=getattr(args, "window", (1200, 900)),
                               init_q=S["park_q"] if self.app else None)
        self.cam = C270SimCamera(load_cfg(self.world.cfg_paths["camera"]), np.random.default_rng(args.seed + 1))
        self.model = load_cube_model()
        self.det = Detector(families="tag36h11", nthreads=1, quad_decimate=1.0)
        self.ws = self.world.workspace
        self.cfg = {k: load_yaml(os.path.join(ROOT, "config", d, f)) for k, (d, f) in {
            "gripper": ("gripper", "gripper.yaml"), "planning": ("planning", "planning.yaml"),
            "ranking": ("planning", "ranking.yaml")}.items()}
        self.q_ready = list(self.ws["ready_joint_state_rad"])
        self.table_z = float(self.ws["table_top_z_m"])
        self.edge = self.model.edge_m
        # hidden shadow world: all IK/FK and all C270 rendering happen here, so the visible GUI world only
        # ever shows the real motion (no flashing arm poses, no stand/shadow toggling). Headless: aux IS world.
        if self.world.gui:
            self.aux = SimWorld(gui=False, cfg_paths=self.world.cfg_paths)
        else:
            self.aux = self.world
        self.ik = PyBulletIK(self.aux)
        self.K = self.dist = None           # ESTIMATED intrinsics (pipeline uses only these)
        self.T_base_cam = None              # ESTIMATED extrinsics
        self.last_frame = None
        self.n_renders = 0
        self.close_ang = None
        self.phase = "idle"
        self.last_outcome = "-"
        self.running_k = None
        self._panel_n = 0
        self._pb_txt = []           # [(item id, last string)]
        self.on_frame = None        # app hook: (img, dets, T_cam_cube) after every show()
        self.on_progress = None     # calibration progress hook: (stage, done, total)
        if self.cv_ui:
            self.world.on_step = self._on_step
        os.makedirs(os.path.join(args.out, "frames"), exist_ok=True)

    def close(self):
        if self.aux is not self.world:
            self.aux.close()
        self.world.close()

    # ---------------- kinematics panel (GUI only) ----------------
    def panel_lines(self):
        w = self.world
        q, qd = np.degrees(w.arm_q()), np.degrees(w.qd)
        T = w.tcp_T()
        R = T[:3, :3]
        roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
        pitch = math.degrees(math.atan2(-R[2, 0], math.hypot(R[2, 1], R[2, 2])))
        yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        g = math.degrees(w.p.getJointState(w.robot, w.grip_j[0])[0])
        L = ["Phase: %s" % self.phase]
        L += ["J%d %7.1f deg %7.1f d/s" % (i + 1, q[i], qd[i]) for i in range(5)]
        L += ["TCP x%6.0f y%6.0f z%6.0f mm" % tuple(1000 * T[:3, 3]),
              "TCP r%6.1f p%6.1f y%6.1f deg" % (roll, pitch, yaw),
              "Gripper %.1f deg" % g, "Last: %s" % self.last_outcome]
        return L

    def _update_pb_panel(self, lines):
        p = self.world.p
        for i, s in enumerate(lines):
            pos = [0.0, 0.55, 0.62 - 0.045 * i]
            if i < len(self._pb_txt):
                if self._pb_txt[i][1] == s:
                    continue
                p.addUserDebugText(s, pos, [1, 1, 0.2], 1.3, replaceItemUniqueId=self._pb_txt[i][0])
                self._pb_txt[i] = (self._pb_txt[i][0], s)
            else:
                self._pb_txt.append((p.addUserDebugText(s, pos, [1, 1, 0.2], 1.3), s))

    def _on_step(self):
        self._panel_n += 1
        if self._panel_n % 6:
            return
        lines = self.panel_lines()
        self._update_pb_panel(lines)
        if self._panel_n % 12 == 0:
            self.refresh_cv(lines)
            self.check_stop()

    def refresh_cv(self, lines=None):
        img = self.last_frame if self.last_frame is not None else np.zeros((480, 640, 3), np.uint8)
        img = img.copy()
        x0 = img.shape[1] - 250
        y = 40
        for s in (lines if lines is not None else self.panel_lines()):
            cv2.rectangle(img, (x0 - 4, y - 13), (img.shape[1] - 2, y + 4), (0, 0, 0), -1)
            cv2.putText(img, s, (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 255, 200), 1)
            y += 18
        cv2.imshow(WIN, img)
        cv2.waitKey(1)

    def draw_zones(self):
        """GUI: PICK (green) and PLACE (red) rectangles on the table + cross at the place target."""
        p, z = self.world.p, self.table_z + 0.003

        def rect(box, col, label):
            (x0, x1), (y0, y1) = box["x_m"], box["y_m"]
            pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
            for i in range(4):
                p.addUserDebugLine([*pts[i], z], [*pts[(i + 1) % 4], z], col, 3)
            p.addUserDebugText(label, [(x0 + x1) / 2 - 0.05, y0 - 0.025, z + 0.005], col, 1.4)
        rect(self.ws["cube_workspace_box"], [0, 0.9, 0], "PICK zone")
        rect(self.ws["place_zone"], [1, 0, 0], "PLACE zone")
        pt = self.ws["place_target"]
        for d in ((0.02, 0), (0, 0.02)):
            p.addUserDebugLine([pt["x_m"] - d[0], pt["y_m"] - d[1], z], [pt["x_m"] + d[0], pt["y_m"] + d[1], z],
                               [1, 0, 0], 3)

    # ---------------- camera / detection ----------------
    def sync_aux(self):
        """Copy arm / gripper / cube state of the visible world into the hidden aux world (no stepping)."""
        w, x = self.world, self.aux
        if x is w:
            return
        q = w.arm_q()
        g = w.p.getJointState(w.robot, w.grip_j[0])[0]
        x.set_arm(q)
        x.set_gripper(g)
        if w.cube is None:
            if x.cube is not None:
                x.remove(x.cube)
            return
        pos, orn = w.p.getBasePositionAndOrientation(w.cube)
        if x.cube is None:
            x.spawn_cube(pos[0], pos[1], 0.0)
        x.p.resetBasePositionAndOrientation(x.cube, pos, orn)
        x.p.resetBaseVelocity(x.cube, [0, 0, 0], [0, 0, 0])

    def grab(self):
        self.n_renders += 1
        self.sync_aux()
        return self.cam.render(self.aux.p, gui=False)

    def detect(self, img):
        """-> {tag_id: corners 4x2 (OpenCV TL,TR,BR,BL)}"""
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return {int(d.tag_id): corners_from_pupil(d) for d in self.det.detect(g)}

    def show(self, img, lines=(), dets=None, T_cam_cube=None):
        out = img.copy()
        for tid, c in (dets or {}).items():
            q = c.astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(out, [q], True, (0, 255, 0), 2)
            cv2.putText(out, "id%d" % tid, (int(c[0, 0]), int(c[0, 1]) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0), 2)
        if T_cam_cube is not None and self.K is not None:
            rv, _ = cv2.Rodrigues(T_cam_cube[:3, :3])
            cv2.drawFrameAxes(out, self.K, self.dist, rv, T_cam_cube[:3, 3], 0.05, 3)
        y = 40
        for s in lines:      # single-pass text on a dark box (OpenCV 5 glyph advance depends on thickness)
            (tw, th), _ = cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
            cv2.rectangle(out, (6, y - th - 4), (14 + tw, y + 5), (0, 0, 0), -1)
            cv2.putText(out, s, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            y += 24
        cv2.putText(out, "SIMULATION ONLY - C270 model, PLACEHOLDER noise", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
        self.last_frame = out
        if self.cv_ui:
            self.refresh_cv()
            self.check_stop()
        if self.on_frame is not None:
            self.on_frame(img, dets, T_cam_cube)
        return out

    def check_stop(self):
        if not self.world.is_connected():
            raise UserStop("PyBullet window closed")
        if self.cv_ui and cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
            raise UserStop("OpenCV window closed")

    def hold(self):
        """Keep windows open after the last run until q/Esc or a window is closed."""
        img = self.last_frame if self.last_frame is not None else np.zeros((480, 640, 3), np.uint8)
        img = img.copy()
        txt = "Done - press q or close window"
        cv2.rectangle(img, (6, img.shape[0] - 40), (20 + 12 * len(txt), img.shape[0] - 8), (0, 0, 0), -1)
        cv2.putText(img, txt, (10, img.shape[0] - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow(WIN, img)
        try:
            while True:
                k = cv2.waitKey(50) & 0xFF
                if k in (ord("q"), 27):
                    return
                self.check_stop()
        except UserStop:
            return
        except pb.error as e:
            if not is_disconnect_error(e):
                raise

    def calib_hash(self):
        a = self.args
        return calib_cache.config_hash(self.world.cfg_paths, a.seed, a.board_poses, a.calib_poses, S)

    def _prog(self, stage, done, total):
        if self.on_progress is not None:
            self.on_progress(stage, done, total)

    def apply_calib(self, d):
        """Install a cached calibration (K, dist, T_base_cam, closed-gripper angle)."""
        self.K, self.dist, self.T_base_cam = d["K"], d["dist"], d["T_base_cam"]
        self.close_ang = d["close_ang"]
        if not self.app:                     # app sets its own park pose; avoid a visible ready-pose flash
            self.world.set_arm(self.q_ready)

    def run_calibrations(self):
        """Intrinsics + extrinsics A. With --gui they run in a hidden headless SimWorld (same configs / camera
        model / seed) and K, dist, T_base_cam are copied here, so the GUI never shows teleporting props."""
        if not self.args.gui:
            use_cache = getattr(self.args, "use_calib_cache", False)
            if use_cache:
                d = calib_cache.load(self.calib_hash())
                if d is not None:
                    log("[calib] using cache %s (%s)" % (calib_cache.PATH, d["timestamp"]))
                    self.apply_calib(d)
                    return d["intr"], d["ext"]
            intr = self.calibrate_intrinsics()
            ext = self.calibrate_extrinsics()
            self.world.set_arm(self.q_ready)
            self.close_ang = self.closed_angle()
            if use_cache:
                calib_cache.save(intr, ext, self.close_ang, self.calib_hash())
            return intr, ext
        p = self.world.p
        txt = p.addUserDebugText("Calibrating (hidden)...", [0.15, 0.0, 0.35], [1, 0.8, 0], 3.0)
        hargs = argparse.Namespace(**vars(self.args))
        hargs.gui = False
        hp = Pipeline(hargs)                  # same seed -> same rng stream and camera noise stream as headless
        intr = hp.calibrate_intrinsics()
        ext = hp.calibrate_extrinsics()
        hp.world.set_arm(hp.q_ready)
        self.close_ang = hp.closed_angle()
        self.K, self.dist, self.T_base_cam = hp.K, hp.dist, hp.T_base_cam
        self.rng, self.cam.rng = hp.rng, hp.cam.rng       # continue the same random streams
        self.n_renders += hp.n_renders
        hp.close()
        p.removeUserDebugItem(txt)
        return intr, ext

    # ---------------- step 1: intrinsics ----------------
    def calibrate_intrinsics(self):
        n = self.args.board_poses
        w, cam = self.world, self.cam
        inner, sq = S["board_inner"], S["board_square_m"]
        w.set_arm(S["park_q"])
        body, pts = w.spawn_checkerboard(np.eye(4), inner, sq)
        T_wc = cam.T_world_cam_true      # scene placement only (props in front of the camera)
        imgs, tries = [], 0
        while len(imgs) < n and tries < 6 * n:
            tries += 1
            d = self.rng.uniform(*S["board_depth_m"])
            lat = self.rng.uniform(-S["board_lateral_m"], S["board_lateral_m"], 2)
            Rt = (rot_axis([0, 0, 1], math.radians(self.rng.uniform(-S["board_roll_deg"], S["board_roll_deg"])))
                  @ rot_axis([math.cos(a := self.rng.uniform(0, 2 * math.pi)), math.sin(a), 0],
                             math.radians(self.rng.uniform(0, S["board_tilt_deg"]))))
            T_cb = T_of(np.diag([1, -1, -1.0]) @ Rt, [lat[0], lat[1], d])   # board faces camera
            T_wb = T_wc @ T_cb
            corners_w = (T_wb[:3, :3] @ np.array([[-.15, -.11, 0], [.15, .11, 0], [-.15, .11, 0], [.15, -.11, 0]]).T
                         ).T + T_wb[:3, 3]
            if corners_w[:, 2].min() < self.table_z + 0.01:     # keep the floating board above the table
                continue
            w._set_pose(body, T_wb)
            img = self.grab()
            self.show(img, ["intrinsics: checkerboard %d/%d" % (len(imgs), n)])
            ok, _ = cv2.findChessboardCorners(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), inner)
            if ok:
                imgs.append(img)
            self._prog("intrinsics", len(imgs), n)
        w.remove(body)
        K, dist, rms = calibrate_checkerboard(imgs, inner, sq)
        self.K, self.dist = K, dist.ravel()[:5]
        Kt, dt = cam.K_true, cam.dist_true
        res = {"rms": rms, "views": len(imgs), "tries": tries, "K": K, "dist": self.dist,
               "K_true": Kt, "dist_true": dt}
        res["err_pct"] = {k: 100 * (K[i, j] - Kt[i, j]) / Kt[i, j]
                          for k, (i, j) in {"fx": (0, 0), "fy": (1, 1), "cx": (0, 2), "cy": (1, 2)}.items()}
        g = np.stack(np.meshgrid(np.linspace(0, cam.W - 1, 33), np.linspace(0, cam.H - 1, 19)), -1).reshape(-1, 1, 2)
        ue = cv2.undistortPoints(g, K, self.dist, P=Kt).reshape(-1, 2)      # est model -> true pinhole coords
        ut = cv2.undistortPoints(g, Kt, dt, P=Kt).reshape(-1, 2)
        res["undist_max_px"] = float(np.max(np.linalg.norm(ue - ut, axis=1)))
        res["undist_mean_px"] = float(np.mean(np.linalg.norm(ue - ut, axis=1)))
        log("[intrinsics] views=%d rms=%.3f px  fx=%.1f(%.2f%%) fy=%.1f(%.2f%%) cx=%.1f cy=%.1f" % (
            len(imgs), rms, K[0, 0], res["err_pct"]["fx"], K[1, 1], res["err_pct"]["fy"], K[0, 2], K[1, 2]))
        return res

    # ---------------- step 2: extrinsics A ----------------
    def tag_T_cam(self, corners, size):
        obj = tag_corners_in_tag_frame(size)
        ok, rv, tv = cv2.solvePnP(obj, corners.astype(np.float64), self.K, self.dist,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        T = np.eye(4)
        T[:3, :3], _ = cv2.Rodrigues(rv)
        T[:3, 3] = tv.ravel()
        return T

    def calibrate_extrinsics(self):
        n = self.args.calib_poses
        w = self.world
        tid, size = self.model.calib_tag_id, self.model.calib_tag_size_m
        w.spawn_calib_tag()                       # mount is chosen in the sim and unknown to the solver
        Ts_base_tcp, Ts_cam_tag = [], []
        tries = ik_fail = det_fail = 0
        (x0, x1), (y0, y1) = S["calib_xy_m"]
        w.set_gripper(0.0)
        q_prev = list(self.q_ready)
        while len(Ts_base_tcp) < n and tries < 12 * n:
            tries += 1
            pos = np.array([self.rng.uniform(x0, x1), self.rng.uniform(y0, y1), self.rng.uniform(*S["calib_z_m"])])
            phi = math.atan2(pos[1], pos[0])
            alpha = math.radians(self.rng.uniform(*S["calib_tilt_deg"]))
            r_hat = np.array([math.cos(phi), math.sin(phi), 0])
            tang = np.array([-math.sin(phi), math.cos(phi), 0])
            z = -np.array([0, 0, 1.0]) * math.cos(alpha) + r_hat * math.sin(alpha)
            R = np.stack([np.cross(tang, z), tang, z], 1) @ rz(self.rng.uniform(-math.pi, math.pi))
            ok, q, _ = self.ik.solve(T_of(R, pos), q_prev)
            if not ok:
                ik_fail += 1
                continue
            w.set_arm(q)
            imgs = [self.grab()]
            d0 = self.detect(imgs[0])
            if tid not in d0:
                det_fail += 1
                continue
            dets = [d0] + [self.detect(self.grab()) for _ in range(S["frames_per_pose"] - 1)]
            cs = [d[tid] for d in dets if tid in d]
            if len(cs) < 2:
                det_fail += 1
                continue
            T_ct = self.tag_T_cam(np.mean(cs, axis=0), size)    # average corners over frames, then IPPE
            if T_ct is None:
                continue
            Ts_base_tcp.append(w.tcp_T())      # T_base_tcp from FK (base == world frame)
            Ts_cam_tag.append(T_ct)
            q_prev = q
            self.show(imgs[0], ["extrinsics A: pose %d/%d" % (len(Ts_base_tcp), n)], dets[0])
            self._prog("extrinsics", len(Ts_base_tcp), n)
        if len(Ts_base_tcp) < 3:
            raise RuntimeError("extrinsics A: only %d usable poses" % len(Ts_base_tcp))
        sol = calibrate_eye_to_hand(Ts_base_tcp, Ts_cam_tag)
        dropped = 0
        # robust step: drop poses with a large residual (e.g. IPPE flips), re-solve
        for _ in range(2):
            res = []
            for a, b in zip(Ts_base_tcp, Ts_cam_tag):
                dlt = np.linalg.inv(a @ sol["T_tcp_tag"]) @ (sol["T_base_cam"] @ b)
                res.append(np.linalg.norm(dlt[:3, 3]) * 1000)
            keep = [i for i, r in enumerate(res) if r <= S["handeye_outlier_trans_mm"]]
            if len(keep) == len(res) or len(keep) < max(3, len(res) // 2):
                break
            dropped += len(res) - len(keep)
            Ts_base_tcp = [Ts_base_tcp[i] for i in keep]
            Ts_cam_tag = [Ts_cam_tag[i] for i in keep]
            sol = calibrate_eye_to_hand(Ts_base_tcp, Ts_cam_tag)
        self.T_base_cam = sol["T_base_cam"]
        # ---- scoring vs ground truth (SCORING ONLY) ----
        Tc_true, Tm_true = self.cam.T_world_cam_true, w.T_tcp_tag
        out = {
            "poses": len(Ts_base_tcp), "tries": tries, "ik_fail": ik_fail, "det_fail": det_fail, "dropped": dropped,
            "resid_rot_deg": sol["rot_resid_deg"], "resid_trans_mm": sol["trans_resid_mm"],
            "cam_pos_mm": float(np.linalg.norm(self.T_base_cam[:3, 3] - Tc_true[:3, 3]) * 1000),
            "cam_rot_deg": rot_deg(self.T_base_cam[:3, :3], Tc_true[:3, :3]),
            "mount_pos_mm": float(np.linalg.norm(sol["T_tcp_tag"][:3, 3] - Tm_true[:3, 3]) * 1000),
            "mount_rot_deg": rot_deg(sol["T_tcp_tag"][:3, :3], Tm_true[:3, :3]),
            "T_base_cam_est": self.T_base_cam, "T_world_cam_true": Tc_true,
        }
        w.remove(w.calib_tag_body)
        w.T_tcp_tag = None
        w.set_arm(self.q_ready)
        log("[extrinsics A] poses=%d (tries %d, dropped %d) resid %.2f mm/%.3f deg | camera err %.1f mm / %.2f deg | "
            "mount err %.1f mm / %.2f deg" % (out["poses"], tries, dropped, out["resid_trans_mm"],
                                              out["resid_rot_deg"], out["cam_pos_mm"], out["cam_rot_deg"],
                                              out["mount_pos_mm"], out["mount_rot_deg"]))
        return out

    # ---------------- step 3 helpers ----------------
    def sample_cube(self):
        box = self.ws["cube_workspace_box"]
        m = S["cube_margin_m"]
        pt = self.ws["place_target"]
        while True:
            x = self.rng.uniform(box["x_m"][0] + m, box["x_m"][1] - m)
            y = self.rng.uniform(box["y_m"][0] + m, box["y_m"][1] - m)
            if math.hypot(x - pt["x_m"], y - pt["y_m"]) >= S["cube_min_dist_to_place_m"]:
                break
        return x, y, float(self.rng.uniform(-math.pi, math.pi))

    def perceive(self):
        """3 frames -> detect -> PnP (K_est) -> median -> base frame with table constraint."""
        frames, dets_all, Ts, ids = [], [], [], set()
        reasons = []
        for _ in range(3):
            img = self.grab()
            d = self.detect(img)
            frames.append(img)
            dets_all.append(d)
            r = estimate_cube_pose([(i, c) for i, c in d.items() if i in self.model.corners_cube],
                                   self.K, self.dist, self.model)
            if r.valid:
                Ts.append(r.T_cam_cube)
                ids.update(r.tag_ids_used)
            else:
                reasons.append(r.invalid_reason)
        info = {"valid_frames": len(Ts), "tag_ids": sorted(ids), "reasons": reasons, "img": frames[0],
                "dets": {i: c for i, c in dets_all[0].items() if i in self.model.corners_cube}}
        if len(Ts) < S["min_valid_frames"]:
            return None, None, info
        T_cc = median_pose(Ts)
        Tb = to_base(self.T_base_cam, T_cc, self.table_z, self.edge)
        info["T_cam_cube"] = T_cc
        return (float(Tb[0, 3]), float(Tb[1, 3]), float(Tb[2, 3]), cube_yaw(Tb)), T_cc, info

    # ---------------- motion ----------------
    def move(self, q_to, steps=None):
        w = self.world
        w.move_joints(w.arm_q(), q_to, steps or S["move_steps"])

    def gripper_to(self, ang, steps=24):
        self.world.move_gripper(ang, steps)

    def closed_angle(self):
        """Gripper joint angle at which the finger AABB gap equals the cube edge (visual only)."""
        w, p = self.aux, self.aux.p
        lname = {p.getJointInfo(w.robot, i)[12].decode(): i for i in range(p.getNumJoints(w.robot))}
        old_q, old_g = w.arm_q(), p.getJointState(w.robot, w.grip_j[0])[0]
        w.set_arm(self.q_ready)       # tcp y == -world y here, so the world-y AABB gap is the jaw gap

        def gap(a):
            w.set_gripper(a)
            lo = p.getAABB(w.robot, lname["left_finger"])
            hi = p.getAABB(w.robot, lname["right_finger"])
            return max(hi[0][1] - lo[1][1], lo[0][1] - hi[1][1])
        a, b = 0.0, 0.85
        for _ in range(30):
            m = (a + b) / 2
            if gap(m) > self.edge:
                a = m
            else:
                b = m
        w.set_arm(old_q)
        w.set_gripper(old_g)
        return b

    # ---------------- one run ----------------
    def run_one(self, k, close_ang, pose=None, total=None, respawn=True):
        """One pick-and-place. pose=(x, y, yaw_rad) is the cube pose source (None = random sampler)."""
        self.running_k = k
        try:
            rec = self._run_one(k, close_ang, pose, total, respawn)
        finally:
            self.phase = "idle"
        self.running_k = None
        self.last_outcome = rec["outcome"]
        return rec

    def _run_one(self, k, close_ang, pose, total, respawn):
        w, a = self.world, self.args
        t0 = time.time()
        x, y, yaw = self.sample_cube() if pose is None else pose
        self.phase = "perceive"
        if a.gui:
            self.move(S["park_q"])
        else:
            w.set_arm(S["park_q"])
        w.set_gripper(0.0)
        if respawn or w.cube is None:
            w.spawn_cube(x, y, yaw)
        else:
            w.set_cube_pose(x, y, yaw)
        Tt = w.cube_pose_true()                                  # GROUND TRUTH: scoring only
        xt, yt, yawt = Tt[0, 3], Tt[1, 3], cube_yaw(Tt)
        rec = {"run": k, "true_x": xt, "true_y": yt, "true_yaw_deg": math.degrees(yawt),
               "est_x": "", "est_y": "", "est_yaw_deg": "", "pos_err_mm": "", "yaw_err_deg": "", "tags": "",
               "feasible": "", "total": "", "chosen_yaw_deg": "", "chosen_rank": "", "grasp_offset_mm": "",
               "place_err_mm": "", "place_yaw_err_deg": "", "upright": "", "outcome": ""}
        est, T_cc, info = self.perceive()
        rec["tags"] = ",".join(map(str, info["tag_ids"]))
        head = ["run %02d%s  true x=%.3f y=%.3f yaw=%.1f" % (k, "" if self.interactive else "/%d" % (total or a.runs), xt, yt, math.degrees(yawt))]
        if est is None:
            rec["outcome"] = "PERCEPTION_FAIL"
            self.show(info["img"], head + ["PERCEPTION_FAIL " + ",".join(info["reasons"])], info["dets"])
            self._save_frame(k)
            return rec
        ex, ey, ez, eyaw = est
        rec.update(est_x=ex, est_y=ey, est_yaw_deg=math.degrees(eyaw),
                   pos_err_mm=1000 * math.hypot(ex - xt, ey - yt),
                   yaw_err_deg=math.degrees(wrap90(eyaw - yawt)))
        cand_lines = head + ["est x=%.3f y=%.3f yaw=%.1f  err %.1f mm / %.2f deg  tags %s" % (
            ex, ey, math.degrees(eyaw), rec["pos_err_mm"], rec["yaw_err_deg"], rec["tags"])]
        self.show(info["img"], cand_lines, info["dets"], T_cc)

        # ---- candidates ----
        cands = generate(est, self.cfg["gripper"], self.cfg["planning"], self.ws, "run%02d" % k, self.edge)
        evaluate(cands, self.ik, self.q_ready)
        res = rank(cands, self.cfg["ranking"])
        log(format_table(cands))
        nfeas = len(res.ranked)
        rec["feasible"] = "%d/%d" % (nfeas, len(cands))
        if nfeas == 0 or res.selected_id is None:
            rec["outcome"] = "NO_FEASIBLE"
            self.show(info["img"], cand_lines + ["NO_FEASIBLE"], info["dets"], T_cc)
            self._save_frame(k)
            return rec

        # ---- place feasibility (walk down the ranking; first candidate whose place poses are reachable) ----
        pt = self.ws["place_target"]
        T_est = T_of(rz(eyaw), [ex, ey, ez])
        chosen = None
        order = [c for c in res.ranked if c.candidate_id == res.selected_id] + \
                [c for c in res.ranked if c.candidate_id != res.selected_id]
        for rank_i, c in enumerate(order):
            plan = self.plan_place(c, T_est, pt, eyaw)
            if plan is not None:
                chosen, chosen_plan, chosen_rank = c, plan, rank_i
                break
        if chosen is None:
            rec["outcome"] = "NO_FEASIBLE"
            rec["feasible"] += " (place IK fail)"
            self.show(info["img"], cand_lines + ["NO_FEASIBLE (place unreachable)"], info["dets"], T_cc)
            self._save_frame(k)
            return rec
        c = chosen
        rec["chosen_id"] = c.candidate_id
        rec.update(chosen_yaw_deg=math.degrees(c.grasp_yaw_rad), chosen_rank=chosen_rank + 1, total=c.score.total)
        log("  chosen %s (rank %d of %d feasible), total %.3f" % (c.candidate_id, chosen_rank + 1, nfeas, c.score.total))
        self.show(info["img"], cand_lines + ["chosen %s yaw %.0f deg (%d feasible)" % (
            c.candidate_id, math.degrees(c.grasp_yaw_rad), nfeas)], info["dets"], T_cc)

        # ---- execute ----
        self.phase = "pregrasp"
        self.move(self.q_ready)
        cube0 = w.cube_pose_true()[:3, 3].copy()
        self.move(c.q_pregrasp)
        self.phase = "grasp"
        self.move(c.q_grasp)
        moved_pre = float(np.linalg.norm(w.cube_pose_true()[:2, 3] - cube0[:2])) * 1000
        self.gripper_to(close_ang)
        T_tcp_cube = Tinv(w.tcp_T()) @ w.cube_pose_true()       # TRUE offset (scoring)
        rec["grasp_offset_mm"] = float(np.linalg.norm(T_tcp_cube[:3, 3]) * 1000)
        w.attach_cube()
        self.phase = "lift"
        self.move(c.q_lift)
        self.phase = "place"
        self.move(chosen_plan["q_pre"])
        self.move(chosen_plan["q_place"])
        w.detach_cube()
        self.gripper_to(0.0)
        self.phase = "retreat"
        self.move(chosen_plan["q_pre"])
        self.move(self.q_ready)
        self.move(S["park_q"], 60)
        for _ in range(S["settle_steps"]):
            w.p.stepSimulation()
        Tf = w.cube_pose_true()
        perr = 1000 * math.hypot(Tf[0, 3] - pt["x_m"], Tf[1, 3] - pt["y_m"])
        up = float(Tf[2, 2])
        rec.update(place_err_mm=perr, place_yaw_err_deg=math.degrees(wrap90(cube_yaw(Tf) - pt["yaw_rad"])),
                   upright=bool(up >= S["cube_upright_cos"]))
        tol = 1000 * float(self.ws["place_tol_m"])
        rec["outcome"] = "SUCCESS" if (perr < tol and rec["upright"]) else "PLACE_FAIL"
        if moved_pre > 2.0:
            log("  note: cube moved %.1f mm before grasp (finger contact)" % moved_pre)
        rec["moved_before_grasp_mm"] = moved_pre
        log("  place err %.1f mm (tol %.0f), upright=%s -> %s  [%.1fs]" % (
            perr, tol, rec["upright"], rec["outcome"], time.time() - t0))
        self.show(info["img"], cand_lines + ["chosen %s | grasp offset %.1f mm | place err %.1f mm | %s" % (
            c.candidate_id, rec["grasp_offset_mm"], perr, rec["outcome"])], info["dets"], T_cc)
        self._save_frame(k)
        if a.gui:       # show the placed cube in the live window too
            self.show(self.grab(), [rec["outcome"] + " - cube placed, place err %.1f mm" % perr])
            if self.cv_ui:
                cv2.waitKey(300)
        return rec

    def interactive_loop(self, recs):
        """GUI sliders (cube x, y, yaw) + 'Pick & Place' button. Idle: cube follows sliders. Press: full run."""
        w, p = self.world, self.world.p
        box, m = self.ws["cube_workspace_box"], S["cube_margin_m"]
        bx, by = (box["x_m"][0] + m, box["x_m"][1] - m), (box["y_m"][0] + m, box["y_m"][1] - m)
        sx = p.addUserDebugParameter("cube x (m)", bx[0], bx[1], 0.5 * (bx[0] + bx[1]))
        sy = p.addUserDebugParameter("cube y (m)", by[0], by[1], 0.5 * (by[0] + by[1]))
        sw = p.addUserDebugParameter("cube yaw (deg)", -180, 180, 0)
        btn = p.addUserDebugParameter("Pick & Place", 1, 0, 0)

        def read():
            return tuple(float(p.readUserDebugParameter(i)) for i in (sx, sy, sw))
        vals = read()
        w.spawn_cube(vals[0], vals[1], math.radians(vals[2]))
        self.move(S["park_q"])
        last_btn, applied, k, t_img = p.readUserDebugParameter(btn), vals, 0, 0.0
        auto = int(os.environ.get("XARM_SIM_TEST_AUTOPRESS", "0") or 0)      # test hook: N automatic presses, then exit
        log("Interactive: set cube with the sliders, press 'Pick & Place'. q / Esc / close window = quit.")
        while True:
            self.check_stop()
            cur, b = read(), p.readUserDebugParameter(btn)
            if b != last_btn or auto > 0:
                last_btn = b
                if auto > 0:
                    auto -= 1
                k += 1
                log("\n--- run %d (sliders x=%.3f y=%.3f yaw=%.0f deg) ---" % (k, cur[0], cur[1], cur[2]))
                recs.append(self.run_one(k, self.close_ang, (cur[0], cur[1], math.radians(cur[2])), respawn=False))
                applied = read()
                self.show(self.grab(), ["%s - press 'Pick & Place' again, or move sliders" % recs[-1]["outcome"]])
                t_img = time.time()
                continue
            if cur != applied:
                w.set_cube_pose(cur[0], cur[1], math.radians(cur[2]), settle=0)
                applied = cur
            if "XARM_SIM_TEST_AUTOPRESS" in os.environ and auto == 0 and k > 0:
                return
            if time.time() - t_img > 0.3:
                self.show(self.grab(), ["IDLE - set cube with sliders, press 'Pick & Place'"])
                t_img = time.time()
            if cv2.waitKey(15) & 0xFF in (ord("q"), 27):
                raise UserStop("quit key")

    def plan_place(self, c, T_est, pt, eyaw):
        """Commanded place poses from the ESTIMATED grasp geometry; IK-check pregrasp + place."""
        T_grasp = pose_to_matrix(c.grasp_pose)
        T_cube_tcp = np.linalg.inv(T_est) @ T_grasp
        n = round((eyaw - pt["yaw_rad"]) / (math.pi / 2))        # cube is 90-deg symmetric: least tool rotation
        T_cube_t = T_of(rz(pt["yaw_rad"] + n * math.pi / 2),
                        [pt["x_m"], pt["y_m"], self.table_z + self.edge / 2 + 0.002])
        T_place = T_cube_t @ T_cube_tcp
        T_pre = T_place.copy()
        T_pre[2, 3] += c.approach_distance_m
        lo, hi = self.ik.joint_limits()
        qs, cur = [], list(c.q_lift)
        for T in (T_pre, T_place):
            ok, q, _ = self.ik.solve(T, cur)
            if not ok or joint_margin(q, lo, hi) < 0:
                return None
            qs.append(q)
            cur = q
        return {"q_pre": qs[0], "q_place": qs[1]}

    def _save_frame(self, k):
        if self.last_frame is not None:
            cv2.imwrite(os.path.join(self.args.out, "frames", "run_%02d.png" % k), self.last_frame)


# ----------------------------------------------------------------- reporting
def fmt(v, f="%.1f"):
    return "" if v == "" or v is None else (f % v if isinstance(v, (int, float, np.floating)) else str(v))


def write_outputs(args, intr, ext, recs, runtime, partial=False, aborted=None):
    os.makedirs(args.out, exist_ok=True)
    cols = ["run", "true_x", "true_y", "true_yaw_deg", "est_x", "est_y", "est_yaw_deg", "pos_err_mm",
            "yaw_err_deg", "tags", "feasible", "chosen_yaw_deg", "chosen_rank", "grasp_offset_mm",
            "place_err_mm", "place_yaw_err_deg", "upright", "outcome"]
    sfx = "_partial" if partial else ""
    with open(os.path.join(args.out, "runs%s.csv" % sfx), "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore", restval="")
        wr.writeheader()
        wr.writerows(recs + ([aborted] if aborted else []))

    def nums(key):
        return [r[key] for r in recs if r[key] != ""]
    n_ok = sum(r["outcome"] == "SUCCESS" for r in recs)
    pe, ye, go, pl = nums("pos_err_mm"), [abs(v) for v in nums("yaw_err_deg")], nums("grasp_offset_mm"), nums("place_err_mm")
    L = []
    L.append("# XarmAI simulation demo - results%s\n" % (" (PARTIAL)" if partial else ""))
    if partial:
        L.append("**Stopped by user after %d runs** (run %s: ABORTED_BY_USER, not counted as failure; "
                 "success rate denominator = completed runs).\n" % (
                     len(recs), aborted["run"] if aborted else "-"))
    L.append("**%s**\n" % BANNER)
    L.append("Command: `python -I scripts/sim_demo.py %s --seed %d` (calib poses %d, board poses %d). "
             "Runtime %.0f s. Ground truth is used for scoring only.\n" % (
                 "--gui (interactive sliders)" if (args.gui and not getattr(args, "auto", False))
                 else "--runs %d" % args.runs, args.seed, args.calib_poses, args.board_poses, runtime))
    L.append("Honesty labels: SIMULATION ONLY; C270 noise/blur/distortion = PLACEHOLDER (tuned by eye); "
             "grasp = kinematic attach, no contact physics; gripper = xArm Gripper stand-in (real gripper unidentified); "
             "IK = PyBullet, no collision checking in planning; no MoveIt, no real robot, no real camera. "
             "`place_tol_m` = %.0f mm is a PLACEHOLDER. Calibration B was cut (plan cut order).\n" % (
                 1000 * float(load_yaml(os.path.join(ROOT, 'config/workspace/workspace.yaml'))['place_tol_m'])))
    L.append("## 1. Intrinsics (checkerboard, %d views, RMS reprojection %.3f px)\n" % (intr["views"], intr["rms"]))
    L.append("| param | estimated | true | error |\n|---|---|---|---|")
    K, Kt = intr["K"], intr["K_true"]
    for nm, (i, j) in {"fx": (0, 0), "fy": (1, 1), "cx": (0, 2), "cy": (1, 2)}.items():
        L.append("| %s | %.2f | %.2f | %+.2f %% (%+.2f px) |" % (nm, K[i, j], Kt[i, j], intr["err_pct"][nm],
                                                                 K[i, j] - Kt[i, j]))
    for i, nm in enumerate(["k1", "k2", "p1", "p2", "k3"]):
        L.append("| %s | %.4f | %.4f | %+.4f |" % (nm, intr["dist"][i], intr["dist_true"][i],
                                                   intr["dist"][i] - intr["dist_true"][i]))
    L.append("\nEffective accuracy: undistorting a 33x19 pixel grid with the estimated vs the true model differs by "
             "%.2f px mean / %.2f px max (k2/k3 are strongly correlated, so judge the model, not single terms). "
             "The pipeline uses the estimated K/dist only.\n" % (intr["undist_mean_px"], intr["undist_max_px"]))
    L.append("## 2. Extrinsics A (tag 10 on gripper, eye-to-hand, no ground truth in the solve)\n")
    L.append("| item | value |\n|---|---|")
    L.append("| poses used / tries / outliers dropped | %d / %d / %d (IK fail %d, tag not detected %d) |" % (
        ext["poses"], ext["tries"], ext["dropped"], ext["ik_fail"], ext["det_fail"]))
    L.append("| residual (mean) | %.2f mm / %.3f deg |" % (ext["resid_trans_mm"], ext["resid_rot_deg"]))
    L.append("| **T_base_cam error vs truth** | **%.1f mm / %.2f deg** |" % (ext["cam_pos_mm"], ext["cam_rot_deg"]))
    L.append("| estimated tag mount (T_tcp_tag) error vs truth | %.1f mm / %.2f deg |\n" % (
        ext["mount_pos_mm"], ext["mount_rot_deg"]))
    L.append("## 3. Runs\n")
    L.append("| run | true x,y (m) / yaw (deg) | est x,y (m) / yaw (deg) | pos err (mm) | yaw err (deg, mod 90) | "
             "tags | feasible | chosen yaw (deg) | grasp offset (mm) | place err (mm) | outcome |")
    L.append("|---|---|---|---|---|---|---|---|---|---|---|")
    for r in recs:
        est = "" if r["est_x"] == "" else "%.3f, %.3f / %.1f" % (r["est_x"], r["est_y"], r["est_yaw_deg"])
        L.append("| %d | %.3f, %.3f / %.1f | %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
            r["run"], r["true_x"], r["true_y"], r["true_yaw_deg"], est, fmt(r["pos_err_mm"]),
            fmt(r["yaw_err_deg"], "%.2f"), r["tags"], r["feasible"], fmt(r["chosen_yaw_deg"], "%.0f"),
            fmt(r["grasp_offset_mm"]), fmt(r["place_err_mm"]), r["outcome"]))
    L.append("\n## 4. Totals\n")
    L.append("| metric | value |\n|---|---|")
    L.append("| **success** | **%d / %d** |" % (n_ok, len(recs)))
    if pe:
        L.append("| cube position error (mm) mean / max | %.2f / %.2f |" % (np.mean(pe), np.max(pe)))
        L.append("| cube yaw error (deg, mod 90) mean / max | %.2f / %.2f |" % (np.mean(ye), np.max(ye)))
    if go:
        L.append("| grasp offset TCP->cube centre (mm) mean / max | %.2f / %.2f |" % (np.mean(go), np.max(go)))
    if pl:
        L.append("| place error (mm) mean / max | %.2f / %.2f |" % (np.mean(pl), np.max(pl)))
    L.append("| outcomes | %s |" % ", ".join("%s=%d" % (o, sum(r["outcome"] == o for r in recs))
                                              for o in sorted({r["outcome"] for r in recs})))
    L.append("| intrinsics RMS | %.3f px |" % intr["rms"])
    L.append("| T_base_cam error | %.1f mm / %.2f deg |" % (ext["cam_pos_mm"], ext["cam_rot_deg"]))
    with open(os.path.join(args.out, "summary%s.md" % sfx), "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")
    return n_ok


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gui", action="store_true", help="PyBullet GUI + OpenCV window 'C270 sim'")
    ap.add_argument("--runs", type=int, default=10)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--calib-poses", type=int, default=20, help="extrinsics A poses")
    ap.add_argument("--board-poses", type=int, default=20, help="checkerboard views")
    ap.add_argument("--auto", action="store_true",
                    help="with --gui: run N random runs (old behaviour) instead of the interactive sliders mode")
    ap.add_argument("--use-calib-cache", action="store_true",
                    help="headless: load/save results/sim_calib/sim_calib.json instead of always calibrating "
                         "(default off: the scored demo calibrates for real)")
    ap.add_argument("--out", default=None, help="default results/sim_demo (results/sim_interactive for interactive)")
    ap.add_argument("--hold", action=argparse.BooleanOptionalAction, default=None,
                    help="keep windows open after the last run (default: on with --gui)")
    args = ap.parse_args(argv)
    interactive = args.gui and not args.auto
    if args.out is None:
        args.out = os.path.join(ROOT, "results", "sim_interactive" if interactive else "sim_demo")
    if args.hold is None:
        args.hold = args.gui and not interactive
    t0 = time.time()
    log("=" * 100 + "\n" + BANNER + "\n" + "=" * 100)
    P = Pipeline(args)
    intr = ext = None
    recs, aborted = [], None
    stopped = False
    inject = int(os.environ.get("XARM_SIM_TEST_DISCONNECT_AFTER_RUN", "0") or 0)   # test hook
    try:
        try:
            intr, ext = P.run_calibrations()
            close_ang = P.close_ang
            if args.gui:
                P.draw_zones()
            if interactive:
                P.interactive_loop(recs)
            else:
                for k in range(1, args.runs + 1):
                    aborted = {"run": k, "outcome": "ABORTED_BY_USER"}
                    P.check_stop()
                    log("\n--- run %d/%d ---" % (k, args.runs))
                    recs.append(P.run_one(k, close_ang))
                    aborted = None
                    if k == inject:
                        P.world.p.disconnect()      # simulate the user closing the PyBullet window
        except (UserStop, pb.error) as e:
            if isinstance(e, pb.error) and not is_disconnect_error(e):
                raise
            stopped = True
            if interactive and P.running_k:
                aborted = {"run": P.running_k, "outcome": "ABORTED_BY_USER"}
        if interactive and not stopped:
            stopped = True      # test-hook exit: still write partial-style outputs
        if stopped:
            if intr is not None and ext is not None and (recs or not interactive):
                write_outputs(args, intr, ext, recs, time.time() - t0, partial=True, aborted=aborted)
            log("\nStopped - partial results saved (%d completed runs): %s" % (
                len(recs), os.path.join(args.out, "summary_partial.md")))
        else:
            n_ok = write_outputs(args, intr, ext, recs, time.time() - t0)
            log("\nSUCCESS %d/%d | summary: %s | runtime %.0f s (%d renders)" % (
                n_ok, len(recs), os.path.join(args.out, "summary.md"), time.time() - t0, P.n_renders))
            if args.hold and args.gui:
                P.hold()
    finally:
        P.close()
        if args.gui:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
