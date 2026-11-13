"""PyBullet world: xArm 5 + gripper, table, cube (tags from cube_model.yaml), camera stand.
SIMULATION ONLY. See sim/README.md for frames and honesty labels.

World frame == robot base frame (link_base at origin). Table top z = workspace.yaml
table_top_z_m (default 0.0 => base sits on the table top).
"""
import math
import contextlib
import os
import time
import warnings

import cv2
import numpy as np
import pybullet as pb
import pybullet_data
import yaml
from pybullet_utils import bullet_client

from sim import trajectory as traj

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CACHE = os.path.join(HERE, "_cache")
URDF = os.path.join(HERE, "urdf", "xarm5_gripper.urdf")
ARUCO = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

DEFAULT_CFG = {
    "cube": os.path.join(ROOT, "config", "cube", "cube_model.yaml"),
    "workspace": os.path.join(ROOT, "config", "workspace", "workspace.yaml"),
    "camera": os.path.join(ROOT, "config", "camera", "c270_sim.yaml"),
    "planning": os.path.join(ROOT, "config", "planning", "planning.yaml"),
    "cubes": os.path.join(ROOT, "config", "cube", "cubes.yaml"),
}


# ---------------- math helpers ----------------
def T_from(pos, quat):
    M = np.eye(4)
    M[:3, :3] = np.array(pb.getMatrixFromQuaternion(quat)).reshape(3, 3)
    M[:3, 3] = pos
    return M


def quat_from_R(R):
    t = np.trace(R)
    if t > 0:
        s = 2 * math.sqrt(t + 1)
        return [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4]
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * math.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
    q = [0.0] * 4
    q[i] = s / 4
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


def Tinv(M):
    X = np.eye(4)
    X[:3, :3] = M[:3, :3].T
    X[:3, 3] = -M[:3, :3].T @ M[:3, 3]
    return X


def rot_err_rad(Ra, Rb):
    return math.acos(float(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1, 1)))


def _yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


# ---------------- texture / mesh generation ----------------
def _write_obj(path, quads):
    """quads: list of (4 corners CCW seen from outside, 4 uv, normal)."""
    L = []
    for corners, uvs, n in quads:
        for c in corners:
            L.append("v %.6f %.6f %.6f" % tuple(np.asarray(c, float)))
        for uv in uvs:
            L.append("vt %.6f %.6f" % tuple(uv))
        L.append("vn %.6f %.6f %.6f" % tuple(np.asarray(n, float)))
    for i in range(len(quads)):
        b = 4 * i + 1
        L.append("f " + " ".join("%d/%d/%d" % (b + k, b + k, i + 1) for k in range(4)))
    with open(path, "w") as f:
        f.write("\n".join(L) + "\n")


def _tag_tile(tag_id, tile_px, tag_frac):
    """White tile with the black tag (8 cells incl. black ring) centred. tag_frac = tag_size/tile_size."""
    tile = np.full((tile_px, tile_px), 255, np.uint8)
    n = int(round(tile_px * tag_frac / 8)) * 8
    m = cv2.aruco.generateImageMarker(ARUCO, tag_id, n)
    o = (tile_px - n) // 2
    tile[o:o + n, o:o + n] = m
    return tile


def _tint_tile(gray_tile, tag_px, rgb, strength, edge_px):
    """Colour a tag tile: white quiet zone -> light pastel of `rgb` (strength 0..1), outer edge band -> full colour.
    The tag itself (black ring + code cells, the square tag_px x tag_px in the centre) is left untouched."""
    n = gray_tile.shape[0]
    bgr = np.array(rgb[::-1], np.float32)
    out = np.repeat(gray_tile[:, :, None], 3, 2).astype(np.float32)
    pastel = 255.0 * (1 - strength) + bgr * strength
    o = (n - tag_px) // 2
    quiet = np.ones((n, n), bool)
    quiet[o:o + tag_px, o:o + tag_px] = False
    out[quiet] = pastel
    if edge_px > 0:
        band = np.zeros((n, n), bool)
        band[:edge_px], band[-edge_px:], band[:, :edge_px], band[:, -edge_px:] = True, True, True, True
        out[band] = bgr
    return np.clip(out, 0, 255).astype(np.uint8)


class SimWorld:
    def __init__(self, gui=False, cfg_paths=None, gui_panel=False, clean=False, window=(1200, 900), init_q=None):
        """clean=True (sim_app.py): calm 3D view only - no debug panes/sliders, shadows on, nicer colours."""
        self.gui = gui
        self.clean = bool(clean and gui)
        self.speed = 1.0             # GUI pacing factor (1 = real time at 240 Hz)
        self._t_next = 0.0
        self.on_step = None          # optional per-step callback (GUI kinematics panel)
        self.qd = np.zeros(5)        # commanded joint velocity [rad/s] (analytic)
        self.sim_t = 0.0             # simulated time [s] = played steps * dt (independent of GUI speed / wall clock)
        self.on_segment = None       # optional hook(Segment) called before a segment is played (kin log)
        self.dt_step = 1 / 240
        self.cfg_paths = dict(DEFAULT_CFG)
        if cfg_paths:
            self.cfg_paths.update(cfg_paths)
        os.makedirs(CACHE, exist_ok=True)
        self.cube_cfg = _yaml(self.cfg_paths["cube"])
        self.edge = float(self.cube_cfg["edge_m"])
        self.cubes_cfg = _yaml(self.cfg_paths["cubes"]) if os.path.exists(self.cfg_paths["cubes"]) else {"cubes": []}
        self.cube_by_num = {int(c["number"]): c for c in self.cubes_cfg["cubes"]}
        if self.clean:
            self.p = bullet_client.BulletClient(pb.GUI, options="--width=%d --height=%d" % tuple(window))
        else:
            self.p = bullet_client.BulletClient(pb.GUI if gui else pb.DIRECT)
        p = self.p
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1 / 240)
        if self.clean:
            for k in (p.COV_ENABLE_GUI, p.COV_ENABLE_RGB_BUFFER_PREVIEW, p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
                      p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, p.COV_ENABLE_MOUSE_PICKING):
                p.configureDebugVisualizer(k, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
            p.configureDebugVisualizer(rgbBackground=[0.13, 0.15, 0.19], lightPosition=[2.5, -2.0, 4.0])
            p.resetDebugVisualizerCamera(1.0, 8, -42, [0.36, 0.0, 0.03])      # wide enough for all 3 place zones
        elif gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1 if gui_panel else 0)
            p.resetDebugVisualizerCamera(1.1, 140, -35, [0.3, 0, 0.1])

        # table: top at table_top_z; robot base at world origin sits on it
        ws = self.cfg_paths["workspace"]
        if os.path.exists(ws):
            w = _yaml(ws)
            self.table_top_z = float(w["table_top_z_m"])
            tb = w.get("table_box", {})
            size = tb.get("size_xyz_m", [0.8, 0.8, 0.02])
            cen = tb.get("center_xyz_m", [0.35, 0, -0.01])
            self.workspace = w
        else:
            warnings.warn("workspace.yaml missing: table_top_z=0.0 PLACEHOLDER")
            self.table_top_z, size, cen, self.workspace = 0.0, [0.8, 0.8, 0.02], [0.35, 0, -0.01], {}
        floor_z = self.table_top_z - 0.75
        if self.clean:      # GL renderer paints a checker texture on plane visuals: use a big flat box instead
            fvs = p.createVisualShape(pb.GEOM_BOX, halfExtents=[20, 20, 0.01], rgbaColor=[0.25, 0.28, 0.34, 1])
            self.plane = p.createMultiBody(0, p.createCollisionShape(pb.GEOM_PLANE), fvs, [0, 0, floor_z - 0.01])
        else:
            self.plane = p.createMultiBody(
                0, p.createCollisionShape(pb.GEOM_PLANE),
                p.createVisualShape(pb.GEOM_PLANE, rgbaColor=[0.55, 0.55, 0.55, 1]), [0, 0, floor_z])
        half = [size[0] / 2, size[1] / 2, (self.table_top_z - floor_z) / 2]
        tpos = [cen[0], cen[1], self.table_top_z - half[2]]
        self.table = p.createMultiBody(
            0, p.createCollisionShape(pb.GEOM_BOX, halfExtents=half),
            p.createVisualShape(pb.GEOM_BOX, halfExtents=half, rgbaColor=[0.78, 0.68, 0.54, 1] if self.clean
                                else [0.45, 0.33, 0.22, 1]), tpos)
        p.changeDynamics(self.table, -1, lateralFriction=0.8)

        # robot (indices looked up by name)
        self.robot = p.loadURDF(URDF, [0, 0, self.table_top_z], useFixedBase=True)
        n = p.getNumJoints(self.robot)
        names = {p.getJointInfo(self.robot, i)[1].decode(): i for i in range(n)}
        lnames = {p.getJointInfo(self.robot, i)[12].decode(): i for i in range(n)}
        self.arm_j = [names["joint%d" % k] for k in range(1, 6)]
        self.grip_j = [names[k] for k in ("drive_joint", "left_finger_joint", "left_inner_knuckle_joint",
                                          "right_outer_knuckle_joint", "right_finger_joint",
                                          "right_inner_knuckle_joint")]
        self.tcp_link = lnames["link_tcp"]
        self.movable = sorted(self.arm_j + self.grip_j)
        info = [p.getJointInfo(self.robot, j) for j in self.movable]
        self.lo = [i[8] for i in info]
        self.hi = [i[9] for i in info]
        self.arm_lo = [p.getJointInfo(self.robot, j)[8] for j in self.arm_j]
        self.arm_hi = [p.getJointInfo(self.robot, j)[9] for j in self.arm_j]
        self.arm_lo, self.arm_hi = np.array(self.arm_lo), np.array(self.arm_hi)
        plan = _yaml(self.cfg_paths["planning"]) if os.path.exists(self.cfg_paths["planning"]) else {}
        self.limits = traj.load_limits(plan, [p.getJointInfo(self.robot, j)[11] for j in self.arm_j])

        self.cube = None                 # ACTIVE cube body (attach / detach / set_cube_pose / cube_pose_true act on it)
        self.cubes = {}                  # cube number -> body (multi-cube mode); the legacy single cube is not in here
        self.cube_num = None
        self.cube_attached = None
        self._T_tcp_cube = None
        self.calib_tag_body = None
        self.T_tcp_tag = None
        self.belt = None                 # sim.conveyor.Conveyor (added by add_conveyor; None = no belt, the default)
        self._quad_off = np.eye(4)
        self.set_arm(init_q if init_q is not None else self.workspace.get("ready_joint_state_rad", [0, -0.5, -0.5, 0, 0]))
        self.set_gripper(0.0)
        self._build_stand()

    # ---------------- camera stand (visual only) ----------------
    def _build_stand(self):
        p = self.p
        cam = _yaml(self.cfg_paths["camera"])
        eye = np.array(cam["mount"]["eye"], float)
        tgt = np.array(cam["mount"]["target"], float)
        fwd = (tgt - eye) / np.linalg.norm(tgt - eye)
        h = eye[2] - self.table_top_z
        px, py = eye[0] + 0.07, eye[1]
        alu = [0.75, 0.77, 0.8, 1]
        pole = p.createMultiBody(0, -1, p.createVisualShape(pb.GEOM_CYLINDER, radius=0.008, length=h, rgbaColor=alu),
                                 [px, py, self.table_top_z + h / 2])
        base = p.createMultiBody(0, -1, p.createVisualShape(pb.GEOM_BOX, halfExtents=[0.04, 0.04, 0.005],
                                                            rgbaColor=[0.62, 0.64, 0.67, 1]),
                                 [px, py, self.table_top_z + 0.005])
        x = np.cross(fwd, [0, 0, 1.0]); x /= np.linalg.norm(x)
        y = np.cross(fwd, x)
        R = np.stack([fwd, x, y], 1)  # box x axis -> viewing direction
        # C270-sized body (25 deep x 70 wide x 30 tall mm), sits just behind the optical centre
        body = p.createMultiBody(0, -1, p.createVisualShape(pb.GEOM_BOX, halfExtents=[0.0125, 0.035, 0.015],
                                                            rgbaColor=[0.22, 0.23, 0.25, 1]),
                                 (eye - 0.02 * fwd).tolist(), quat_from_R(R))
        self.stand = [pole, base, body]

    # ---------------- textured bodies ----------------
    def _textured_quad(self, img, width, height, name):
        """Visual-only flat quad. Frame: z out of face, x right, y up (as viewed). img row0 = top."""
        fimg = os.path.join(CACHE, name + ".png")
        cv2.imwrite(fimg, img)
        fobj = os.path.join(CACHE, name + ".obj")
        c = [(-.5, -.5, 0), (.5, -.5, 0), (.5, .5, 0), (-.5, .5, 0)]
        _write_obj(fobj, [(c, [(0, 0), (1, 0), (1, 1), (0, 1)], (0, 0, 1))])
        p = self.p
        vs = p.createVisualShape(pb.GEOM_MESH, fileName=fobj, meshScale=[width, height, 1], rgbaColor=[1, 1, 1, 1])
        b = p.createMultiBody(0, -1, vs, [5, 5, 5])
        p.changeVisualShape(b, -1, textureUniqueId=p.loadTexture(fimg))
        return b

    def _set_pose(self, body, M):
        self.p.resetBasePositionAndOrientation(body, M[:3, 3].tolist(), quat_from_R(M[:3, :3]))

    def _build_cube_assets(self, number=None):
        """Texture atlas + mesh. number=None: legacy plain white cube with cube_model.yaml IDs (unchanged).
        number=n: cube n of cubes.yaml: its own tag IDs, tinted quiet zone, full-colour bottom face."""
        cc = self.cube_cfg
        e = self.edge
        tile = 512
        frac = cc["tag_size_m"] / e
        faces = cc["faces"]
        ce = self.cube_by_num.get(number) if number is not None else None
        tint = self.cubes_cfg.get("tint", {}) if ce else {}
        strength = float(tint.get("margin_strength", 0.0))
        edge_px = int(round(float(tint.get("edge_band_frac", 0.0)) * tile))
        tag_px = int(round(tile * frac / 8)) * 8
        atlas = np.full((2 * tile, 3 * tile, 3), 255, np.uint8)  # 3 cols x 2 rows, BGR
        for k, fc in enumerate(faces):  # slots 0..4 tags; slot 5 = bottom
            r, c = divmod(k, 3)
            tid = int(ce["face_ids"][k]) if ce else fc["id"]
            g = _tag_tile(tid, tile, frac)
            t3 = (_tint_tile(g, tag_px, ce["color_rgb"], strength, edge_px) if ce
                  else cv2.cvtColor(g, cv2.COLOR_GRAY2BGR))
            atlas[r * tile:(r + 1) * tile, c * tile:(c + 1) * tile] = t3
        if ce:      # bottom face (slot 5, col 2 row 1): full cube colour
            atlas[tile:, 2 * tile:] = np.array(ce["color_rgb"][::-1], np.uint8)
        # file name carries the tint (PyBullet caches textures by file name)
        fimg = os.path.join(CACHE, "cube_atlas.png" if ce is None else "cube_atlas_%d_t%03d_e%02d.png" % (
            number, int(round(100 * strength)), edge_px))
        cv2.imwrite(fimg, atlas)
        h = e / 2

        def uv_of(k, a, b):  # tag-plane coords (x right, y up) -> atlas uv (v up from image bottom)
            r, c = divmod(k, 3)
            u = (c + (a / e + 0.5)) / 3
            v = 1.0 - (r + (1.0 - (b / e + 0.5))) / 2
            return (u, v)

        quads = []
        for k, fc in enumerate(faces):
            R = np.array(fc["R_cube_tag"], float)
            t = np.array(fc["t_cube_tag"], float)
            ab = [(-h, -h), (h, -h), (h, h), (-h, h)]
            quads.append(([R @ np.array([a, b, 0.0]) + t for a, b in ab], [uv_of(k, a, b) for a, b in ab], R[:, 2]))
        buv = [(2.5 / 3, 1 - 1.5 / 2)] * 4  # centre of slot 5 (col 2,row 1)
        quads.append(([(-h, h, -h), (h, h, -h), (h, -h, -h), (-h, -h, -h)], buv, (0, 0, -1)))
        fobj = os.path.join(CACHE, "cube_%0.4f.obj" % e)
        _write_obj(fobj, quads)
        return fobj, fimg

    # ---------------- cube ----------------
    def spawn_cube(self, x, y, yaw, number=None, z=None):
        """number=None: the legacy single white cube (replaces the active cube). number=n: coloured identified cube n
        (replaces only cube n) and it becomes the active cube. z = centre height (default: resting on the table)."""
        p = self.p
        if number is None:
            if self.cube is not None:
                self.detach_cube()
                self.remove(self.cube)
        elif number in self.cubes:
            if self.cube == self.cubes[number]:
                self.detach_cube()
            self.remove(self.cubes[number])
        fobj, fimg = self._build_cube_assets(number)
        e = self.edge
        vs = p.createVisualShape(pb.GEOM_MESH, fileName=fobj, meshScale=[1, 1, 1], rgbaColor=[1, 1, 1, 1])
        cs = p.createCollisionShape(pb.GEOM_BOX, halfExtents=[e / 2] * 3)
        q = p.getQuaternionFromEuler([0, 0, yaw])
        zc = self.table_top_z + e / 2 + 0.002 if z is None else z
        body = p.createMultiBody(float(self.cube_cfg["mass_kg"]), cs, vs, [x, y, zc], q)
        p.changeVisualShape(body, -1, textureUniqueId=p.loadTexture(fimg))
        p.changeDynamics(body, -1, lateralFriction=0.8, rollingFriction=0.001, spinningFriction=0.001)
        self.cube = body
        if number is not None:
            self.cubes[number], self.cube_num = body, number
        for _ in range(240):
            p.stepSimulation()
        return body

    def activate_cube(self, number):
        """Make cube `number` the active cube (the one attach/detach/cube_pose_true/set_cube_pose act on)."""
        if self.cube_attached is not None:
            self.detach_cube()
        self.cube, self.cube_num = self.cubes[number], number

    def cube_pose_true(self, number=None):
        """4x4 world<-cube. GROUND TRUTH, for scoring only. number=None: the active cube."""
        body = self.cube if number is None else self.cubes[number]
        pos, q = self.p.getBasePositionAndOrientation(body)
        return T_from(pos, q)

    # ---------------- arm ----------------
    def arm_q(self):
        return [self.p.getJointState(self.robot, j)[0] for j in self.arm_j]

    def set_arm(self, q5):
        for j, q in zip(self.arm_j, q5):
            self.p.resetJointState(self.robot, j, float(q))
            self.p.setJointMotorControl2(self.robot, j, pb.POSITION_CONTROL, float(q), force=500)
        self._sync_attachments()

    def set_gripper(self, angle):
        for j in self.grip_j:
            self.p.resetJointState(self.robot, j, float(angle))
            self.p.setJointMotorControl2(self.robot, j, pb.POSITION_CONTROL, float(angle), force=50)
        self._sync_attachments()

    def tcp_T(self):
        s = self.p.getLinkState(self.robot, self.tcp_link, computeForwardKinematics=1)
        return T_from(s[4], s[5])

    def fk_tcp(self, q):
        """TCP pose for arm config q (arm state restored afterwards)."""
        old = self.arm_q()
        for j, v in zip(self.arm_j, q):
            self.p.resetJointState(self.robot, j, float(v))
        T = self.tcp_T()
        for j, v in zip(self.arm_j, old):
            self.p.resetJointState(self.robot, j, v)
        return T

    def ik_tcp(self, T_target, seed=None, tries=8, pos_tol=5e-4, rot_tol=math.radians(1.0)):
        """PyBullet IK onto link_tcp (position + orientation), joint limits, verified by FK.
        Returns (ok, q5, pos_err_m, rot_err_rad). ok = pos<1 mm and rot<2 deg.
        Arm state is restored. 5-DOF: not every orientation is reachable (rot_err tells)."""
        p = self.p
        old = self.arm_q()
        old_all = [p.getJointState(self.robot, j)[0] for j in self.movable]
        seed = list(old if seed is None else seed)
        rng = np.random.default_rng(1)
        pos = T_target[:3, 3].tolist()
        quat = quat_from_R(T_target[:3, :3])
        rngs = [h - l for l, h in zip(self.lo, self.hi)]
        best = None
        self.last_tries = 0
        for k in range(tries):
            self.last_tries = k + 1
            s = seed if k == 0 else [float(np.clip(v + rng.normal(0, 0.5), l, h))
                                     for v, l, h in zip(seed, self.arm_lo, self.arm_hi)]
            full = list(s) + [0.0] * 6
            for j, v in zip(self.movable, full):
                p.resetJointState(self.robot, j, v)
            sol = p.calculateInverseKinematics(
                self.robot, self.tcp_link, pos, quat, lowerLimits=self.lo, upperLimits=self.hi,
                jointRanges=rngs, restPoses=full, maxNumIterations=400, residualThreshold=1e-6)
            q = [float(np.clip(v, l, h)) for v, l, h in zip(sol[:5], self.arm_lo, self.arm_hi)]
            T = self.fk_tcp(q)
            pe = float(np.linalg.norm(T[:3, 3] - T_target[:3, 3]))
            re = rot_err_rad(T[:3, :3], T_target[:3, :3])
            score = pe + 0.05 * re
            if best is None or score < best[0]:
                best = (score, q, pe, re)
            if pe < pos_tol and re < rot_tol:
                break
        for j, v in zip(self.movable, old_all):      # restore arm AND gripper joints (IK resets them)
            p.resetJointState(self.robot, j, v)
        _, q, pe, re = best
        return (pe < 1e-3 and re < math.radians(2.0)), q, pe, re

    @staticmethod
    def quintic(t):
        """Minimum-jerk time scaling s(t), t in [0,1]."""
        return 10 * t ** 3 - 15 * t ** 4 + 6 * t ** 5

    def ik_refine(self, T_target, q_seed, iters=12, w_rot=0.1):
        """Damped Newton IK polish on link_tcp (pos + w_rot*rot residual) from q_seed, to ~1e-10 m when reachable.
        Used per step of Cartesian segments (smooth q). Arm state restored. -> (q5, pos_err_m, rot_err_rad)."""
        p, rb, tcp = self.p, self.robot, self.tcp_link
        old = self.arm_q()
        q = np.array(q_seed, float)
        Rt = T_target[:3, :3]
        zeros = [0.0] * len(self.movable)
        pe = re = 0.0
        for it in range(iters + 1):
            for j, v in zip(self.arm_j, q):
                p.resetJointState(rb, j, float(v))
            st = p.getLinkState(rb, tcp, computeForwardKinematics=1)
            pos = np.array(st[4])
            R = np.array(pb.getMatrixFromQuaternion(st[5])).reshape(3, 3)
            ev = T_target[:3, 3] - pos
            er = cv2.Rodrigues(Rt @ R.T)[0].ravel()
            pe, re = float(np.linalg.norm(ev)), float(np.linalg.norm(er))
            if it == iters or (pe < 1e-10 and re < 1e-9):
                break
            Jt, Jr = p.calculateJacobian(rb, tcp, [0, 0, 0], list(q) + [0.0] * (len(zeros) - 5), zeros, zeros)
            J = np.vstack([np.array(Jt)[:, :5], w_rot * np.array(Jr)[:, :5]])
            e = np.concatenate([ev, w_rot * er])
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-10 * np.eye(6), e)
            q = q + dq
            if float(np.max(np.abs(dq))) < 1e-12:
                break
        for j, v in zip(self.arm_j, old):
            p.resetJointState(rb, j, v)
        return q, pe, re

    def add_conveyor(self):
        """Build the conveyor belt of workspace.yaml `conveyor:` in this world (idempotent). Not built by default, so
        the scored single-cube demo / calibration never see it. -> Conveyor (or None when the config has none)."""
        if self.belt is None and self.workspace.get("conveyor"):
            from sim.conveyor import Conveyor, ConveyorCfg
            self.belt = Conveyor(ConveyorCfg(self.workspace["conveyor"], self.table_top_z, self.edge))
            self.belt.build(self.p)
        return self.belt

    def idle(self, seconds):
        """Let sim time pass with the arm / gripper left as they are (belt keeps moving). Paced like play() in the GUI."""
        for _ in range(int(round(seconds / self.dt_step))):
            self._tick()

    def advance_fast(self, n):
        """n sim steps without GUI pacing / callbacks (app idle loop: belt animation in real time)."""
        for _ in range(int(n)):
            if self.belt is not None:
                self.belt.step(self, self.dt_step)
            self.p.stepSimulation()
            self.sim_t += self.dt_step

    def _tick(self):
        if self.belt is not None:
            self.belt.step(self, self.dt_step)
        self.p.stepSimulation()
        self.sim_t += self.dt_step
        if self.on_step is not None:
            self.on_step()
        if self.gui:        # pace to dt/speed (deadline based, so on_step cost is absorbed); speed = playback only
            now = time.perf_counter()
            if self._t_next < now - 0.1:
                self._t_next = now
            self._t_next += self.dt_step / max(self.speed, 0.05)
            sl = self._t_next - time.perf_counter()
            if sl > 0:
                time.sleep(sl)

    def play(self, seg):
        """Play a trajectory.Segment: one set_arm / set_gripper + physics step per sim step (kinematic, teleport)."""
        seg.t0 = self.sim_t
        seg.attached = self.cube_attached is not None
        if self.on_segment is not None:
            self.on_segment(seg)
        arm_moves = seg.kind != "gripper-quintic"
        for i in range(seg.n):
            if arm_moves:
                self.set_arm(seg.Q[i])
                if seg.kind == "cartesian-tracking" and seg.g0 != seg.g1:       # tracking while the gripper closes
                    self.set_gripper(seg.G[i])
            else:
                self.set_gripper(seg.G[i])
            self.qd = seg.QD[i]
            self._tick()
        self.qd = np.zeros(5)
        seg.T_tcp_end_pb = self.tcp_T()
        return seg

    def move_joints(self, q_from, q_to, name="move"):
        """Joint-space quintic within the velocity / acceleration limits (time from the limits, not from a step count)."""
        a, b = np.array(q_from, float), np.array(q_to, float)
        if float(np.max(np.abs(b - a))) < 1e-6:
            return None
        g = self.p.getJointState(self.robot, self.grip_j[0])[0]
        return self.play(traj.plan_joint(name, a, b, self.limits, g))

    def move_line(self, q_to, ik_world=None, name="line"):
        """Straight Cartesian line from the current TCP pose to FK(q_to) (IK per step on `ik_world`, default self)."""
        a = np.array(self.arm_q(), float)
        g = self.p.getJointState(self.robot, self.grip_j[0])[0]
        return self.play(traj.plan_line(name, a, q_to, self.limits, ik_world or self, g))

    def move_gripper(self, ang, name="gripper"):
        """Eased gripper open/close over limits.grip_time (PLACEHOLDER)."""
        cur = self.p.getJointState(self.robot, self.grip_j[0])[0]
        if abs(ang - cur) < 1e-9:
            return None
        return self.play(traj.plan_gripper(name, self.arm_q(), cur, ang, self.limits))

    def set_cube_pose(self, x, y, yaw, settle=240, z=None, number=None):
        """Teleport a cube (default: the active one) to (x, y, yaw) on the table (or at height z) and let it settle."""
        p = self.p
        if number is not None and self.cubes[number] != self.cube:
            body = self.cubes[number]
        else:
            self.detach_cube()
            body = self.cube
        zc = self.table_top_z + self.edge / 2 + 0.002 if z is None else z
        p.resetBasePositionAndOrientation(body, [x, y, zc], p.getQuaternionFromEuler([0, 0, yaw]))
        p.resetBaseVelocity(body, [0, 0, 0], [0, 0, 0])
        for _ in range(settle):
            p.stepSimulation()

    # ---------------- kinematic attach (NOT a physical grasp) ----------------
    def attach_cube(self):
        """KINEMATIC ATTACH (L2.5 label): fixed constraint cube<->link_tcp plus pose re-sync.
        Not a physical grasp."""
        p = self.p
        if self.cube is None or self.cube_attached is not None:
            return
        self._T_tcp_cube = Tinv(self.tcp_T()) @ self.cube_pose_true()
        for l in range(-1, p.getNumJoints(self.robot)):
            p.setCollisionFilterPair(self.robot, self.cube, l, -1, 0)
        Tc = self._T_tcp_cube
        self.cube_attached = p.createConstraint(
            self.robot, self.tcp_link, self.cube, -1, pb.JOINT_FIXED, [0, 0, 0],
            Tc[:3, 3].tolist(), [0, 0, 0], quat_from_R(Tc[:3, :3].T), [0, 0, 0, 1])
        p.changeConstraint(self.cube_attached, maxForce=1e4)

    def detach_cube(self):
        p = self.p
        if self.cube_attached is None:
            return
        p.removeConstraint(self.cube_attached)
        self.cube_attached = None
        for l in range(-1, p.getNumJoints(self.robot)):
            p.setCollisionFilterPair(self.robot, self.cube, l, -1, 1)

    def _sync_attachments(self):
        if self.cube_attached is not None and self.cube is not None:
            self._set_pose(self.cube, self.tcp_T() @ self._T_tcp_cube)
            self.p.resetBaseVelocity(self.cube, [0, 0, 0], [0, 0, 0])
        if self.calib_tag_body is not None:
            self._set_pose(self.calib_tag_body, self.tcp_T() @ self.T_tcp_tag)

    # ---------------- calibration props ----------------
    def spawn_checkerboard(self, T_world_board, inner=(9, 6), square=0.025):
        """Flat textured board (10x7 squares, 1-square white margin). Board frame: z out of face,
        x right, y up, origin = board centre. Returns (body, corners Nx3 in board frame, ordered like
        cv2.findChessboardCorners: row-major from top-left, `inner[0]` per row)."""
        cols, rows = inner[0] + 1, inner[1] + 1
        m, px = 1, 80
        img = np.full(((rows + 2 * m) * px, (cols + 2 * m) * px), 255, np.uint8)
        for r in range(rows):
            for c in range(cols):
                if (r + c) % 2 == 0:
                    img[(r + m) * px:(r + m + 1) * px, (c + m) * px:(c + m + 1) * px] = 0
        w, h = (cols + 2 * m) * square, (rows + 2 * m) * square
        b = self._textured_quad(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), w, h, "checker_%dx%d" % inner)
        self._set_pose(b, T_world_board)
        pts = [[-cols * square / 2 + (c + 1) * square, rows * square / 2 - (r + 1) * square, 0.0]
               for r in range(inner[1]) for c in range(inner[0])]
        return b, np.array(pts)

    def spawn_calib_tag(self, tag_id=None, T_tcp_tag=None):
        """Calibration tag on the gripper. The mount T_tcp_tag is chosen HERE and is UNKNOWN to the
        solver (it estimates it). Default: beside the gripper body, normal = -z_tcp (faces up when the
        tool points down). Tag frame: OpenCV ArUco convention (z out of face)."""
        ct = self.cube_cfg["calib_tag"]
        tag_id = ct["id"] if tag_id is None else tag_id
        size = float(ct["tag_size_m"])
        if T_tcp_tag is None:
            T_tcp_tag = np.eye(4)
            T_tcp_tag[:3, :3] = np.diag([1, -1, -1.0])
            T_tcp_tag[:3, 3] = [0.07, 0.0, -0.11]
        self.T_tcp_tag = T_tcp_tag
        total = size * self.cube_cfg["tag_cells_total"] / self.cube_cfg["tag_cells_black"]
        img = _tag_tile(tag_id, 640, size / total)
        self.calib_tag_body = self._textured_quad(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), total, total,
                                                  "calib_tag_%d" % tag_id)
        self._sync_attachments()
        return self.calib_tag_body

    def remove(self, body):
        if body is None:
            return
        if body == self.calib_tag_body:
            self.calib_tag_body = None
        if body == self.cube:
            self.cube = None
        for n in [n for n, b in self.cubes.items() if b == body]:
            del self.cubes[n]
            if self.cube_num == n:
                self.cube_num = None
        self.p.removeBody(body)

    # ---------------- visual-only zone overlays (sim_app) ----------------
    def add_zone_overlays(self):
        """Thin flat semi-transparent boxes on the table (no collision): green PICK, orange PLACE, target disc."""
        p, ws = self.p, self.workspace
        if not ws:
            return []
        z0 = self.table_top_z
        bodies = []

        def box(cx, cy, hx, hy, hz, zc, rgba):
            vs = p.createVisualShape(pb.GEOM_BOX, halfExtents=[hx, hy, hz], rgbaColor=rgba)
            bodies.append(p.createMultiBody(0, -1, vs, [cx, cy, zc]))

        def zone(b, rgb):
            (x0, x1), (y0, y1) = b["x_m"], b["y_m"]
            cx, cy, hx, hy = (x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 2, (y1 - y0) / 2
            box(cx, cy, hx, hy, 0.0005, z0 + 0.0005, [*rgb, 0.30])
            e = 0.003      # border strips
            box(cx, y0, hx, e / 2, 0.001, z0 + 0.001, [*rgb, 0.9])
            box(cx, y1, hx, e / 2, 0.001, z0 + 0.001, [*rgb, 0.9])
            box(x0, cy, e / 2, hy, 0.001, z0 + 0.001, [*rgb, 0.9])
            box(x1, cy, e / 2, hy, 0.001, z0 + 0.001, [*rgb, 0.9])
        zone(ws["cube_workspace_box"], (0.20, 0.80, 0.40))
        from sim.zones import place_zones
        for zn in place_zones(ws):
            rgb = tuple(c / 255.0 for c in zn["color_rgb"])
            zone(zn, rgb)
            pt = zn["target"]
            vs = p.createVisualShape(pb.GEOM_CYLINDER, radius=0.012, length=0.002, rgbaColor=[*rgb, 0.95])
            bodies.append(p.createMultiBody(0, -1, vs, [pt["x_m"], pt["y_m"], z0 + 0.0015]))
            # zone number plate (visual only): digit on a coloured square lying on the table at the zone's near corner
            img = np.full((128, 128, 3), [int(c * 255) for c in rgb[::-1]], np.uint8)
            cv2.putText(img, str(zn["number"]), (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 3.2, (255, 255, 255), 8, cv2.LINE_AA)
            # unrotated: plate frame is x right / y up, same as the table view with +x to the right
            plate = self._textured_quad(img, 0.045, 0.045, "zone_label_%d" % zn["number"])
            M = np.eye(4)
            M[:3, 3] = [zn["x_m"][0] + 0.03, zn["y_m"][0] + 0.03, z0 + 0.003]
            self._set_pose(plate, M)
            bodies.append(plate)
        return bodies

    def is_connected(self):
        try:
            return bool(self.p.isConnected())
        except Exception:
            return False

    def close(self):
        if self.is_connected():
            try:
                self.p.disconnect()
            except pb.error:
                pass
