"""Fake Logitech C270 for PyBullet.  SIMULATION ONLY.

Pipeline per frame: PyBullet render at the TRUE pinhole K -> apply lens distortion
(so the image looks like a real lens) -> blur -> brightness jitter -> signal-dependent
noise -> JPEG encode/decode (MJPEG).

!!! GROUND TRUTH WARNING !!!
`K_true`, `dist_true`, `T_world_cam_true` are the simulator's secrets. They exist for
SCORING errors only. The perception / calibration pipeline MUST NOT read them; it must
use intrinsics from its own checkerboard calibration and extrinsics from hand-eye
calibration, exactly as with the real camera.

Camera frame = OpenCV (x right, y down, z forward). World = robot base frame.
"""
import math
import cv2
import numpy as np
import yaml


def load_cfg(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


class C270SimCamera:
    def __init__(self, cfg, rng=None):
        self.cfg = cfg
        self.rng = rng if rng is not None else np.random.default_rng(0)
        self.W, self.H = int(cfg["width"]), int(cfg["height"])
        diag = math.hypot(self.W, self.H)
        f = (diag / 2) / math.tan(math.radians(cfg["diag_fov_deg"]) / 2)
        self._K = np.array([[f, 0, self.W / 2], [0, f, self.H / 2], [0, 0, 1.0]])
        self._dist = np.array(cfg["distortion"], float)
        m = cfg["mount"]
        self._eye, self._tgt, self._up = m["eye"], m["target"], m["up"]
        self._near, self._far = cfg["near_m"], cfg["far_m"]
        # pybullet matrices (true pinhole: vertical fov from fy)
        import pybullet as p
        self._p = p
        self.view = p.computeViewMatrix(self._eye, self._tgt, self._up)
        fov_v = 2 * math.degrees(math.atan((self.H / 2) / f))
        self.proj = p.computeProjectionMatrixFOV(fov_v, self.W / self.H, self._near, self._far)
        V = np.array(self.view).reshape(4, 4, order="F")
        self._T_world_cam = np.linalg.inv(V) @ np.diag([1, -1, -1, 1.0])
        # distortion map: out(u,v) = ideal(undistort(u,v))
        uu, vv = np.meshgrid(np.arange(self.W, dtype=np.float32), np.arange(self.H, dtype=np.float32))
        pts = np.stack([uu.ravel(), vv.ravel()], 1).reshape(-1, 1, 2)
        und = cv2.undistortPoints(pts, self._K, self._dist, None, None, self._K,
                                      (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 20, 1e-7))
        und = und.reshape(self.H, self.W, 2).astype(np.float32)
        self._mapx, self._mapy = und[..., 0], und[..., 1]

    # ---- ground truth (SCORING ONLY) ----
    @property
    def K_true(self):
        return self._K.copy()

    @property
    def dist_true(self):
        return self._dist.copy()

    @property
    def T_world_cam_true(self):
        return self._T_world_cam.copy()

    # ---- rendering ----
    def render(self, pyb_client, gui=False, apply_effects=True):
        """pyb_client: pybullet_utils BulletClient (or the `pybullet` module). Returns BGR uint8 HxWx3."""
        p = pyb_client
        renderer = self._p.ER_BULLET_HARDWARE_OPENGL if gui else self._p.ER_TINY_RENDERER
        img = p.getCameraImage(self.W, self.H, self.view, self.proj, renderer=renderer, shadow=0)[2]
        rgb = np.asarray(img, np.uint8).reshape(self.H, self.W, 4)[:, :, :3]
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        bgr = cv2.remap(bgr, self._mapx, self._mapy, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        if not apply_effects:
            return bgr
        c = self.cfg
        x = bgr.astype(np.float32)
        s = c["blur_sigma_px"]
        if s > 0:
            x = cv2.GaussianBlur(x, (0, 0), s)
        x *= 1.0 + self.rng.uniform(-c["brightness_jitter"], c["brightness_jitter"])
        var = c["noise"]["read_sigma_gray"] ** 2 + c["noise"]["shot_gain"] * np.clip(x, 0, 255)
        x += self.rng.standard_normal(x.shape).astype(np.float32) * np.sqrt(var)
        x = np.clip(x, 0, 255).astype(np.uint8)
        ok, buf = cv2.imencode(".jpg", x, [cv2.IMWRITE_JPEG_QUALITY, int(c["mjpeg_quality"])])
        return cv2.imdecode(buf, cv2.IMREAD_COLOR)

    @staticmethod
    def live_window(img, overlays=None, name="C270 sim", delay_ms=1):
        """cv2.imshow for --gui. overlays: list of (text,) strings or dicts
        {'quad': 4x2 pixels, 'label': str}. Honesty banner is always drawn."""
        out = img.copy()
        for o in overlays or []:
            if isinstance(o, str):
                continue
            q = np.asarray(o["quad"], np.float32).reshape(-1, 1, 2).astype(np.int32)
            cv2.polylines(out, [q], True, (0, 255, 0), 2)
            if o.get("label"):
                cv2.putText(out, o["label"], tuple(q[0, 0]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        y = 20
        for o in [o for o in (overlays or []) if isinstance(o, str)] + ["SIMULATION ONLY - C270 model, PLACEHOLDER noise"]:
            cv2.putText(out, o, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
            y += 20
        cv2.imshow(name, out)
        cv2.waitKey(delay_ms)
        return out
