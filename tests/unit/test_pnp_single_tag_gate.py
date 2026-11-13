"""Single-tag quality gate in cube_perception.pnp (synthetic, no simulator)."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src" / "cube_perception"))
from cube_perception import estimate_cube_pose, load_cube_model  # noqa: E402
import cv2  # noqa: E402

K = np.array([[1000.0, 0, 640], [0, 1000.0, 360], [0, 0, 1.0]])
TABLE_Z, EDGE = 0.0, 0.04


def _setup():
    m = load_cube_model()
    tid = sorted(m.corners_cube)[0]
    # camera looking straight down at 0.5 m; cube resting on the table, tag-id face up (tilt 0)
    T_base_cam = np.eye(4)
    T_base_cam[:3, :3] = np.diag([1.0, -1.0, -1.0])
    T_base_cam[:3, 3] = [0.0, 0.0, 0.5]
    return m, tid, T_base_cam


def _project(m, tid, T_cam_cube, shift=(0, 0)):
    rv, _ = cv2.Rodrigues(T_cam_cube[:3, :3])
    p, _ = cv2.projectPoints(m.corners_cube[tid].astype(np.float64), rv, T_cam_cube[:3, 3], K, None)
    return p.reshape(4, 2) + np.array(shift)


def _cube_in_cam(T_base_cam, z):
    Tb = np.eye(4)
    Tb[:3, 3] = [0.0, 0.0, z]
    # put the tag face towards the camera: rotate so the tag normal faces +z base (try both flips, keep the visible one)
    return np.linalg.inv(T_base_cam) @ Tb


def test_gate_accepts_good_and_rejects_bad_single_tag():
    m, tid, Tbc = _setup()
    ctx = {"image_size": (1280, 720), "T_base_cam": Tbc, "table_z": TABLE_Z, "edge": EDGE}
    z = TABLE_Z + EDGE / 2
    ok = None
    for R in (np.eye(3), cv2.Rodrigues(np.array([np.pi, 0, 0.0]))[0], cv2.Rodrigues(np.array([0, np.pi, 0.0]))[0]):
        T = _cube_in_cam(Tbc, z)
        T[:3, :3] = T[:3, :3] @ R
        c = _project(m, tid, T)
        if np.all((c > 0) & (c < [1280, 720])) and np.linalg.det(np.eye(3)) > 0:
            r = estimate_cube_pose([(tid, c)], K, None, m, ctx)
            if r.valid:
                ok = (T, c)
                break
    if ok is None:
        raise AssertionError("no visible synthetic tag pose found")
    T, c = ok
    # 1) border: shift the tag against the image border
    r = estimate_cube_pose([(tid, c + np.array([1279.0 - c[:, 0].max(), 0.0]))], K, None, m, ctx)
    assert not r.valid
    # 2) cube floating 15 mm above the table -> not resting
    T2 = T.copy()
    T2[:3, 3] = np.linalg.inv(Tbc)[:3, :3] @ np.array([0, 0, z + 0.015]) + np.linalg.inv(Tbc)[:3, 3]
    r2 = estimate_cube_pose([(tid, _project(m, tid, T2))], K, None, m, ctx)
    assert not r2.valid and r2.invalid_reason == "SINGLE_TAG_NOT_RESTING"
    # 3) heavy pixel noise -> reprojection gate
    rng = np.random.default_rng(0)
    r3 = estimate_cube_pose([(tid, c + rng.normal(0, 1.5, c.shape))], K, None, m, ctx)
    assert not r3.valid
    # 4) the gate never applies without a single tag: default cfg (no context) still accepts the clean one
    assert estimate_cube_pose([(tid, c)], K, None, m).valid
