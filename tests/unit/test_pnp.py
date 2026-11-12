"""Synthetic tests for cube_perception.pnp. Run: python -m pytest tests/unit/test_pnp.py
(or plain: python tests/unit/test_pnp.py)."""
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src" / "cube_perception"))
from cube_perception import (load_cube_model, estimate_cube_pose, to_base, fold_yaw,  # noqa: E402
                             median_pose, corners_from_pupil)

K = np.array([[600.0, 0, 320], [0, 600.0, 240], [0, 0, 1]])
MODEL = load_cube_model()


def rot(axis, deg):
    R, _ = cv2.Rodrigues(np.array(axis, float) * np.radians(deg))
    return R


def make_T(R, t):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def visible_ids(T_cam_cube):
    out = []
    for i, T in MODEL.T_cube_tag.items():
        n = T_cam_cube[:3, :3] @ T[:3, 2]
        c = T_cam_cube[:3, :3] @ T[:3, 3] + T_cam_cube[:3, 3]
        if n @ c < -0.2 * np.linalg.norm(c):
            out.append(i)
    return out


def project(T_cam_cube, ids, noise=0.0, rng=None):
    dets = []
    for i in ids:
        P = MODEL.corners_cube[i] @ T_cam_cube[:3, :3].T + T_cam_cube[:3, 3]
        uv = (P @ K.T)
        uv = uv[:, :2] / uv[:, 2:3]
        if noise:
            uv = uv + rng.normal(0, noise, uv.shape)
        dets.append((i, uv))
    return dets


def pose_err(Ta, Tb):
    d = np.linalg.inv(Ta) @ Tb
    return (np.linalg.norm(d[:3, 3]), np.degrees(np.linalg.norm(cv2.Rodrigues(d[:3, :3])[0])))


# Cube seen from above-front: +z (id0) tilted to camera, plus side faces.
T_TRUE = make_T(rot([1, 0, 0], 200) @ rot([0, 0, 1], 30) @ rot([0, 1, 0], 25), [0.03, -0.02, 0.45])


def _check_n(n):
    vis = visible_ids(T_TRUE)
    assert len(vis) >= 3, vis
    r = estimate_cube_pose(project(T_TRUE, vis[:n]), K, None, MODEL)
    assert r.valid and r.invalid_reason == ""
    assert len(r.tag_ids_used) == n
    te, re = pose_err(T_TRUE, r.T_cam_cube)
    assert te < 1e-6 and re < 1e-6, (te, re)
    assert r.rms_px < 1e-6


def test_one_tag():
    _check_n(1)


def test_two_tags():
    _check_n(2)


def test_three_tags():
    _check_n(3)


def test_noise_small_error():
    rng = np.random.default_rng(1)
    vis = visible_ids(T_TRUE)
    r = estimate_cube_pose(project(T_TRUE, vis, noise=0.3, rng=rng), K, None, MODEL)
    te, re = pose_err(T_TRUE, r.T_cam_cube)
    assert r.valid and te < 0.005 and re < 3.0


def test_no_tags_and_unknown_id():
    assert estimate_cube_pose([], K, None, MODEL).invalid_reason == "NO_TAGS"
    d = project(T_TRUE, visible_ids(T_TRUE)[:1])
    r = estimate_cube_pose([(99, d[0][1])], K, None, MODEL)
    assert not r.valid and r.invalid_reason == "UNKNOWN_ID"
    # unknown id mixed with a good one is ignored
    r = estimate_cube_pose([(99, d[0][1]), d[0]], K, None, MODEL)
    assert r.valid and r.tag_ids_used == [d[0][0]]


def test_occlusion_subset_consistent():
    vis = visible_ids(T_TRUE)
    full = estimate_cube_pose(project(T_TRUE, vis), K, None, MODEL)
    part = estimate_cube_pose(project(T_TRUE, vis[1:]), K, None, MODEL)
    assert pose_err(full.T_cam_cube, part.T_cam_cube)[0] < 1e-6


def test_high_reproj_rejected():
    vis = visible_ids(T_TRUE)[:2]
    d = project(T_TRUE, vis)
    d[1] = (d[1][0], d[1][1] + np.array([25.0, -30.0]))   # corrupt one tag
    r = estimate_cube_pose(d, K, None, MODEL)
    assert not r.valid and r.invalid_reason == "HIGH_REPROJ"
    r2 = estimate_cube_pose(d, K, None, MODEL, cfg={"max_reproj_rms_px": 1e3})
    assert r2.valid


def test_fold_yaw():
    q = np.pi / 4
    assert abs(fold_yaw(0.0)) < 1e-12
    assert abs(fold_yaw(np.pi / 2)) < 1e-12
    assert abs(fold_yaw(np.radians(50)) - np.radians(-40)) < 1e-12
    assert abs(fold_yaw(-q) - (-q)) < 1e-12          # -45 inclusive
    assert abs(fold_yaw(q) - (-q)) < 1e-12           # +45 excluded -> -45
    for y in np.linspace(-7, 7, 101):
        assert -q - 1e-12 <= fold_yaw(y) < q + 1e-12


def test_to_base_table_constraint():
    T_base_cam = make_T(rot([1, 0, 0], 180) @ rot([0, 0, 1], 10), [0.3, 0.0, 0.6])
    # upright cube on table z=0 at (0.35, 0.05), yaw 20 deg, with small tilt noise
    T_bc_true = make_T(rot([0, 0, 1], 20), [0.35, 0.05, 0.025])
    T_cam_cube = np.linalg.inv(T_base_cam) @ make_T(rot([1, 0, 0], 3) @ rot([0, 0, 1], 20),
                                                    [0.351, 0.049, 0.030])
    T = to_base(T_base_cam, T_cam_cube, table_z=0.0, edge=0.05)
    assert abs(T[2, 3] - 0.025) < 1e-12
    assert np.allclose(T[:3, 2], [0, 0, 1])
    yaw = np.arctan2(T[1, 0], T[0, 0])
    assert abs(yaw - np.radians(20)) < np.radians(0.2)  # 3 deg tilt perturbs heading slightly
    # unconstrained = plain composition
    assert np.allclose(to_base(T_base_cam, T_cam_cube), T_base_cam @ T_cam_cube)
    # yaw 50 deg folds to -40 deg
    T2 = to_base(T_base_cam, np.linalg.inv(T_base_cam) @ make_T(rot([0, 0, 1], 50), [0.3, 0, 0.025]),
                 table_z=0.0, edge=0.05)
    assert abs(np.arctan2(T2[1, 0], T2[0, 0]) - np.radians(-40)) < 1e-9
    del T_bc_true


def test_median_pose():
    Ts = [make_T(rot([0, 0, 1], a), [0.1 + d, 0.2, 0.3]) for a, d in [(10, 0), (10, 0), (10.0, 0), (80, 0.5)]]
    M = median_pose(Ts)
    assert pose_err(M, Ts[0])[0] < 1e-9 and pose_err(M, Ts[0])[1] < 1e-6


def test_pupil_corner_order_matches_opencv():
    from pupil_apriltags import Detector
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    for tag_id in (0, 3, 10):
        m = cv2.aruco.generateImageMarker(d, tag_id, 80)
        img = np.full((400, 400), 255, np.uint8)
        img[100:180, 100:180] = m
        src = np.float32([[0, 0], [400, 0], [400, 400], [0, 400]])
        dst = np.float32([[60, 40], [330, 70], [350, 320], [40, 360]])
        w = cv2.warpPerspective(img, cv2.getPerspectiveTransform(src, dst), (400, 400), borderValue=255)
        c, ids, _ = cv2.aruco.ArucoDetector(d).detectMarkers(w)
        assert ids is not None and int(ids.ravel()[0]) == tag_id
        ref = c[0].reshape(4, 2)
        dets = Detector(families="tag36h11").detect(w)
        assert len(dets) == 1 and dets[0].tag_id == tag_id
        conv = corners_from_pupil(dets[0])
        assert np.abs(conv - ref).max() < 2.0, (conv, ref)


if __name__ == "__main__":
    n = 0
    for k, v in list(globals().items()):
        if k.startswith("test_") and callable(v):
            v()
            n += 1
            print("ok", k)
    print(n, "passed")
