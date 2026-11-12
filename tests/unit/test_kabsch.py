"""Tests for calibration_tools kabsch + handeye_A. Run: python -m pytest tests/unit/test_kabsch.py
(or plain: python tests/unit/test_kabsch.py)."""
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src" / "calibration_tools"))
from calibration_tools import umeyama, umeyama_ransac, rigid_error, calibrate_eye_to_hand  # noqa: E402


def rand_T(rng, tmax=0.5):
    R, _ = cv2.Rodrigues(rng.normal(size=3) * 0.9)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, rng.uniform(-tmax, tmax, 3)
    return T


def test_umeyama_recovers_rigid():
    rng = np.random.default_rng(0)
    T = rand_T(rng)
    P = rng.normal(size=(20, 3))
    Q = P @ T[:3, :3].T + T[:3, 3]
    R, t, s = umeyama(P, Q)
    assert np.allclose(R, T[:3, :3], atol=1e-9) and np.allclose(t, T[:3, 3], atol=1e-9) and s == 1.0
    assert np.isclose(np.linalg.det(R), 1.0)


def test_umeyama_scale_and_reflection_guard():
    rng = np.random.default_rng(1)
    T = rand_T(rng)
    P = rng.normal(size=(30, 3))
    Q = 1.7 * P @ T[:3, :3].T + T[:3, 3]
    R, t, s = umeyama(P, Q, with_scale=True)
    assert abs(s - 1.7) < 1e-9 and np.allclose(R, T[:3, :3], atol=1e-9)
    # coplanar points must not yield a reflection
    P2 = P.copy()
    P2[:, 2] = 0
    R2, _, _ = umeyama(P2, P2 @ T[:3, :3].T + T[:3, 3])
    assert np.isclose(np.linalg.det(R2), 1.0)


def test_ransac_rejects_outliers():
    rng = np.random.default_rng(2)
    T = rand_T(rng)
    P = rng.normal(size=(40, 3))
    Q = P @ T[:3, :3].T + T[:3, 3]
    Q[:8] += rng.uniform(0.2, 0.5, (8, 3))
    R, t, s, m = umeyama_ransac(P, Q)
    assert m[8:].all() and not m[:8].any()
    Te = np.eye(4)
    Te[:3, :3], Te[:3, 3] = R, t
    e_mm, e_deg = rigid_error(Te, T)
    assert e_mm < 1e-6 and e_deg < 1e-6


def _handeye_data(rng, n, noise_t=0.0, noise_r_deg=0.0):
    X = rand_T(rng, 0.1)      # T_tcp_tag
    Y = rand_T(rng, 1.0)      # T_base_cam
    A, B = [], []
    for _ in range(n):
        a = rand_T(rng, 0.4)
        b = np.linalg.inv(Y) @ a @ X
        if noise_t or noise_r_deg:
            dR, _ = cv2.Rodrigues(rng.normal(size=3) * np.radians(noise_r_deg) / np.sqrt(3))
            b = b.copy()
            b[:3, :3] = dR @ b[:3, :3]
            b[:3, 3] += rng.normal(size=3) * noise_t / np.sqrt(3)
        A.append(a)
        B.append(b)
    return A, B, X, Y


def test_handeye_noise_free():
    rng = np.random.default_rng(3)
    A, B, X, Y = _handeye_data(rng, 15)
    r = calibrate_eye_to_hand(A, B)
    e_mm, e_deg = rigid_error(r["T_base_cam"], Y)
    assert e_mm < 1.0 and e_deg < 0.1, (e_mm, e_deg)
    assert e_mm < 1e-6 and e_deg < 1e-6
    ex_mm, ex_deg = rigid_error(r["T_tcp_tag"], X)
    assert ex_mm < 1e-6 and ex_deg < 1e-6
    assert r["trans_resid_mm"] < 1e-6


def test_handeye_small_noise():
    rng = np.random.default_rng(4)
    A, B, X, Y = _handeye_data(rng, 40, noise_t=0.001, noise_r_deg=0.2)
    r = calibrate_eye_to_hand(A, B)
    e_mm, e_deg = rigid_error(r["T_base_cam"], Y)
    assert e_mm < 3.0 and e_deg < 0.5, (e_mm, e_deg)


if __name__ == "__main__":
    n = 0
    for k, v in list(globals().items()):
        if k.startswith("test_") and callable(v):
            v()
            n += 1
            print("ok", k)
    print(n, "passed")
