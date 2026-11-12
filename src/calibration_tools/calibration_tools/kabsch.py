"""Kabsch / Umeyama rigid (or similarity) alignment: Q ~= s R P + t."""
from typing import Optional, Tuple

import numpy as np

DEFAULT_RANSAC = {
    "iters": 200,              # random minimal-sample trials
    "sample_size": 3,          # minimal points for a rigid fit
    "inlier_thresh_m": 0.005,  # residual (m) under which a point is an inlier
    "seed": 0,
}


def umeyama(P, Q, with_scale: bool = False):
    """Return (R, t, s) with Q ~= s*R@P + t (s=1 unless with_scale). P,Q: Nx3."""
    P = np.asarray(P, dtype=np.float64)
    Q = np.asarray(Q, dtype=np.float64)
    mp, mq = P.mean(0), Q.mean(0)
    Pc, Qc = P - mp, Q - mq
    Sg = Qc.T @ Pc / len(P)
    U, D, Vt = np.linalg.svd(Sg)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = float(np.trace(np.diag(D) @ S) / Pc.var(0).sum()) if with_scale else 1.0
    t = mq - s * R @ mp
    return R, t, s


def umeyama_ransac(P, Q, cfg: Optional[dict] = None, with_scale: bool = False):
    """RANSAC wrapper. Returns (R, t, s, inlier_mask); final refit on all inliers."""
    c = dict(DEFAULT_RANSAC)
    c.update(cfg or {})
    P = np.asarray(P, dtype=np.float64)
    Q = np.asarray(Q, dtype=np.float64)
    rng = np.random.default_rng(int(c["seed"]))
    n = len(P)
    best = np.zeros(n, bool)
    for _ in range(int(c["iters"])):
        idx = rng.choice(n, int(c["sample_size"]), replace=False)
        R, t, s = umeyama(P[idx], Q[idx], with_scale)
        r = np.linalg.norm(Q - (s * P @ R.T + t), axis=1)
        m = r < c["inlier_thresh_m"]
        if m.sum() > best.sum():
            best = m
    if best.sum() < int(c["sample_size"]):
        best = np.ones(n, bool)
    R, t, s = umeyama(P[best], Q[best], with_scale)
    return R, t, s, best


def _rotvec(R):
    """Axis-angle vector (numerically accurate for tiny angles, unlike arccos(trace))."""
    import cv2
    return cv2.Rodrigues(np.asarray(R, dtype=np.float64))[0].ravel()


def rigid_error(T_est, T_true) -> Tuple[float, float]:
    """(translation error mm, rotation error deg) between two 4x4 transforms."""
    dT = np.linalg.inv(T_true) @ T_est
    tr = float(np.linalg.norm(dT[:3, 3]) * 1000.0)
    rv = _rotvec(dT[:3, :3])
    return tr, float(np.degrees(np.linalg.norm(rv)))
