"""Eye-to-hand calibration (extrinsics method A), no ground truth.

Camera fixed, tag on gripper. Per pose i:  T_base_tcp_i @ X = Y @ T_cam_tag_i
with X = T_tcp_tag (unknown) and Y = T_base_cam (unknown).
Solved jointly: linear rotation (Kronecker/null-space), then linear translation LS.
(cv2.calibrateRobotWorldHandEye is absent from the installed OpenCV 5 wheel.)
"""
from typing import Sequence

import numpy as np


def _project_so3(M):
    U, _, Vt = np.linalg.svd(M)
    return U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt


def calibrate_eye_to_hand(T_base_tcp: Sequence[np.ndarray], T_cam_tag: Sequence[np.ndarray]):
    """Return dict: T_base_cam, T_tcp_tag, rot_resid_deg (mean), trans_resid_mm (mean)."""
    A = [np.asarray(T, dtype=np.float64) for T in T_base_tcp]
    B = [np.asarray(T, dtype=np.float64) for T in T_cam_tag]
    n = len(A)
    if n < 3 or n != len(B):
        raise ValueError("need >=3 matched pose pairs")
    I3 = np.eye(3)
    rows = []
    for a, b in zip(A, B):
        Ra, Rb = a[:3, :3], b[:3, :3]
        # column-major vec: vec(Ra X) = (I (x) Ra) vec X ; vec(Y Rb) = (Rb^T (x) I) vec Y
        rows.append(np.hstack([np.kron(I3, Ra), -np.kron(Rb.T, I3)]))
    M = np.vstack(rows)
    v = np.linalg.svd(M)[2][-1]
    if np.linalg.det(v[:9].reshape(3, 3, order="F")) < 0:
        v = -v
    Rx = _project_so3(v[:9].reshape(3, 3, order="F"))
    Ry = _project_so3(v[9:].reshape(3, 3, order="F"))
    # Translation: Ra tx + ta = Ry tb + ty  ->  [Ra, -I][tx;ty] = Ry tb - ta
    L = np.vstack([np.hstack([a[:3, :3], -I3]) for a in A])
    r = np.concatenate([Ry @ b[:3, 3] - a[:3, 3] for a, b in zip(A, B)])
    sol = np.linalg.lstsq(L, r, rcond=None)[0]
    X, Y = np.eye(4), np.eye(4)
    X[:3, :3], X[:3, 3] = Rx, sol[:3]
    Y[:3, :3], Y[:3, 3] = Ry, sol[3:]
    rot, tr = [], []
    for a, b in zip(A, B):
        d = np.linalg.inv(a @ X) @ (Y @ b)
        tr.append(np.linalg.norm(d[:3, 3]) * 1000.0)
        rot.append(np.degrees(np.arccos(np.clip((np.trace(d[:3, :3]) - 1) / 2, -1, 1))))
    return {"T_base_cam": Y, "T_tcp_tag": X,
            "rot_resid_deg": float(np.mean(rot)), "trans_resid_mm": float(np.mean(tr))}
