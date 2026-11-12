"""Multi-tag cube pose from AprilTag corners via PnP.

Inputs : detections [(tag_id, corners 4x2 in OpenCV order TL,TR,BR,BL)], K, dist, CubeModel.
Outputs: CubePoseResult (T_cam_cube 4x4, tag_ids_used, rms px, valid, invalid_reason).
"""
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

# Documented defaults; override by passing a cfg dict (e.g. loaded from yaml).
DEFAULT_CFG = {
    "max_reproj_rms_px": 2.0,   # reject if RMS reprojection error above this
    "min_tags": 1,              # minimum model tags required
    "lm_max_iter": 20,          # solvePnPRefineLM iterations
    "lm_eps": 1e-8,
    "min_depth_m": 1e-3,        # cube centre must be at least this far in front of camera
}


@dataclass
class CubePoseResult:
    T_cam_cube: np.ndarray = field(default_factory=lambda: np.eye(4))
    tag_ids_used: List[int] = field(default_factory=list)
    rms_px: float = float("nan")
    valid: bool = False
    invalid_reason: str = "NO_TAGS"


def corners_from_pupil(det) -> np.ndarray:
    """pupil_apriltags Detection -> 4x2 corners in OpenCV order (TL,TR,BR,BL).

    Verified empirically (synthetic homography test): pupil_apriltags returns
    [TR, TL, BL, BR] in image coords for an upright tag, so OpenCV order is
    pupil[[1,0,3,2]].  Accepts a Detection or a raw 4x2 array.
    """
    c = np.asarray(getattr(det, "corners", det), dtype=np.float64).reshape(4, 2)
    return c[[1, 0, 3, 2]].copy()


def _rt_to_T(rvec, tvec):
    T = np.eye(4)
    T[:3, :3], _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).ravel()
    return T


def estimate_cube_pose(detections: Sequence[Tuple[int, np.ndarray]], K, dist, model,
                       cfg: Optional[dict] = None) -> CubePoseResult:
    c = dict(DEFAULT_CFG)
    c.update(cfg or {})
    res = CubePoseResult()
    if not detections:
        res.invalid_reason = "NO_TAGS"
        return res
    ids, img_pts, obj_pts = [], [], []
    for tid, corners in detections:
        tid = int(tid)
        if tid not in model.corners_cube:
            continue
        if tid in ids:
            res.invalid_reason = "INCONSISTENT"   # duplicate id
            return res
        ids.append(tid)
        img_pts.append(np.asarray(corners, dtype=np.float64).reshape(4, 2))
        obj_pts.append(model.corners_cube[tid])
    if not ids:
        res.invalid_reason = "UNKNOWN_ID"
        return res
    if len(ids) < int(c["min_tags"]):
        res.invalid_reason = "NO_TAGS"
        return res
    obj = np.vstack(obj_pts).astype(np.float64)
    img = np.vstack(img_pts).astype(np.float64)
    K = np.asarray(K, dtype=np.float64)
    dist = np.zeros(5) if dist is None else np.asarray(dist, dtype=np.float64)
    try:
        ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_SQPNP)
        if not ok:
            res.invalid_reason = "INCONSISTENT"
            return res
        rvec, tvec = cv2.solvePnPRefineLM(
            obj, img, K, dist, rvec, tvec,
            (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, int(c["lm_max_iter"]), float(c["lm_eps"])))
    except cv2.error:
        res.invalid_reason = "INCONSISTENT"
        return res
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
    res.T_cam_cube = _rt_to_T(rvec, tvec)
    res.tag_ids_used = ids
    res.rms_px = rms
    if res.T_cam_cube[2, 3] < float(c["min_depth_m"]):
        res.invalid_reason = "INCONSISTENT"
    elif not np.isfinite(rms) or rms > float(c["max_reproj_rms_px"]):
        res.invalid_reason = "HIGH_REPROJ"
    else:
        res.valid, res.invalid_reason = True, ""
    return res


def fold_yaw(yaw: float) -> float:
    """Fold yaw (rad) into [-pi/4, pi/4) (cube has 90 deg symmetry about z)."""
    q = np.pi / 2.0
    return float((yaw + q / 2.0) % q - q / 2.0)


def _rz(a):
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[ca, -sa, 0], [sa, ca, 0], [0, 0, 1.0]])


def cube_yaw(T_base_cube: np.ndarray) -> float:
    """Folded yaw of a (near-upright) cube: heading of a horizontal cube axis, mod 90 deg."""
    R = T_base_cube[:3, :3]
    j = int(np.argmax(np.abs(R[2, :])))          # axis most aligned with vertical
    k = (j + 1) % 3
    return fold_yaw(float(np.arctan2(R[1, k], R[0, k])))


def to_base(T_base_cam, T_cam_cube, table_z: Optional[float] = None,
            edge: Optional[float] = None) -> np.ndarray:
    """T_base_cube = T_base_cam @ T_cam_cube, optionally constrained to a table plane.

    With table_z and edge given: z = table_z + edge/2; roll/pitch snapped so the
    nearest cube face is down (cube upright); yaw recomputed from the measured
    orientation and folded to [-pi/4, pi/4).
    """
    T = np.asarray(T_base_cam) @ np.asarray(T_cam_cube)
    if table_z is None or edge is None:
        return T
    out = np.eye(4)
    out[:3, :3] = _rz(cube_yaw(T))
    out[:3, 3] = [T[0, 3], T[1, 3], table_z + edge / 2.0]
    return out


def median_pose(Ts: Sequence[np.ndarray]) -> np.ndarray:
    """Multi-frame robust average: component-wise median translation; rotation =
    SVD projection of the element-wise median rotation matrix."""
    Ts = [np.asarray(T, dtype=np.float64) for T in Ts]
    t = np.median(np.stack([T[:3, 3] for T in Ts]), axis=0)
    Rm = np.median(np.stack([T[:3, :3] for T in Ts]), axis=0)
    U, _, Vt = np.linalg.svd(Rm)
    R = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = R, t
    return out
