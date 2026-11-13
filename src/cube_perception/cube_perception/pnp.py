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
    # --- single-tag quality gate (a lone tag has a depth / tilt ambiguity; measured outliers up to 46 mm / 30 deg).
    # Thresholds from scripts/measure_edge_poses.py (simulated C270, PLACEHOLDER noise): good single-tag solutions have
    # rms <= 0.58 px, side >= 58 px, border >= 3.6 px, tilt <= 0.9 deg, |z error| <= 3.9 mm.
    "single_max_rms_px": 0.7,
    "single_min_side_px": 40.0,       # mean tag side length in the image
    "single_border_px": 3.0,          # tag corner closer than this to the image border -> reject
    "single_max_tilt_deg": 6.0,       # resting-cube constraint: most vertical cube axis vs world z
    "single_max_z_err_m": 0.006,      # resting-cube constraint: centre z vs table / stack level (needs T_base_cam)
    # optional context, passed by the caller (the checks that need it are skipped when absent):
    "image_size": None,               # (width, height) px
    "T_base_cam": None, "table_z": None, "edge": None, "max_level": 0,
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
    if res.valid and len(ids) == 1:
        why = single_tag_gate(res, img, c)
        if why:
            res.valid, res.invalid_reason = False, why
    return res


def single_tag_gate(res: CubePoseResult, img_pts, c) -> str:
    """Extra checks for a solution from ONE tag. -> "" if it passes, else the invalid_reason."""
    if res.rms_px > float(c["single_max_rms_px"]):
        return "SINGLE_TAG_RMS"
    pts = np.asarray(img_pts, float).reshape(4, 2)
    side = float(np.mean([np.linalg.norm(pts[i] - pts[(i + 1) % 4]) for i in range(4)]))
    if side < float(c["single_min_side_px"]):
        return "SINGLE_TAG_SMALL"
    if c.get("image_size") is not None:
        w, h = c["image_size"]
        m = float(c["single_border_px"])
        if pts[:, 0].min() < m or pts[:, 1].min() < m or pts[:, 0].max() > w - m or pts[:, 1].max() > h - m:
            return "SINGLE_TAG_BORDER"
    if c.get("T_base_cam") is not None:
        T = np.asarray(c["T_base_cam"], float) @ res.T_cam_cube
        tilt = float(np.degrees(np.arccos(min(1.0, float(np.max(np.abs(T[2, :3])))))))
        if tilt > float(c["single_max_tilt_deg"]):
            return "SINGLE_TAG_NOT_RESTING"
        if c.get("table_z") is not None and c.get("edge") is not None:
            e, z0 = float(c["edge"]), float(c["table_z"]) + float(c["edge"]) / 2.0
            n = float(np.clip(round((T[2, 3] - z0) / e), 0, int(c.get("max_level") or 0)))
            if abs(T[2, 3] - (z0 + n * e)) > float(c["single_max_z_err_m"]):
                return "SINGLE_TAG_NOT_RESTING"
    return ""


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
            edge: Optional[float] = None, max_level: int = 0) -> np.ndarray:
    """T_base_cube = T_base_cam @ T_cam_cube, optionally constrained to a table plane.

    With table_z and edge given: z = table_z + edge/2; roll/pitch snapped so the
    nearest cube face is down (cube upright); yaw recomputed from the measured
    orientation and folded to [-pi/4, pi/4).
    max_level > 0 (stacks): z is snapped to the nearest stack level table_z + edge/2 + n*edge, n = 0..max_level
    (a cube can only rest on the table or on another cube). max_level = 0 is the legacy table-only behaviour.
    """
    T = np.asarray(T_base_cam) @ np.asarray(T_cam_cube)
    if table_z is None or edge is None:
        return T
    out = np.eye(4)
    out[:3, :3] = _rz(cube_yaw(T))
    z = table_z + edge / 2.0
    if max_level > 0:
        z += edge * float(np.clip(round((T[2, 3] - z) / edge), 0, max_level))
    out[:3, 3] = [T[0, 3], T[1, 3], z]
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


def estimate_cubes(detections, K, dist, cube_set, cfg: Optional[dict] = None):
    """All identified cubes at once: group tags by cube ID set, run estimate_cube_pose per cube.
    -> {cube number: CubePoseResult} for every cube with at least one detected tag."""
    from .cube_model import identify_cubes
    groups = identify_cubes(detections, cube_set)
    return {n: estimate_cube_pose(d, K, dist, cube_set[n], cfg) for n, d in groups.items()}
