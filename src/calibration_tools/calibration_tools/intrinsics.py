"""Checkerboard intrinsic calibration."""
from typing import Optional, Sequence

import cv2
import numpy as np

DEFAULT_CFG = {
    "subpix_win": 11,        # cornerSubPix half-window (px)
    "subpix_max_iter": 30,
    "subpix_eps": 0.001,
    "min_views": 3,
}


def calibrate_checkerboard(images: Sequence[np.ndarray], pattern=(9, 6), square_m=0.025,
                           cfg: Optional[dict] = None):
    """images: BGR or gray arrays. pattern = inner corners (cols, rows). Returns K, dist, rms."""
    c = dict(DEFAULT_CFG)
    c.update(cfg or {})
    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * square_m
    objs, imgs, size = [], [], None
    crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
            int(c["subpix_max_iter"]), float(c["subpix_eps"]))
    for im in images:
        g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY) if im.ndim == 3 else im
        size = g.shape[::-1]
        ok, corners = cv2.findChessboardCorners(g, pattern)
        if not ok:
            continue
        w = (int(c["subpix_win"]),) * 2
        corners = cv2.cornerSubPix(g, corners, w, (-1, -1), crit)
        objs.append(objp)
        imgs.append(corners)
    if len(objs) < int(c["min_views"]):
        raise ValueError(f"only {len(objs)} usable views (need {c['min_views']})")
    rms, K, dist, _, _ = cv2.calibrateCamera(objs, imgs, size, None, None)
    return K, dist, float(rms)
