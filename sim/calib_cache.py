"""Calibration cache for the simulation (SIMULATION ONLY).

Saves the ESTIMATED calibration (K, dist, T_base_cam) plus its score numbers to
results/sim_calib/sim_calib.json, keyed by a hash of the camera + cube + workspace yaml files
and the calibration settings. Ground-truth numbers stored here are scores only.
"""
import hashlib
import json
import os
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATH = os.path.join(ROOT, "results", "sim_calib", "sim_calib.json")
LABEL = "SIMULATION ONLY - calibration of the simulated C270 / xArm 5, not real hardware"
_ARR_INTR = ("K", "dist", "K_true", "dist_true")
_ARR_EXT = ("T_base_cam_est", "T_world_cam_true")


def config_hash(cfg_paths, seed, board_poses, calib_poses, settings):
    """sha256 over camera/cube/workspace yaml bytes (CRLF-normalised) + seed + calibration settings."""
    h = hashlib.sha256()
    for k in ("camera", "cube", "workspace"):
        with open(cfg_paths[k], "rb") as f:
            h.update(k.encode() + f.read().replace(b"\r\n", b"\n"))
    keys = ("board_inner", "board_square_m", "board_depth_m", "board_lateral_m", "board_tilt_deg", "board_roll_deg",
            "calib_xy_m", "calib_z_m", "calib_tilt_deg", "frames_per_pose", "handeye_outlier_trans_mm", "park_q")
    h.update(json.dumps({"seed": seed, "board": board_poses, "poses": calib_poses,
                         "S": {k: settings[k] for k in keys}}, sort_keys=True, default=list).encode())
    return h.hexdigest()


def _ser(d, arr_keys):
    out = {}
    for k, v in d.items():
        if isinstance(v, np.ndarray):
            out[k] = v.tolist()
        elif isinstance(v, dict):
            out[k] = {a: float(b) for a, b in v.items()}
        elif isinstance(v, (np.floating, np.integer)):
            out[k] = v.item()
        else:
            out[k] = v
    return out


def save(intr, ext, close_ang, chash, path=PATH):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    d = {"label": LABEL, "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"), "epoch": time.time(), "hash": chash,
         "close_ang": float(close_ang), "intr": _ser(intr, _ARR_INTR), "ext": _ser(ext, _ARR_EXT)}
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(d, f, indent=1)
    os.replace(tmp, path)
    return d


def load(chash, path=PATH):
    """-> dict(intr, ext, K, dist, T_base_cam, close_ang, timestamp) or None (missing / stale / broken)."""
    try:
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f)
        if d.get("hash") != chash:
            return None
        intr, ext = d["intr"], d["ext"]
        for k in _ARR_INTR:
            intr[k] = np.array(intr[k], float)
        for k in _ARR_EXT:
            ext[k] = np.array(ext[k], float)
        return {"intr": intr, "ext": ext, "K": intr["K"], "dist": intr["dist"], "T_base_cam": ext["T_base_cam_est"],
                "close_ang": float(d["close_ang"]), "timestamp": d["timestamp"]}
    except Exception:
        return None
