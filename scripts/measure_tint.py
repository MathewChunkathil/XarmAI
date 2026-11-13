"""Does tinting the white quiet zone of the cube faces hurt AprilTag detection / pose?  SIMULATION ONLY.

    .venv\\Scripts\\python -I scripts\\measure_tint.py [--poses 20] [--out results/tint_measurement.md]

For every cube (red/green/blue) and every tint strength the SAME 20 random cube poses (and the same camera noise
stream) are rendered with the simulated C270, tags detected (pupil_apriltags), cube pose by PnP with the TRUE camera
intrinsics (isolates the texture effect from calibration error) and compared with the true pose (scoring only).
strength 0 = plain white quiet zone (reference). Camera noise / blur / JPEG are PLACEHOLDER (see c270_sim.yaml).
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (ROOT, os.path.join(ROOT, "src", "cube_perception")):
    if p not in sys.path:
        sys.path.insert(0, p)

from pupil_apriltags import Detector  # noqa: E402

from cube_perception import corners_from_pupil, cube_yaw, estimate_cubes, load_cube_set, to_base  # noqa: E402
from sim.c270_camera import C270SimCamera, load_cfg  # noqa: E402
from sim.pyb_world import SimWorld  # noqa: E402

STRENGTHS = (0.0, 0.2, 0.35, 0.5, 0.7)


def measure(world, cam, det, cube_set, number, strength, poses, seed):
    world.cubes_cfg["tint"]["margin_strength"] = strength
    cam.rng = np.random.default_rng(seed)
    K, dist, T_wc = cam.K_true, cam.dist_true, cam.T_world_cam_true
    ntag, valid, pe, ye, rms = [], 0, [], [], []
    for (x, y, yaw) in poses:
        world.spawn_cube(x, y, yaw, number=number)
        img = cam.render(world.p)
        dets = [(int(d.tag_id), corners_from_pupil(d)) for d in det.detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))]
        mine = [d for d in dets if d[0] in cube_set[number].ids]
        ntag.append(len(mine))
        r = estimate_cubes(dets, K, dist, cube_set).get(number)
        if r is not None and r.valid:
            valid += 1
            Tb = to_base(T_wc, r.T_cam_cube, world.table_top_z, world.edge)
            Tt = world.cube_pose_true()
            pe.append(1000 * math.hypot(Tb[0, 3] - Tt[0, 3], Tb[1, 3] - Tt[1, 3]))
            d = cube_yaw(Tb) - cube_yaw(Tt)
            ye.append(abs(math.degrees((d + math.pi / 4) % (math.pi / 2) - math.pi / 4)))
            rms.append(r.rms_px)
    n = len(poses)
    return {"tags_mean": float(np.mean(ntag)), "tags_min": int(min(ntag)), "valid": valid, "n": n,
            "pos_mean": float(np.mean(pe)) if pe else float("nan"), "pos_max": float(np.max(pe)) if pe else float("nan"),
            "yaw_mean": float(np.mean(ye)) if ye else float("nan"), "yaw_max": float(np.max(ye)) if ye else float("nan"),
            "rms_mean": float(np.mean(rms)) if rms else float("nan")}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--poses", type=int, default=20)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=os.path.join(ROOT, "results", "tint_measurement.md"))
    a = ap.parse_args()
    w = SimWorld(gui=False)
    cam = C270SimCamera(load_cfg(w.cfg_paths["camera"]), np.random.default_rng(a.seed))
    det = Detector(families="tag36h11", nthreads=1, quad_decimate=1.0)
    cube_set = load_cube_set()
    box = w.workspace["cube_workspace_box"]
    rng = np.random.default_rng(a.seed + 7)
    poses = [(rng.uniform(box["x_m"][0] + 0.03, box["x_m"][1] - 0.03), rng.uniform(box["y_m"][0] + 0.03, box["y_m"][1] - 0.03),
              rng.uniform(-math.pi, math.pi)) for _ in range(a.poses)]
    L = ["# Tint measurement (SIMULATION ONLY)\n",
         "%d random cube poses in the pick zone per cell, simulated C270 (noise/blur/JPEG = PLACEHOLDER), pose by PnP with the "
         "TRUE intrinsics (isolates the texture effect), error vs true pose (scoring only). Strength 0 = plain white quiet zone.\n" % a.poses,
         "| cube | strength | tags seen / pose (mean, min) | cube valid | pos err mm mean / max | yaw err deg mean / max | reproj rms px |",
         "|---|---|---|---|---|---|---|"]
    print("\n".join(L[-2:]))
    for number, m in cube_set.items():
        for s in STRENGTHS:
            r = measure(w, cam, det, cube_set, number, s, poses, a.seed + 100)
            row = "| %s | %.2f | %.2f, %d | %d / %d | %.2f / %.2f | %.2f / %.2f | %.3f |" % (
                m.name, s, r["tags_mean"], r["tags_min"], r["valid"], r["n"], r["pos_mean"], r["pos_max"],
                r["yaw_mean"], r["yaw_max"], r["rms_mean"])
            L.append(row)
            print(row, flush=True)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")
    w.close()


if __name__ == "__main__":
    main()
