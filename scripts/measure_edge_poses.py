"""Single-tag outlier check: poses spread over the whole camera view INCLUDING the edges, gate OFF vs ON.  SIMULATION ONLY.

    .venv\\Scripts\\python -I scripts\\measure_edge_poses.py [--poses 120] [--out results/edge_pose_gate.md]

Same renders (simulated C270, PLACEHOLDER noise) are solved twice with the TRUE intrinsics / extrinsics (isolates the
perception gate from calibration error): "off" = single-tag quality gate disabled (old behaviour), "on" = gate enabled
(pnp.DEFAULT_CFG single_*). Outlier = position error > 5 mm or yaw error > 5 deg (vs the true pose, scoring only).
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

OFF = {"single_max_rms_px": 1e9, "single_min_side_px": 0.0, "single_border_px": -1.0, "single_max_tilt_deg": 1e9,
       "single_max_z_err_m": 1e9}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--poses", type=int, default=120, help="poses per cube")
    ap.add_argument("--seed", type=int, default=5)
    ap.add_argument("--out", default=os.path.join(ROOT, "results", "edge_pose_gate.md"))
    a = ap.parse_args()
    w = SimWorld(gui=False)
    cam = C270SimCamera(load_cfg(w.cfg_paths["camera"]), np.random.default_rng(a.seed))
    det = Detector(families="tag36h11", nthreads=1, quad_decimate=1.0)
    cs = load_cube_set()
    box = w.workspace["cube_workspace_box"]
    rng = np.random.default_rng(a.seed)
    K, dist, T_wc = cam.K_true, cam.dist_true, cam.T_world_cam_true
    ctx = {"image_size": (cam.W, cam.H), "T_base_cam": T_wc, "table_z": w.table_top_z, "edge": w.edge, "max_level": 0}
    stat = {"off": [], "on": []}                      # (n_tags, valid, pos_err_mm, yaw_err_deg)
    reasons = {}
    for number in cs:
        for _ in range(a.poses):
            x = rng.uniform(box["x_m"][0] - 0.05, box["x_m"][1] + 0.05)       # beyond the pick zone -> view edge
            y = rng.uniform(box["y_m"][0] - 0.07, box["y_m"][1] + 0.07)
            w.spawn_cube(x, y, rng.uniform(-math.pi, math.pi), number=number)
            img = cam.render(w.p)
            dets = [(int(d.tag_id), corners_from_pupil(d)) for d in det.detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))]
            Tt = w.cube_pose_true()
            for key, cfg in (("off", dict(ctx, **OFF)), ("on", ctx)):
                r = estimate_cubes(dets, K, dist, cs, cfg).get(number)
                if r is None:
                    continue
                if not r.valid:
                    reasons[r.invalid_reason] = reasons.get(r.invalid_reason, 0) + 1 if key == "on" else reasons.get(
                        r.invalid_reason, 0)
                    stat[key].append((len(r.tag_ids_used), False, float("nan"), float("nan")))
                    continue
                Tb = to_base(T_wc, r.T_cam_cube, w.table_top_z, w.edge)
                d = cube_yaw(Tb) - cube_yaw(Tt)
                stat[key].append((len(r.tag_ids_used), True, 1000 * math.hypot(Tb[0, 3] - Tt[0, 3], Tb[1, 3] - Tt[1, 3]),
                                  abs(math.degrees((d + math.pi / 4) % (math.pi / 2) - math.pi / 4))))
    L = ["# Single-tag outlier gate (SIMULATION ONLY)\n",
         "%d poses per cube (3 cubes) spread over and beyond the pick zone (view edges included), simulated C270 (noise PLACEHOLDER), true "
         "intrinsics / extrinsics. Outlier = pos error > 5 mm or yaw error > 5 deg vs true pose (scoring only).\n" % a.poses,
         "| gate | valid poses | 1-tag valid | outliers (all) | outliers (1-tag) | pos err mm mean / max (valid) |", "|---|---|---|---|---|---|"]
    for key in ("off", "on"):
        v = [r for r in stat[key] if r[1]]
        one = [r for r in v if r[0] == 1]
        out = [r for r in v if r[2] > 5 or r[3] > 5]
        out1 = [r for r in out if r[0] == 1]
        L.append("| %s | %d | %d | %d | %d | %.2f / %.2f |" % (key, len(v), len(one), len(out), len(out1),
                                                           np.mean([r[2] for r in v]), np.max([r[2] for r in v])))
    L.append("\nrejected by the gate (reason: count over all frames of the 'on' pass): %s\n" % reasons)
    print("\n".join(L))
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")
    w.close()


if __name__ == "__main__":
    main()
