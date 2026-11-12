"""Render one frame, save results/sim_demo/preview_cam.png, print detected tag IDs. SIMULATION ONLY."""
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pupil_apriltags import Detector  # noqa: E402
from sim.c270_camera import C270SimCamera, load_cfg  # noqa: E402
from sim.pyb_world import ROOT, SimWorld  # noqa: E402

w = SimWorld(gui="--gui" in sys.argv)
cam = C270SimCamera(load_cfg(w.cfg_paths["camera"]), np.random.default_rng(0))
w.spawn_cube(0.38, 0.02, np.radians(25))
t0 = time.time()
img = cam.render(w.p, gui=w.gui)
dt = time.time() - t0
dets = Detector(families="tag36h11").detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
out = img.copy()
for d in dets:
    cv2.polylines(out, [d.corners.astype(np.int32).reshape(-1, 1, 2)], True, (0, 255, 0), 2)
    cv2.putText(out, str(d.tag_id), tuple(d.corners[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
os.makedirs(os.path.join(ROOT, "results", "sim_demo"), exist_ok=True)
cv2.imwrite(os.path.join(ROOT, "results", "sim_demo", "preview_cam.png"), out)
print("SIMULATION ONLY | detected IDs:", sorted(d.tag_id for d in dets), "| render+effects %.2fs" % dt)
