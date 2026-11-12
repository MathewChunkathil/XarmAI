"""DEMO-5: live C270 AprilTag detection. UNCALIBRATED default intrinsics (fx~=fy~=800 @640x480, PLACEHOLDER).
Keys: q quit, s save screenshot to results/tag_detect.png."""
import sys, cv2, numpy as np
from pupil_apriltags import Detector
TAG = float(sys.argv[2]) if len(sys.argv) > 2 else 0.040   # m, black edge (measure on phone with ruler)
cam = int(sys.argv[1]) if len(sys.argv) > 1 else 0
cap = cv2.VideoCapture(cam, cv2.CAP_DSHOW)
cap.set(3, 640); cap.set(4, 480)
w, h = int(cap.get(3)), int(cap.get(4))
fx = fy = 800.0 * w / 640; cx, cy = w/2, h/2    # PLACEHOLDER: unmeasured
det = Detector(families="tag36h11")
while True:
    ok, f = cap.read()
    if not ok: print("no frame"); break
    g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
    for t in det.detect(g, estimate_tag_pose=True, camera_params=(fx, fy, cx, cy), tag_size=TAG):
        pts = t.corners.astype(int)
        cv2.polylines(f, [pts], True, (0, 255, 0), 2)
        x, y, z = t.pose_t.ravel()
        cv2.putText(f, f"id{t.tag_id} xyz=({x:.2f},{y:.2f},{z:.2f})m", tuple(pts[0]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    cv2.putText(f, "CAMERA UNCALIBRATED", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.imshow("tags", f); k = cv2.waitKey(1) & 0xFF
    if k == ord("s"): cv2.imwrite("results/tag_detect.png", f)
    if k == ord("q"): break
