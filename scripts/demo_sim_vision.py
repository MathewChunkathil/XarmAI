r"""SIM vision + auto extrinsic calibration + vision-driven pick-place (PyBullet).
SIMULATION ONLY | camera = rendered pinhole (ideal intrinsics, pixel noise) | grasp = kinematic attach
| robot = xArm6 URDF stand-in | gripper = xArm Gripper stand-in.
Pipeline: (1) mount alignment  (2) calib A: tag-on-gripper  (3) calib B: touch-point
          (4) spawn cube (pose unknown to pipeline)  (5) detect cube tag -> base frame -> pick-place.
Ground-truth camera pose is used ONLY to score errors (and for the one-off tag-mount alignment).
Run: .venv\Scripts\python -I scripts\demo_sim_vision.py [--gui]
"""
import math, os, sys, time
import numpy as np, cv2
import pybullet as p, pybullet_data
from pupil_apriltags import Detector

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
rng = np.random.default_rng(0)
GUI = "--gui" in sys.argv
W, H, FOV = 1280, 960, 60.0                      # sim camera (PLACEHOLDER spec, not C270)
fy = (H / 2) / math.tan(math.radians(FOV) / 2); fx = fy; cx, cy = W / 2, H / 2
TAG, PLATE, TH, CUBE = 0.040, 0.050, 0.002, 0.050   # tag black edge, plate edge, plate thick, cube edge (m)
PIX_NOISE, TOUCH_NOISE = 2.0, 0.001              # gray levels, metres
TCP_OFF, EE, ARM = 0.15, 7, [1, 2, 3, 4, 5, 6]   # TCP_OFF PLACEHOLDER (m)
HOME = [0, -0.5, -0.8, 0, 1.3, 0]
APPROACH, POS_TOL = 0.12, 0.005
PLACE_XY = (0.30, -0.20)
D = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
det = Detector(families="tag36h11")

# ---------- math ----------
def T(pos, orn):
    M = np.eye(4); M[:3, :3] = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3); M[:3, 3] = pos; return M
def Tinv(M):
    R, t = M[:3, :3], M[:3, 3]; X = np.eye(4); X[:3, :3] = R.T; X[:3, 3] = -R.T @ t; return X
def quat(R):
    t = np.trace(R)
    if t > 0:
        s = 2 * math.sqrt(t + 1); return [(R[2,1]-R[1,2])/s, (R[0,2]-R[2,0])/s, (R[1,0]-R[0,1])/s, s/4]
    i = int(np.argmax(np.diag(R))); j, k = (i+1) % 3, (i+2) % 3
    s = 2 * math.sqrt(1 + R[i,i] - R[j,j] - R[k,k]); q = [0]*4
    q[i] = s/4; q[j] = (R[j,i]+R[i,j])/s; q[k] = (R[k,i]+R[i,k])/s; q[3] = (R[k,j]-R[j,k])/s; return q
def set_pose(body, M): p.resetBasePositionAndOrientation(body, M[:3, 3].tolist(), quat(M[:3, :3]))
def rot_err_deg(Ra, Rb): return math.degrees(math.acos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1, 1)))
def avg_rot(Rs):
    U, _, Vt = np.linalg.svd(sum(Rs)); R = U @ Vt
    if np.linalg.det(R) < 0: U[:, -1] *= -1; R = U @ Vt
    return R
def kabsch(P, Q):                                # Q = R P + t
    pm, qm = P.mean(0), Q.mean(0); U, _, Vt = np.linalg.svd((P - pm).T @ (Q - qm))
    R = Vt.T @ np.diag([1, 1, np.linalg.det(Vt.T @ U.T)]) @ U.T; return R, qm - R @ pm

# ---------- world ----------
p.connect(p.GUI if GUI else p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath()); p.setGravity(0, 0, -9.81)
plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("xarm/xarm6_with_gripper.urdf", useFixedBase=True)
lo = [p.getJointInfo(robot, j)[8] for j in ARM]; hi = [p.getJointInfo(robot, j)[9] for j in ARM]
rngs = [h - l for l, h in zip(lo, hi)]
for j, q in zip(ARM, HOME): p.resetJointState(robot, j, q)

def tex(i):
    d = os.path.join(ROOT, "results", "tags", "sim"); os.makedirs(d, exist_ok=True)
    im = np.full((1000, 1000), 255, np.uint8); im[100:900, 100:900] = cv2.aruco.generateImageMarker(D, i, 800)
    f = os.path.join(d, f"tag_{i}.png"); cv2.imwrite(f, cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)); return p.loadTexture(f)
def obj_file():
    """Unit cube OBJ, every face has full 0..1 UV (pybullet box UVs are not usable for tags)."""
    f = os.path.join(ROOT, "results", "tags", "sim", "unit_cube.obj"); os.makedirs(os.path.dirname(f), exist_ok=True)
    faces = [[(-.5,-.5,.5),(.5,-.5,.5),(.5,.5,.5),(-.5,.5,.5)], [(-.5,.5,-.5),(.5,.5,-.5),(.5,-.5,-.5),(-.5,-.5,-.5)],
             [(.5,-.5,-.5),(.5,.5,-.5),(.5,.5,.5),(.5,-.5,.5)], [(-.5,.5,-.5),(-.5,-.5,-.5),(-.5,-.5,.5),(-.5,.5,.5)],
             [(.5,.5,-.5),(-.5,.5,-.5),(-.5,.5,.5),(.5,.5,.5)], [(-.5,-.5,-.5),(.5,-.5,-.5),(.5,-.5,.5),(-.5,-.5,.5)]]
    L = ["vt 0 0", "vt 1 0", "vt 1 1", "vt 0 1"]
    for fc in faces: L += [f"v {a} {b} {c}" for a, b, c in fc]
    for i in range(6): b = 4 * i; L.append("f " + " ".join(f"{b + k + 1}/{k + 1}" for k in range(4)))
    open(f, "w").write(chr(10).join(L) + chr(10)); return f
def box(half, tid, mass=0.0, collide=False, xyz=(5, 5, 5), orn=(0, 0, 0, 1)):
    vs = p.createVisualShape(p.GEOM_MESH, fileName=obj_file(), meshScale=[2 * h for h in half], rgbaColor=[1, 1, 1, 1])
    cs = p.createCollisionShape(p.GEOM_BOX, halfExtents=half) if collide else -1
    b = p.createMultiBody(mass, cs, vs, xyz, orn); p.changeVisualShape(b, -1, textureUniqueId=tid); return b

# camera (ground truth used only for scoring)
EYE, TGT = [0.65, 0.0, 0.60], [0.35, 0.0, 0.05]
view = p.computeViewMatrix(EYE, TGT, [0, 0, 1]); proj = p.computeProjectionMatrixFOV(FOV, W / H, 0.05, 3.0)
T_base_cam_gt = Tinv(np.array(view).reshape(4, 4, order="F")) @ np.diag([1, -1, -1, 1])

def render():
    img = p.getCameraImage(W, H, view, proj, renderer=p.ER_TINY_RENDERER, shadow=0)[2]
    rgb = np.array(img, np.uint8).reshape(H, W, 4)[:, :, :3]
    g = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY).astype(float) + rng.normal(0, PIX_NOISE, (H, W))
    return rgb, np.clip(g, 0, 255).astype(np.uint8)
def detect_all(g): return det.detect(g, estimate_tag_pose=True, camera_params=(fx, fy, cx, cy), tag_size=TAG)
def detect(g): return {t.tag_id: t for t in detect_all(g)}
def T_cam_tag(t): X = np.eye(4); X[:3, :3] = t.pose_R; X[:3, 3] = t.pose_t.ravel(); return X

# ---------- robot ----------
def flange_target(xyz, yaw):
    q = p.getQuaternionFromEuler([math.pi, 0, yaw]); R = np.array(p.getMatrixFromQuaternion(q)).reshape(3, 3)
    return list(np.array(xyz) - R @ np.array([0, 0, TCP_OFF])), q
def ik(xyz, yaw, seed):
    for j, q in zip(ARM, seed): p.resetJointState(robot, j, q)
    pos, q = flange_target(xyz, yaw)
    sol = p.calculateInverseKinematics(robot, EE, pos, q, lowerLimits=lo, upperLimits=hi, jointRanges=rngs,
                                       restPoses=seed, maxNumIterations=300, residualThreshold=1e-5)[:6]
    for j, v in zip(ARM, sol): p.resetJointState(robot, j, v)
    err = np.linalg.norm(np.array(p.getLinkState(robot, EE, computeForwardKinematics=1)[4]) - np.array(pos))
    lim = all(l - 1e-3 <= v <= h + 1e-3 for v, l, h in zip(sol, lo, hi))
    floor = not [c for c in p.getClosestPoints(robot, plane, 0.0) if c[3] >= 2]
    return list(sol), err, lim, floor
def ee_T(): s = p.getLinkState(robot, EE, computeForwardKinematics=1); return T(s[4], s[5])
def tcp_pos(): M = ee_T(); return M[:3, 3] + M[:3, :3] @ np.array([0, 0, TCP_OFF])
def go_home():
    for j, q in zip(ARM, HOME): p.resetJointState(robot, j, q)
def set_grip(v):
    for j in (8, 9, 10, 11, 12, 13): p.resetJointState(robot, j, v)
def move(a_, b_, steps=40):
    for t in range(1, steps + 1):
        for j, f, g in zip(ARM, a_, b_): p.resetJointState(robot, j, f + t / steps * (g - f))
        p.stepSimulation()
        if GUI: time.sleep(1 / 120)

def report(name, R, t):
    te = np.linalg.norm(t - T_base_cam_gt[:3, 3]) * 1000; re = rot_err_deg(T_base_cam_gt[:3, :3], R)
    print(f"  {name:<22} trans err {te:7.2f} mm | rot err {re:6.3f} deg"); return te, re

print("SIMULATION ONLY | rendered camera | grasp=kinematic attach | robot=xArm6 stand-in | gripper=xArm Gripper stand-in")
# ---------- (1) mount alignment ----------
T_ee_plate = T([0.10, 0, 0.03], [1, 0, 0, 0])      # plate sits beside gripper, normal = -z_ee (faces up)
grip_plate = box([PLATE / 2, PLATE / 2, TH / 2], tex(20))
res = []
for k in range(10):
    Tp = T([0.30 + 0.02 * (k % 5), -0.10 + 0.08 * (k // 5), TH / 2], p.getQuaternionFromEuler([0, 0, math.radians(15 * k - 60)]))
    set_pose(grip_plate, Tp)
    t = detect(render()[1]).get(20)
    if t is None: continue
    res.append(Tinv(Tp) @ T_base_cam_gt @ T_cam_tag(t))
if len(res) < 5: sys.exit("alignment: too few detections")
T_plate_tag = np.eye(4); T_plate_tag[:3, :3] = avg_rot([m[:3, :3] for m in res]); T_plate_tag[:3, 3] = np.mean([m[:3, 3] for m in res], 0)
print(f"[1] mount alignment: {len(res)} views, spread {np.std([m[:3, 3] for m in res], 0).max() * 1000:.2f} mm "
      f"(stands for measured tag mount)")
T_ee_tag = T_ee_plate @ T_plate_tag

# ---------- (2) calib A: tag-on-gripper ----------
print("[2] calib A: tag-on-gripper")
Ts, used = [], 0
for n in range(40):
    if used >= 14: break
    x, y, z, yaw = rng.uniform(0.28, 0.45), rng.uniform(-0.15, 0.15), rng.uniform(0.08, 0.20), rng.uniform(-0.6, 0.6)
    q, e, lim, _ = ik((x, y, z), yaw, HOME)
    if e > POS_TOL or not lim: continue
    Tee = ee_T(); set_pose(grip_plate, Tee @ T_ee_plate)
    t = detect(render()[1]).get(20)
    if t is None: continue
    Ts.append(Tee @ T_ee_tag @ Tinv(T_cam_tag(t))); used += 1
go_home(); set_pose(grip_plate, T([5, 5, 5], [0, 0, 0, 1]))
if used < 4: sys.exit("calib A: too few views")
RA, tA = avg_rot([m[:3, :3] for m in Ts]), np.mean([m[:3, 3] for m in Ts], 0)
spread = np.std([m[:3, 3] for m in Ts], 0) * 1000
print(f"  views used {used}, per-view spread (mm) x/y/z {spread[0]:.1f}/{spread[1]:.1f}/{spread[2]:.1f}")
teA, reA = report("A tag-on-gripper", RA, tA)

# ---------- (3) calib B: touch-point ----------
print("[3] calib B: touch-point (6 floor tags, TCP touches each centre)")
PTS = {11: (0.26, 0.22), 12: (0.46, 0.22), 13: (0.26, 0.0), 14: (0.46, 0.0), 15: (0.46, -0.10), 16: (0.26, -0.10)}
for i, (x, y) in PTS.items(): box([PLATE / 2, PLATE / 2, TH / 2], tex(i), xyz=(x, y, TH / 2))
go_home(); dets = detect(render()[1])
Pc, Qb = [], []
for i, (x, y) in PTS.items():
    if i not in dets: continue
    q, e, lim, _ = ik((x, y, TH), 0.0, HOME)
    if e > POS_TOL: continue
    Pc.append(dets[i].pose_t.ravel()); Qb.append(tcp_pos() + rng.normal(0, TOUCH_NOISE, 3))
go_home()
RB, tB = kabsch(np.array(Pc), np.array(Qb))
print(f"  points used {len(Pc)}")
teB, reB = report("B touch-point", RB, tB)
T_base_cam = np.eye(4); T_base_cam[:3, :3] = RA; T_base_cam[:3, 3] = tA   # active = A
print("  active extrinsics = A (tag-on-gripper)")

# ---------- (4) spawn cube ----------
cube_xy = (0.36, 0.08); cube_yaw = math.radians(25)
cube = box([CUBE / 2] * 3, tex(0), mass=0.05, collide=True, xyz=(*cube_xy, CUBE / 2),
           orn=p.getQuaternionFromEuler([0, 0, cube_yaw]))
p.changeDynamics(cube, -1, lateralFriction=1.0)
for _ in range(120): p.stepSimulation()
gt_pos, gt_orn = p.getBasePositionAndOrientation(cube)

# ---------- (5) detect cube ----------
rgb, g = render(); tags = [t for t in detect_all(g) if t.tag_id == 0]
vis = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
best = None
for t in tags:
    cv2.polylines(vis, [t.corners.astype(int)], True, (0, 255, 0), 2)
    Tb = T_base_cam @ T_cam_tag(t); up = abs(Tb[2, 2])
    if best is None or up > best[0]: best = (up, Tb)
if best is None: sys.exit("cube tag not detected")
Tb = best[1]; ex, ey = Tb[0, 3], Tb[1, 3]; yaw_est = math.atan2(Tb[1, 0], Tb[0, 0])
dyaw = math.degrees(((yaw_est - cube_yaw + math.pi / 4) % (math.pi / 2)) - math.pi / 4)
cv2.putText(vis, "SIM camera | detected cube tag", (10, 30), 0, 0.9, (0, 255, 0), 2)
cv2.imwrite(os.path.join(ROOT, "results", "sim_camera.png"), vis)
print(f"[5] cube pose est xy=({ex:.3f},{ey:.3f}) vs gt ({gt_pos[0]:.3f},{gt_pos[1]:.3f}) "
      f"-> err {math.hypot(ex - gt_pos[0], ey - gt_pos[1]) * 1000:.1f} mm, yaw err {dyaw:.2f} deg (mod 90)")

# ---------- pick-place from ESTIMATED pose ----------
print(f"{'yaw_deg':>8} {'ik_err_mm':>10} {'cost':>7}  verdict")
cands = []
for k in range(4):
    yaw = yaw_est + k * math.pi / 2
    qa, ea, la, ca = ik((ex, ey, CUBE / 2 + APPROACH), yaw, HOME)
    qg, eg, lg, cg = ik((ex, ey, CUBE / 2), yaw, qa)
    ok = ea < POS_TOL and eg < POS_TOL and la and lg and cg
    cost = sum(abs(a - b) for a, b in zip(HOME, qg)) + sum(abs(a - b) for a, b in zip(qa, qg))
    print(f"{math.degrees(yaw) % 360:8.1f} {max(ea, eg) * 1000:10.2f} {cost:7.2f}  {'feasible' if ok else 'rejected'}")
    cands.append((ok, cost, yaw, qa, qg))
feas = sorted([c for c in cands if c[0]], key=lambda c: c[1])
if not feas: sys.exit("no feasible candidate")
_, _, yaw, qa, qg = feas[0]
go_home(); set_grip(0.0); move(HOME, qa); move(qa, qg)
cp, _ = p.getBasePositionAndOrientation(cube); off = np.array(cp) - tcp_pos()
print(f"GRASP offset TCP->cube centre: {np.linalg.norm(off[:2]) * 1000:.1f} mm (xy)  <- calibration quality")
set_grip(0.85)
s = p.getLinkState(robot, EE, computeForwardKinematics=1); inv = p.invertTransform(s[4], s[5])
cpos, corn = p.getBasePositionAndOrientation(cube); lp, lo_ = p.multiplyTransforms(inv[0], inv[1], cpos, corn)
cid = p.createConstraint(robot, EE, cube, -1, p.JOINT_FIXED, [0, 0, 0], lp, [0, 0, 0], lo_, [0, 0, 0, 1])
move(qg, qa)
qpa, e1, _, _ = ik((*PLACE_XY, CUBE / 2 + APPROACH), yaw, qa); qp, e2, _, _ = ik((*PLACE_XY, CUBE / 2 + 0.003), yaw, qpa)
move(qa, qpa, 60); move(qpa, qp)
p.removeConstraint(cid); set_grip(0.0); move(qp, qpa); move(qpa, HOME, 60)
for _ in range(120): p.stepSimulation()
pos, _ = p.getBasePositionAndOrientation(cube)
err = math.hypot(pos[0] - PLACE_XY[0], pos[1] - PLACE_XY[1]) * 1000
print(f"cube final xy=({pos[0]:.3f},{pos[1]:.3f}) place err={err:.1f} mm  {'OK' if err < 10 else 'FAIL'}")
if GUI: time.sleep(3)
