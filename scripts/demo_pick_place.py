r"""DEMO-4 throwaway: PyBullet top-down pick-place, hard-coded cube pose.
Labels: SIMULATION ONLY | grasp = kinematic attach | gripper = xArm Gripper stand-in
| robot = xArm6 URDF stand-in (xArm5 URDF not in pybullet_data) | camera not calibrated.
Run: .venv\Scripts\python scripts\demo_pick_place.py [--gui]
"""
import math, os, sys, time
import numpy as np
import pybullet as p, pybullet_data

GUI = "--gui" in sys.argv
CUBE = 0.04                      # m, cube edge (spec: 40 mm)
CUBE_POSE = (0.35, 0.10, CUBE/2)  # hard-coded pick pose (x,y,z), yaw 20 deg
CUBE_YAW = math.radians(20)
PLACE_XY = (0.30, -0.20)
TCP_OFF = 0.15                   # PLACEHOLDER: unmeasured TCP offset from gripper base (m)
EE = 7                           # xarm_gripper_base_link
ARM = [1, 2, 3, 4, 5, 6]
HOME = [0, -0.5, -0.8, 0, 1.3, 0]
APPROACH = 0.12                  # m above grasp
POS_TOL = 0.005

p.connect(p.GUI if GUI else p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("xarm/xarm6_with_gripper.urdf", useFixedBase=True)
cube = p.createMultiBody(0.05,
    p.createCollisionShape(p.GEOM_BOX, halfExtents=[CUBE/2]*3),
    p.createVisualShape(p.GEOM_BOX, halfExtents=[CUBE/2]*3, rgbaColor=[0.9, 0.2, 0.2, 1]),
    CUBE_POSE, p.getQuaternionFromEuler([0, 0, CUBE_YAW]))
p.changeDynamics(cube, -1, lateralFriction=1.0)
for j, q in zip(ARM, HOME): p.resetJointState(robot, j, q)

lo = [p.getJointInfo(robot, j)[8] for j in ARM]
hi = [p.getJointInfo(robot, j)[9] for j in ARM]
rng = [h - l for l, h in zip(lo, hi)]

def flange_target(xyz, yaw):
    """Top-down gripper orientation + yaw -> target for EE link (TCP offset removed)."""
    q = p.getQuaternionFromEuler([math.pi, 0, yaw])
    R = np.array(p.getMatrixFromQuaternion(q)).reshape(3, 3)
    return list(np.array(xyz) - R @ np.array([0, 0, TCP_OFF])), q

def ik(xyz, yaw, seed):
    for j, q in zip(ARM, seed): p.resetJointState(robot, j, q)
    pos, q = flange_target(xyz, yaw)
    sol = p.calculateInverseKinematics(robot, EE, pos, q, lowerLimits=lo, upperLimits=hi,
        jointRanges=rng, restPoses=seed, maxNumIterations=300, residualThreshold=1e-5)[:6]
    for j, v in zip(ARM, sol): p.resetJointState(robot, j, v)
    got = np.array(p.getLinkState(robot, EE, computeForwardKinematics=1)[4])
    err = np.linalg.norm(got - np.array(pos))
    in_lim = all(l - 1e-3 <= v <= h + 1e-3 for v, l, h in zip(sol, lo, hi))
    hit = [c for c in p.getClosestPoints(robot, plane, 0.0) if c[3] >= 2]  # ignore base
    return list(sol), err, in_lim, len(hit) == 0

def move(q_from, q_to, steps=60, carry=None):
    for t in range(1, steps + 1):
        a = t / steps
        for j, f, g in zip(ARM, q_from, q_to): p.resetJointState(robot, j, f + a * (g - f))
        p.stepSimulation()
        if GUI: time.sleep(1/120)

def set_grip(v):
    for j in (8, 9, 10, 11, 12, 13): p.resetJointState(robot, j, v)

# ---- candidates: 4 yaws (90 deg symmetry of cube) ----
print("SIMULATION ONLY | grasp=kinematic attach | gripper=xArm Gripper stand-in | robot=xArm6 stand-in")
print(f"{'yaw_deg':>8} {'ik_err_mm':>10} {'limits':>7} {'no_floor':>9} {'cost':>7}  verdict")
cands = []
for k in range(4):
    yaw = CUBE_YAW + k * math.pi / 2
    grasp_xyz = (CUBE_POSE[0], CUBE_POSE[1], CUBE/2)
    qa, ea, la, ca = ik((grasp_xyz[0], grasp_xyz[1], grasp_xyz[2] + APPROACH), yaw, HOME)
    qg, eg, lg, cg = ik(grasp_xyz, yaw, qa)
    ok = ea < POS_TOL and eg < POS_TOL and la and lg and cg
    cost = sum(abs(a - b) for a, b in zip(HOME, qg)) + sum(abs(a - b) for a, b in zip(qa, qg))
    why = "feasible" if ok else "rejected: " + ", ".join(
        n for n, f in (("ik_err", ea < POS_TOL and eg < POS_TOL), ("joint_limit", la and lg), ("floor_hit", cg)) if not f)
    print(f"{math.degrees(yaw)%360:8.1f} {max(ea,eg)*1000:10.2f} {str(la and lg):>7} {str(cg):>9} {cost:7.2f}  {why}")
    cands.append((ok, cost, yaw, qa, qg))
feas = sorted([c for c in cands if c[0]], key=lambda c: c[1])
if not feas: sys.exit("no feasible candidate")
_, cost, yaw, qa, qg = feas[0]
print(f"chosen yaw={math.degrees(yaw)%360:.1f} deg cost={cost:.2f}")

# ---- execute ----
for j, q in zip(ARM, HOME): p.resetJointState(robot, j, q)
set_grip(0.0)
move(HOME, qa); move(qa, qg)
set_grip(0.85)                                   # close
cid = p.createConstraint(robot, EE, cube, -1, p.JOINT_FIXED, [0,0,0], [0,0,TCP_OFF],
                         [0,0,0], p.getQuaternionFromEuler([0,0,0]), [0,0,0])
print("ATTACH (kinematic)")
move(qg, qa)                                     # lift
qp_a, e1, l1, c1 = ik((PLACE_XY[0], PLACE_XY[1], CUBE/2 + APPROACH), yaw, qa)
qp, e2, l2, c2 = ik((PLACE_XY[0], PLACE_XY[1], CUBE/2 + 0.003), yaw, qp_a)
print(f"place IK err {max(e1,e2)*1000:.2f} mm, limits {l1 and l2}")
move(qa, qp_a, 80); move(qp_a, qp)
p.removeConstraint(cid); set_grip(0.0)
print("DETACH")
move(qp, qp_a); move(qp_a, HOME, 80)
for _ in range(120): p.stepSimulation()
pos, _ = p.getBasePositionAndOrientation(cube)
err = math.hypot(pos[0]-PLACE_XY[0], pos[1]-PLACE_XY[1]) * 1000
print(f"cube final xy=({pos[0]:.3f},{pos[1]:.3f}) place err={err:.1f} mm  {'OK' if err < 10 else 'FAIL'}")
if GUI: time.sleep(3)
