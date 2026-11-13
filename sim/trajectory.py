"""Timed trajectories for the simulated arm.  SIMULATION ONLY.

Every motion segment is sampled at the sim step dt = 1/240 s and carries analytic q, qd, qdd:

    q_j(t)   = q0_j + D_j * s(tau),               s(tau)  = 10 tau^3 - 15 tau^4 + 6 tau^5,   tau = t / T
    qd_j(t)  = D_j / T   * (30 tau^2 - 60 tau^3 + 30 tau^4)
    qdd_j(t) = D_j / T^2 * (60 tau - 180 tau^2 + 120 tau^3)

Joint segment:      T = max_j max(1.875 |D_j| / vmax_j, sqrt(5.7735 |D_j| / amax_j)), floor t_min, whole steps.
Cartesian segment:  p(t) = p0 + (p1 - p0) s(tau), orientation R0 -> R1 along the same s (constant when R0 == R1),
                    q(t) by IK at every step (Newton polish, seed = previous q).
                    T is raised until the sampled joint speed / acceleration are within the limits.
                    qd, qdd are ANALYTIC (not finite differences of the IK samples): qd = J_w^+ [pdot; w*omega],
                    qdd = J_w^+ ([pddot; w*alpha] - Jdot_w qd), J_w = diag(1,1,1,w,w,w) J, w = IK_W_ROT (the same weighting
                    as the IK, so a 5-DOF arm that cannot follow all 6 twist rows gets the same least-squares compromise as
                    q itself), pdot/pddot from the analytic quintic, omega/alpha from the slerp (0 when R0 == R1),
                    Jdot qd by a symmetric finite difference of J along qd. The tiny end mismatch between the last IK
                    sample and the waypoint q1 is removed by a smooth correction c*s(tau) (also differentiated analytically).
                    If IK fails (pos err > 0.5 mm, joint limit, jump) the segment falls back to the joint quintic and
                    `info["fallback"]` says why.
Limits come from the URDF (velocity x sim_velocity_scale) and config/planning/planning.yaml (PLACEHOLDER values).
"""
import math

import cv2
import numpy as np

DT = 1.0 / 240.0
K_V = 1.875        # peak of s'(tau)  = 30/16
K_A = 5.7735       # peak of s''(tau) = 10/sqrt(3)
POS_TOL_M = 5e-4   # per-step IK position tolerance of a Cartesian segment
IK_W_ROT = 0.1     # row weight of the rotation rows (same as SimWorld.ik_refine)


def s_q(tau):
    tau = np.asarray(tau, float)
    return 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5


def ds_q(tau):
    tau = np.asarray(tau, float)
    return 30 * tau ** 2 - 60 * tau ** 3 + 30 * tau ** 4


def dds_q(tau):
    tau = np.asarray(tau, float)
    return 60 * tau - 180 * tau ** 2 + 120 * tau ** 3


class Limits:
    """vmax / amax per arm joint (rad/s, rad/s^2) + timing placeholders."""

    def __init__(self, vmax, amax, t_min=0.3, grip_time=0.6, cart_v=0.1, dt=DT):
        self.vmax = np.asarray(vmax, float)
        self.amax = np.asarray(amax, float)
        self.t_min, self.grip_time, self.cart_v, self.dt = float(t_min), float(grip_time), float(cart_v), float(dt)


def load_limits(planning_cfg, urdf_vel):
    """planning_cfg: dict from planning.yaml; urdf_vel: URDF velocity limit per arm joint [rad/s]."""
    sc = float(planning_cfg.get("sim_velocity_scale", 0.5))
    a = float(planning_cfg.get("sim_joint_acc_limit_rad_s2", 2.5))
    return Limits(np.asarray(urdf_vel, float) * sc, np.full(len(urdf_vel), a),
                  planning_cfg.get("sim_min_segment_s", 0.3), planning_cfg.get("sim_gripper_time_s", 0.6),
                  planning_cfg.get("sim_cartesian_peak_speed_m_s", 0.1))


class Segment:
    """One timed motion segment: n steps of length dt; arrays Q/QD/QDD are (n, 5), G is (n,) gripper angle."""

    def __init__(self, name, kind, q0, q1, g0, g1, n, dt):
        self.name, self.kind = name, kind
        self.q0, self.q1 = np.asarray(q0, float), np.asarray(q1, float)
        self.g0, self.g1 = float(g0), float(g1)
        self.n, self.dt = int(n), dt
        self.T = n * dt
        self.phase = ""
        self.t0 = 0.0                # sim time at segment start (set by SimWorld.play)
        self.attached = False        # cube attached during this segment
        self.Q = self.QD = self.QDD = None
        self.G = None
        self.info = {}               # kind-specific: cartesian p0/p1/..., fallback reason
        self.T_tcp_end_pb = None     # PyBullet link_tcp pose of the visible world after the last step

    def peak(self):
        """-> (max|qd| per joint, max|qdd| per joint)."""
        return np.abs(self.QD).max(0), np.abs(self.QDD).max(0)


def _steps(T, dt):
    return max(1, int(math.ceil(T / dt - 1e-9)))


def joint_steps(q0, q1, lim):
    """Whole step count of a joint quintic q0 -> q1 within the limits."""
    D = np.abs(np.asarray(q1, float) - np.asarray(q0, float))
    T = max(lim.t_min, float(np.max(np.maximum(K_V * D / lim.vmax, np.sqrt(K_A * D / lim.amax)))))
    return _steps(T, lim.dt)


def plan_joint(name, q0, q1, lim, g=0.0):
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    n = joint_steps(q0, q1, lim)
    seg = Segment(name, "joint-quintic", q0, q1, g, g, n, lim.dt)
    tau = np.arange(1, n + 1) / n
    D = q1 - q0
    seg.Q = q0 + np.outer(s_q(tau), D)
    seg.Q[-1] = q1
    seg.QD = np.outer(ds_q(tau), D) / seg.T
    seg.QDD = np.outer(dds_q(tau), D) / seg.T ** 2
    seg.G = np.full(n, float(g))
    return seg


def plan_gripper(name, q, g0, g1, lim):
    q = np.asarray(q, float)
    n = _steps(lim.grip_time, lim.dt)
    seg = Segment(name, "gripper-quintic", q, q, g0, g1, n, lim.dt)
    tau = np.arange(1, n + 1) / n
    seg.Q = np.tile(q, (n, 1))
    seg.QD = np.zeros((n, 5))
    seg.QDD = np.zeros((n, 5))
    seg.G = g0 + (g1 - g0) * s_q(tau)
    seg.G[-1] = g1
    return seg


def _slerp(R0, R1, s):
    rv, _ = cv2.Rodrigues(R0.T @ R1)
    return np.array([R0 @ cv2.Rodrigues(rv * float(v))[0] for v in s])


def _central(Q, q0, q1, dt):
    """Central-difference qd, qdd of samples Q (n,5); ghost samples before/after = q0 / q1 (s' = 0 there)."""
    P = np.vstack([q0[None], Q, q1[None]])
    qd = (P[2:] - P[:-2]) / (2 * dt)
    qdd = (P[2:] - 2 * P[1:-1] + P[:-2]) / dt ** 2
    return qd, qdd


def analytic_cartesian(world, Q, s, ds, dds, dp, w_axis, w_rot=IK_W_ROT, lam=1e-9):
    """Analytic qd, qdd along IK samples Q (n,5) of a Cartesian line.
    s, ds, dds: s(tau), ds/dt, d2s/dt2 per sample; dp = p1 - p0; w_axis = R0 @ rotvec(R0^T R1) (world-frame total rotation).
    -> qd, qdd (n,5). Weighted damped LS on the 6-D twist (see module doc)."""
    nm = len(world.movable)
    z = [0.0] * nm
    p, rb, tcp = world.p, world.robot, world.tcp_link
    W = np.diag([1, 1, 1, w_rot, w_rot, w_rot])
    I5 = np.eye(5)

    def Jw(q):
        Jt, Jr = p.calculateJacobian(rb, tcp, [0, 0, 0], list(map(float, q)) + [0.0] * (nm - 5), z, z)
        return W @ np.vstack([np.array(Jt)[:, :5], np.array(Jr)[:, :5]])

    def lsq(J, v):
        return np.linalg.solve(J.T @ J + lam * I5, J.T @ v)

    qd, qdd = np.zeros_like(Q), np.zeros_like(Q)
    eps = 1e-6
    for i in range(len(Q)):
        J = Jw(Q[i])
        v = W @ np.concatenate([dp * ds[i], w_axis * ds[i]])
        a = W @ np.concatenate([dp * dds[i], w_axis * dds[i]])
        qd[i] = lsq(J, v)
        Jdot = (Jw(Q[i] + eps * qd[i]) - Jw(Q[i] - eps * qd[i])) / (2 * eps)
        qdd[i] = lsq(J, a - Jdot @ qd[i])
    return qd, qdd


def plan_line(name, q0, q1, lim, world, g=0.0):
    """Straight Cartesian TCP line between FK(q0) and FK(q1) (quintic s), IK per step on `world` (hidden aux world;
    its arm state is restored). Falls back to the joint quintic on any problem (reason in seg.info['fallback'])."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    T0, T1 = world.fk_tcp(q0), world.fk_tcp(q1)
    p0, p1 = T0[:3, 3], T1[:3, 3]
    L = float(np.linalg.norm(p1 - p0))

    def fallback(reason):
        seg = plan_joint(name, q0, q1, lim, g)
        seg.info = {"fallback": reason, "p0": p0, "p1": p1, "R0": T0[:3, :3], "R1": T1[:3, :3], "length_m": L}
        return seg

    if L < 1e-5:
        return fallback("zero length")
    T = max(lim.t_min, K_V * L / lim.cart_v)
    for attempt in range(6):
        n = _steps(T, lim.dt)
        tau = np.arange(1, n + 1) / n
        s = s_q(tau)
        Rs = _slerp(T0[:3, :3], T1[:3, :3], s)
        Q = np.zeros((n, 5))
        qprev = q0.copy()
        worst = 0.0
        for i in range(n):
            Tt = np.eye(4)
            Tt[:3, :3] = Rs[i]
            Tt[:3, 3] = p0 + (p1 - p0) * s[i]
            q, pe, re = world.ik_refine(Tt, qprev)
            worst = max(worst, pe)
            if pe > POS_TOL_M or np.any(q < world.arm_lo) or np.any(q > world.arm_hi):
                return fallback("IK step %d/%d pos err %.2f mm (tol %.1f) or joint limit" % (
                    i + 1, n, 1000 * pe, 1000 * POS_TOL_M))
            if float(np.max(np.abs(q - qprev))) > 0.05:
                return fallback("IK jump %.3f rad at step %d/%d" % (float(np.max(np.abs(q - qprev))), i + 1, n))
            Q[i] = qprev = q
        if float(np.max(np.abs(Q[-1] - q1))) > 2e-3:
            return fallback("end of line differs from waypoint q by %.4f rad" % float(np.max(np.abs(Q[-1] - q1))))
        c = q1 - Q[-1]                                   # end mismatch (<= 2e-3 rad): smooth correction c*s(tau)
        Rrel = cv2.Rodrigues(T0[:3, :3].T @ T1[:3, :3])[0].ravel()
        T_s = n * lim.dt
        qd, qdd = analytic_cartesian(world, Q + np.outer(s, c), s, ds_q(tau) / T_s, dds_q(tau) / T_s ** 2,
                                     p1 - p0, T0[:3, :3] @ Rrel)
        Q = Q + np.outer(s, c)
        Q[-1] = q1
        qd = qd + np.outer(ds_q(tau), c) / T_s
        qdd = qdd + np.outer(dds_q(tau), c) / T_s ** 2
        ratio = max(float(np.max(np.abs(qd) / lim.vmax)), math.sqrt(float(np.max(np.abs(qdd) / lim.amax))))
        if ratio <= 1.0:
            seg = Segment(name, "cartesian-quintic", q0, q1, g, g, n, lim.dt)
            seg.Q, seg.QD, seg.QDD = Q, qd, qdd
            seg.G = np.full(n, float(g))
            seg.info = {"p0": p0, "p1": p1, "R0": T0[:3, :3], "R1": T1[:3, :3], "length_m": L,
                        "ik_worst_pos_err_m": worst, "attempts": attempt + 1}
            return seg
        T *= ratio * 1.03
    return fallback("limits not met after 6 time scalings")


# ----------------------------------------------------------------------------- tracking segments (moving target)
def Is_q(tau):
    """Integral of s(tau): 2.5 tau^4 - 3 tau^5 + tau^6."""
    tau = np.asarray(tau, float)
    return 2.5 * tau ** 4 - 3 * tau ** 5 + tau ** 6


def tracking_path(p0, v, w0, w1, dz, T, tau):
    """Analytic TCP path of a 'cartesian-tracking' segment at tau = t/T (arrays):

        p(t)   = p0 + v*Ib(t) + dz*s(tau) e_z,           Ib(t) = w0 t + (w1 - w0) T Is(tau),   Is(tau) = 2.5 tau^4 - 3 tau^5 + tau^6
        pd(t)  = v*(w0 + (w1 - w0) s(tau)) + dz*s'(tau)/T e_z
        pdd(t) = v*(w1 - w0) s'(tau)/T + dz*s''(tau)/T^2 e_z

    v = estimated belt velocity (3-vector, z = 0); w = belt-velocity weight (w0 -> w1 along the quintic s):
    hover-track ramp-in 0 -> 1, tracking descent / close 1 -> 1, lift 1 -> 0 (velocity blended to zero).
    -> p (n,3), pd (n,3), pdd (n,3)."""
    tau = np.atleast_1d(np.asarray(tau, float))
    p0, v = np.asarray(p0, float), np.asarray(v, float)
    ez = np.array([0.0, 0.0, 1.0])
    t = tau * T
    Ib = w0 * t + (w1 - w0) * T * Is_q(tau)
    p = p0[None] + np.outer(Ib, v) + dz * np.outer(s_q(tau), ez)
    pd = np.outer(w0 + (w1 - w0) * s_q(tau), v) + dz * np.outer(ds_q(tau) / T, ez)
    pdd = np.outer((w1 - w0) * ds_q(tau) / T, v) + dz * np.outer(dds_q(tau) / T ** 2, ez)
    return p, pd, pdd


def analytic_twist(world, Q, pdot, pddot, w_rot=IK_W_ROT, lam=1e-9):
    """Analytic qd, qdd along IK samples Q (n,5) for a path with CONSTANT orientation and given TCP velocity / acceleration
    (n,3): same weighted damped LS as analytic_cartesian (omega = alpha = 0)."""
    nm = len(world.movable)
    z = [0.0] * nm
    p, rb, tcp = world.p, world.robot, world.tcp_link
    W = np.diag([1, 1, 1, w_rot, w_rot, w_rot])
    I5 = np.eye(5)

    def Jw(q):
        Jt, Jr = p.calculateJacobian(rb, tcp, [0, 0, 0], list(map(float, q)) + [0.0] * (nm - 5), z, z)
        return W @ np.vstack([np.array(Jt)[:, :5], np.array(Jr)[:, :5]])

    def lsq(J, v):
        return np.linalg.solve(J.T @ J + lam * I5, J.T @ v)

    qd, qdd = np.zeros_like(Q), np.zeros_like(Q)
    eps = 1e-6
    for i in range(len(Q)):
        J = Jw(Q[i])
        v = W @ np.concatenate([pdot[i], np.zeros(3)])
        a = W @ np.concatenate([pddot[i], np.zeros(3)])
        qd[i] = lsq(J, v)
        Jdot = (Jw(Q[i] + eps * qd[i]) - Jw(Q[i] - eps * qd[i])) / (2 * eps)
        qdd[i] = lsq(J, a - Jdot @ qd[i])
    return qd, qdd


def plan_tracking(name, q0, p0, R, v, w0, w1, dz, T, lim, world, g0=0.0, g1=0.0, scale=True):
    """Cartesian segment whose target moves with the (estimated) belt velocity: p(t) = tracking_path(...) (see there),
    constant orientation R, IK per step on `world` (hidden aux world, arm state restored), q(t) by Newton polish seeded with
    the previous q, qd / qdd ANALYTIC (analytic_twist). The gripper follows a quintic g0 -> g1 over the same time
    (closing while tracking). T is raised (x1.03 ratio) until the sampled joint speed / acceleration are within the limits
    when `scale`. Returns (Segment, reason): Segment None + reason text when IK fails (pos err > 0.5 mm, joint limit, jump)
    or the limits cannot be met."""
    q0 = np.asarray(q0, float)
    p0, v = np.asarray(p0, float), np.asarray(v, float)
    T = max(float(T), lim.dt)
    for attempt in range(6):
        n = _steps(T, lim.dt)
        T_s = n * lim.dt
        tau = np.arange(1, n + 1) / n
        P, Pd, Pdd = tracking_path(p0, v, w0, w1, dz, T_s, tau)
        Q = np.zeros((n, 5))
        qprev = q0.copy()
        worst = 0.0
        for i in range(n):
            Tt = np.eye(4)
            Tt[:3, :3] = R
            Tt[:3, 3] = P[i]
            q, pe, re = world.ik_refine(Tt, qprev)
            worst = max(worst, pe)
            if pe > POS_TOL_M or np.any(q < world.arm_lo) or np.any(q > world.arm_hi):
                return None, "IK step %d/%d pos err %.2f mm (tol %.1f) or joint limit (%s)" % (
                    i + 1, n, 1000 * pe, 1000 * POS_TOL_M, name)
            if float(np.max(np.abs(q - qprev))) > 0.05:
                return None, "IK jump %.3f rad at step %d/%d (%s)" % (float(np.max(np.abs(q - qprev))), i + 1, n, name)
            Q[i] = qprev = q
        qd, qdd = analytic_twist(world, Q, Pd, Pdd)
        ratio = max(float(np.max(np.abs(qd) / lim.vmax)), math.sqrt(float(np.max(np.abs(qdd) / lim.amax))))
        if ratio > 1.0 and not scale:
            return None, "joint speed / acceleration limits exceeded (ratio %.2f) in the fixed-time segment %s" % (ratio, name)
        if ratio <= 1.0:
            seg = Segment(name, "cartesian-tracking", q0, Q[-1], g0, g1, n, lim.dt)
            seg.Q, seg.QD, seg.QDD = Q, qd, qdd
            seg.G = g0 + (g1 - g0) * s_q(tau) if g1 != g0 else np.full(n, float(g0))
            if g1 != g0:
                seg.G[-1] = g1
            seg.info = {"p0": p0, "p1": P[-1], "R0": R, "R1": R, "v": v, "w0": float(w0), "w1": float(w1),
                        "dz": float(dz), "ik_worst_pos_err_m": worst, "attempts": attempt + 1, "ratio": ratio,
                        "length_m": float(np.linalg.norm(P[-1] - p0))}
            return seg, ""
        T *= ratio * 1.03
    return None, "limits not met after 6 time scalings (%s)" % name
