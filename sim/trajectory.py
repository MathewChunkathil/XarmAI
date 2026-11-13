"""Timed trajectories for the simulated arm.  SIMULATION ONLY.

Every motion segment is sampled at the sim step dt = 1/240 s and carries analytic q, qd, qdd:

    q_j(t)   = q0_j + D_j * s(tau),               s(tau)  = 10 tau^3 - 15 tau^4 + 6 tau^5,   tau = t / T
    qd_j(t)  = D_j / T   * (30 tau^2 - 60 tau^3 + 30 tau^4)
    qdd_j(t) = D_j / T^2 * (60 tau - 180 tau^2 + 120 tau^3)

Joint segment:      T = max_j max(1.875 |D_j| / vmax_j, sqrt(5.7735 |D_j| / amax_j)), floor t_min, whole steps.
Cartesian segment:  p(t) = p0 + (p1 - p0) s(tau), orientation R0 -> R1 along the same s (constant when R0 == R1),
                    q(t) by IK at every step (Newton polish, seed = previous q). qd, qdd by central differences of the
                    sampled q. T is raised until the sampled joint speed / acceleration are within the limits.
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
        Q[-1] = q1
        qd, qdd = _central(Q, q0, q1, lim.dt)
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
