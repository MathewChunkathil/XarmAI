"""kin_log + trajectory tests (headless PyBullet DIRECT). SIMULATION ONLY."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from sim import kin_log, trajectory as traj  # noqa: E402
from sim.pyb_world import SimWorld  # noqa: E402


@pytest.fixture(scope="module")
def world():
    w = SimWorld(gui=False)
    yield w
    w.close()


def _rand_q(world, rng):
    return rng.uniform(np.array(world.arm_lo) * 0.6, np.array(world.arm_hi) * 0.6)


def test_independent_fk_matches_pybullet(world):
    fk = kin_log.IndepFK(base_pos=(0.0, 0.0, world.table_top_z))
    rng = np.random.default_rng(0)
    for _ in range(50):
        q = _rand_q(world, rng)
        Tp, Tn = world.fk_tcp(q), fk.fk(q)
        assert np.linalg.norm(Tp[:3, 3] - Tn[:3, 3]) < 1e-6
        assert kin_log.rot_angle(Tp[:3, :3], Tn[:3, :3]) < 1e-6


def test_quintic_coefficients_match_samples(world):
    q0, q1 = np.array([0.1, -0.5, -0.5, 0.2, 0.0]), np.array([-0.6, 0.3, -1.2, 0.9, 0.7])
    seg = traj.plan_joint("t", q0, q1, world.limits)
    D, T = q1 - q0, seg.T
    t = (np.arange(seg.n) + 1) * seg.dt
    a3, a4, a5 = 10 * D / T ** 3, -15 * D / T ** 4, 6 * D / T ** 5
    q_poly = q0 + np.outer(t ** 3, a3) + np.outer(t ** 4, a4) + np.outer(t ** 5, a5)
    qd_poly = np.outer(3 * t ** 2, a3) + np.outer(4 * t ** 3, a4) + np.outer(5 * t ** 4, a5)
    qdd_poly = np.outer(6 * t, a3) + np.outer(12 * t ** 2, a4) + np.outer(20 * t ** 3, a5)
    assert np.max(np.abs(seg.Q - q_poly)) < 1e-9
    assert np.max(np.abs(seg.QD - qd_poly)) < 1e-9
    assert np.max(np.abs(seg.QDD - qdd_poly)) < 1e-8
    pv, pa = seg.peak()
    assert np.all(pv <= world.limits.vmax) and np.all(pa <= world.limits.amax)
    # numeric derivative of the sampled q agrees with the analytic qd
    assert np.max(np.abs(np.gradient(seg.Q, seg.dt, axis=0)[2:-2] - seg.QD[2:-2])) < 5e-3


def test_limits_from_urdf_and_planning(world):
    assert np.allclose(world.limits.vmax, 3.14 * 0.5)
    assert world.limits.amax[0] > 0 and world.limits.grip_time > 0


def test_forward_dynamics_round_trip(world):
    p, rb = world.p, world.robot
    rng = np.random.default_rng(1)
    for _ in range(20):
        q = list(_rand_q(world, rng)) + [0.3] * 6
        qd = list(rng.normal(0, 0.5, 5)) + [0.0] * 6
        qdd = list(rng.normal(0, 1.0, 5)) + [0.0] * 6
        tau = np.array(p.calculateInverseDynamics(rb, q, qd, qdd))
        h = np.array(p.calculateInverseDynamics(rb, q, qd, [0.0] * 11))
        M = np.array(p.calculateMassMatrix(rb, q))
        assert np.max(np.abs(np.linalg.solve(M, tau - h) - np.array(qdd))) < 1e-6


def test_cartesian_line_is_straight_and_within_limits(world):
    q0 = np.array([0.0, -0.3, -0.6, 0.9, 0.0])
    T0 = world.fk_tcp(q0)
    T1 = T0.copy()
    T1[2, 3] -= 0.08
    q1, pe, _ = world.ik_refine(T1, q0)
    assert pe < 1e-6
    seg = traj.plan_line("t", q0, q1, world.limits, world)
    assert seg.kind == "cartesian-quintic" and not seg.info.get("fallback")
    p0, p1 = world.fk_tcp(q0)[:3, 3], world.fk_tcp(q1)[:3, 3]
    d = (p1 - p0) / np.linalg.norm(p1 - p0)
    dev = 0.0
    for q in seg.Q[::7]:
        r = world.fk_tcp(q)[:3, 3] - p0
        dev = max(dev, float(np.linalg.norm(r - d * (r @ d))))
    assert dev < 5e-4
    pv, pa = seg.peak()
    assert np.all(pv <= world.limits.vmax) and np.all(pa <= world.limits.amax)


def test_run_log_files(world, tmp_path):
    q0, q1 = np.array([0.0, -0.5, -0.5, 0.0, 0.0]), np.array([0.3, -0.2, -0.8, 0.5, 0.1])
    world.set_arm(q0)
    log = kin_log.RunLog(world, "headless", 0, 3, t_base=world.sim_t)
    world.on_segment = log.add_segment
    world.move_joints(world.arm_q(), q1, name="seg")
    world.on_segment = None
    rec = {"run": 3, "true_x": 0.35, "true_y": -0.05, "true_yaw_deg": 25.0, "outcome": "SUCCESS"}
    txt, csv_ = log.write(str(tmp_path), {"rec": rec, "cube_mass": 0.05})
    assert os.path.basename(txt).endswith("_run03_x350_y-050_yaw+025_SUCCESS.txt")
    assert txt[:-4] == csv_[:-4]
    body = open(txt, encoding="utf-8").read()
    assert "SIMULATION ONLY" in body and "max FD round-trip err" in body
    assert log.summary["fd"] < 1e-6 and log.summary["fk_mm"] < 1e-3
    rows = open(csv_, encoding="utf-8").read().splitlines()
    assert rows[0].startswith("t_s,segment,phase,q1_rad") and len(rows) == 1 + log.segs[0].n
