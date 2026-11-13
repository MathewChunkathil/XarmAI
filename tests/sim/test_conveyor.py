"""Conveyor belt + tracking segment tests (headless PyBullet DIRECT, no calibration). SIMULATION ONLY."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "src", "cube_perception"))
from sim import trajectory as traj  # noqa: E402
from sim.belt_task import circ_mean_yaw, fit_track, ray_plane_xy  # noqa: E402
from sim.conveyor import ConveyorCfg  # noqa: E402
from sim.pyb_world import SimWorld  # noqa: E402


@pytest.fixture(scope="module")
def world():
    w = SimWorld(gui=False)
    w.add_conveyor()
    yield w
    w.close()


def test_config_lane_is_separate_from_all_zones(world):
    from sim.zones import boxes_overlap, place_zones
    cfg = world.belt.cfg
    for z in place_zones(world.workspace) + [world.workspace["cube_workspace_box"]]:
        zz = z if "x_m" in z else {"x_m": z["x_m"], "y_m": z["y_m"]}
        assert not boxes_overlap(cfg.box, zz), "lane overlaps %s" % z
    assert cfg.speed_max == pytest.approx(0.06)
    assert cfg.speed_default == pytest.approx(0.03)
    assert cfg.window[0] > 0 and cfg.window[1] < cfg.length


def test_lane_visible_with_margin_in_the_true_camera(world):
    """Project every cube footprint a fed cube can have along the lane (centre s from the feed point to the end stop,
    lateral +-feed_cross, 3.5 cm half diagonal, belt top and cube top) with the TRUE camera: >= 30 px margin to the image."""
    import cv2
    from sim.c270_camera import C270SimCamera, load_cfg
    cam = C270SimCamera(load_cfg(world.cfg_paths["camera"]), np.random.default_rng(0))
    Tc = np.linalg.inv(cam.T_world_cam_true)
    cfg = world.belt.cfg
    pts = []
    for s in np.linspace(cfg.feed_s, cfg.length - 0.035, 8):
        for c in (-cfg.feed_cross, cfg.feed_cross):
            x, y = cfg.to_world(s, c)
            for dx, dy in ((-.035, -.035), (.035, .035), (-.035, .035), (.035, -.035)):
                for z in (cfg.top_z, cfg.top_z + cfg.edge):
                    pc = Tc[:3, :3] @ np.array([x + dx, y + dy, z]) + Tc[:3, 3]
                    uv, _ = cv2.projectPoints(pc.reshape(1, 1, 3), np.zeros(3), np.zeros(3), cam.K_true, cam.dist_true)
                    pts.append(uv.ravel())
    pts = np.array(pts)
    assert pts.min() > 30 and pts[:, 0].max() < cam.W - 30 and pts[:, 1].max() < cam.H - 30


def test_belt_moves_cube_exactly_v_dt_per_step_and_keeps_yaw(world):
    cfg = world.belt.cfg
    x, y = cfg.to_world(cfg.feed_s, 0.004)
    world.spawn_cube(x, y, 0.4, number=1, z=cfg.rest_z + 0.002)
    world.belt.set_speed(0.05)
    world.belt.running = True
    T0 = world.cube_pose_true(1)
    n = 240
    for _ in range(n):
        world._tick()
    T1 = world.cube_pose_true(1)
    s0, c0 = cfg.from_world(*T0[:2, 3])
    s1, c1 = cfg.from_world(*T1[:2, 3])
    assert (s1 - s0) == pytest.approx(0.05 * n / 240, abs=2e-4)          # 50 mm in 1 s, no physics drift
    assert c1 == pytest.approx(c0, abs=2e-4)
    yaw0 = math.atan2(T0[1, 0], T0[0, 0])
    yaw1 = math.atan2(T1[1, 0], T1[0, 0])
    assert yaw1 == pytest.approx(yaw0, abs=0.02)
    assert T1[2, 3] == pytest.approx(T0[2, 3], abs=1e-3)


def test_stopped_belt_does_not_move_cubes_and_cleats_stay(world):
    cfg = world.belt.cfg
    world.belt.running = False
    T0 = world.cube_pose_true(1)
    ph = world.belt.phase
    for _ in range(120):
        world._tick()
    assert np.allclose(world.cube_pose_true(1)[:3, 3], T0[:3, 3], atol=3e-4)
    assert world.belt.phase == ph


def test_cube_stops_at_the_end_stop(world):
    cfg = world.belt.cfg
    world.belt.set_speed(0.06)
    world.belt.running = True
    for _ in range(int(7 * 240)):                                    # 7 s * 60 mm/s >> lane length
        world._tick()
    T = world.cube_pose_true(1)
    s, _ = cfg.from_world(*T[:2, 3])
    yaw = math.atan2(T[1, 0], T[0, 0])
    assert s == pytest.approx(cfg.length - cfg.half_extent(yaw), abs=2e-3)
    assert s + cfg.half_extent(yaw) <= cfg.length + 1e-3
    s_a = s
    for _ in range(120):
        world._tick()
    assert cfg.from_world(*world.cube_pose_true(1)[:2, 3])[0] == pytest.approx(s_a, abs=5e-4)    # stays there
    world.belt.running = False


def test_attached_cube_is_not_carried_by_the_belt(world):
    cfg = world.belt.cfg
    x, y = cfg.to_world(0.1, 0.0)
    world.spawn_cube(x, y, 0.0, number=2, z=cfg.rest_z + 0.002)
    world.activate_cube(2)
    world.set_arm([0.0, -0.5, -0.5, 0.0, 0.0])
    world.attach_cube()
    world.belt.set_speed(0.05)
    world.belt.running = True
    p0 = np.array(world.cube_pose_true(2)[:3, 3])
    for _ in range(60):
        world._tick()
    assert np.allclose(world.cube_pose_true(2)[:3, 3], p0, atol=1e-3)
    world.detach_cube()
    world.belt.running = False


def test_cleats_step_with_the_belt(world):
    world.belt.running = True
    world.belt.set_speed(0.06)
    ph = world.belt.phase
    for _ in range(240):
        world._tick()
    assert world.belt.phase - ph == pytest.approx(0.06, abs=1e-6)
    world.belt.running = False


# ----------------------------------------------------------------------------- tracking segments + fit
def test_tracking_path_derivatives_are_analytic():
    T, p0, v = 1.3, np.array([.1, .2, .3]), np.array([-.05, .01, 0.0])
    tau = np.linspace(0.02, 0.98, 40)
    h = 1e-6
    for w0, w1, dz in ((0, 1, 0), (1, 1, -.07), (1, 0, .08)):
        P, Pd, Pdd = traj.tracking_path(p0, v, w0, w1, dz, T, tau)
        Pp, Pdp, _ = traj.tracking_path(p0, v, w0, w1, dz, T, tau + h / T)
        Pm, Pdm, _ = traj.tracking_path(p0, v, w0, w1, dz, T, tau - h / T)
        assert np.abs((Pp - Pm) / (2 * h) - Pd).max() < 1e-7
        assert np.abs((Pdp - Pdm) / (2 * h) - Pdd).max() < 1e-6
        P1, Pd1, _ = traj.tracking_path(p0, v, w0, w1, dz, T, np.array([1.0]))
        assert Pd1[0, :2] == pytest.approx(w1 * v[:2])               # end velocity = w1 * v (lift ends at rest)


def test_tracking_segment_matches_its_equation_and_qd_is_consistent(world):
    lim = world.limits
    q0 = np.array([0.0, -0.5, -0.5, 0.0, 0.0])
    T0 = world.fk_tcp(q0)
    # a tool-down pose over the lane
    cfg = world.belt.cfg
    x, y = cfg.to_world(0.12, 0.0)
    Rd = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1.0]])
    Tt = np.eye(4)
    Tt[:3, :3], Tt[:3, 3] = Rd, [x, y, 0.11]
    ok, q, pe, re = world.ik_tcp(Tt)
    assert ok
    Tf = world.fk_tcp(q)
    v = np.array([cfg.dir[0] * 0.04, cfg.dir[1] * 0.04, 0.0])
    seg, why = traj.plan_tracking("test", q, Tf[:3, 3], Tf[:3, :3], v, 1.0, 1.0, -0.07, 0.8, lim, world)
    assert seg is not None, why
    assert seg.kind == "cartesian-tracking"
    tau = np.arange(1, seg.n + 1) / seg.n
    P, Pd, Pdd = traj.tracking_path(Tf[:3, 3], v, 1.0, 1.0, -0.07, seg.T, tau)
    for i in (0, seg.n // 3, seg.n - 1):                             # q(t) really follows p(t)
        assert np.linalg.norm(world.fk_tcp(seg.Q[i])[:3, 3] - P[i]) < 6e-4
    # analytic qd vs finite difference of the IK samples (interior)
    fd = (seg.Q[2:] - seg.Q[:-2]) / (2 * seg.dt)
    assert np.abs(fd - seg.QD[1:-1]).max() < 5e-3
    fdd = (seg.Q[2:] - 2 * seg.Q[1:-1] + seg.Q[:-2]) / seg.dt ** 2
    assert np.abs(fdd - seg.QDD[1:-1]).max() < 0.2
    pv, pa = seg.peak()
    assert np.all(pv <= lim.vmax * 1.0001) and np.all(pa <= lim.amax * 1.0001)


def test_tracking_segment_fails_cleanly_when_unreachable(world):
    lim = world.limits
    q0 = np.array([0.0, -0.5, -0.5, 0.0, 0.0])
    Tf = world.fk_tcp(q0)
    far = np.array([-0.04, 0.0, 0.0])                                  # 4 cm/s toward the base ...
    seg, why = traj.plan_tracking("far", q0, np.array([1.4, 0.0, 0.2]), Tf[:3, :3], far, 1.0, 1.0, 0.0, 0.5, lim, world)
    assert seg is None and "IK" in why


def synth(v=0.03, n=6, dt=0.1, noise=0.0004, seed=0, cfg=None):
    rng = np.random.default_rng(seed)
    ts = 1.0 + dt * np.arange(n)
    s = 0.05 + v * (ts - ts[0])
    xy = np.array([cfg.to_world(si, 0.006) for si in s]) + rng.normal(0, noise, (n, 2))
    return ts, xy


def test_fit_track_recovers_speed_and_reports_uncertainty(world):
    cfg = world.belt.cfg
    errs = []
    for seed in range(40):
        ts, xy = synth(0.04, cfg=cfg, seed=seed)
        f = fit_track(ts, xy, cfg)
        errs.append(f["v_s"] - 0.04)
        assert f["n_used"] >= 5 and f["sigma_v"] > 0
        assert f["c_ref"] == pytest.approx(0.006, abs=1e-3)
    assert abs(np.mean(errs)) < 3 * np.std(errs) / math.sqrt(40) + 1e-4
    # the reported sigma is about the real scatter (within 2x)
    assert 0.5 < np.std(errs) / f["sigma_v"] < 2.0


def test_fit_track_rejects_a_wild_frame(world):
    cfg = world.belt.cfg
    ts, xy = synth(0.03, cfg=cfg, noise=0.0002)
    xy[2] += cfg.dir * 0.02                                            # 20 mm outlier along the flow
    f = fit_track(ts, xy, cfg)
    assert f["n_used"] == 5 and abs(f["v_s"] - 0.03) < 5e-4


def test_fit_track_reference_is_the_last_frame(world):
    cfg = world.belt.cfg
    ts, xy = synth(0.02, cfg=cfg, noise=0.0)
    f = fit_track(ts, xy, cfg)
    assert f["t_ref"] == pytest.approx(ts[-1]) and f["s_ref"] == pytest.approx(0.05 + 0.02 * 0.5, abs=1e-9)


def test_ray_plane_and_yaw_mean():
    Tbc = np.eye(4)
    Tbc[:3, 3] = [0.65, 0.0, 0.60]
    Tcc = np.eye(4)
    Tcc[:3, 3] = [0.0, 0.0, 0.5]                    # centre 0.5 m ahead in camera z (here == base z, down is not modelled)
    x, y = ray_plane_xy(Tbc, Tcc, z_plane=0.0)      # ray from (0.65, 0, 0.6) through (0.65, 0, 1.1) hits z = 0 behind the camera: sign check
    assert math.isfinite(x) and abs(x - 0.65) < 1e-9
    assert abs(abs(circ_mean_yaw([math.radians(44), math.radians(-44)])) - math.radians(45)) < math.radians(1.5)
    assert abs(circ_mean_yaw([0.01, 0.03]) - 0.02) < 1e-9
