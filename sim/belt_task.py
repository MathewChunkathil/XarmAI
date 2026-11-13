"""Conveyor tasks: start / stop / speed, feed a cube, and PICK A MOVING CUBE from the belt.  SIMULATION ONLY.

Pick-from-belt flow (TaskRunner task "pick_belt"):
  1. arm waits at the belt-side park pose (out of the camera view).
  2. VISION TRACKING: N >= 3 camera frames over ~0.5 s of sim time while the cube moves; every frame -> tags -> PnP (K_est)
     -> base frame; a least-squares line fit of the position along the lane gives the cube velocity + its uncertainty.
     The robot does NOT read the belt speed from config; the true belt speed is used only to score the estimate.
  3. INTERCEPT PLAN: earliest grasp time t* such that the arm can be above the predicted cube by t* - t_descend - t_hover
     (joint quintic timing from sim/trajectory.py); candidates / neighbour filter / IK as for a static cube, on the
     predicted pose. The grasp contact must lie inside the pick window.
  4. EXECUTION: joint move to the pregrasp above the predicted point (arrives early) -> hover-track (belt velocity ramps in)
     -> TRACKING descent (Cartesian line whose target moves with the estimated velocity) -> close the gripper while
     tracking -> kinematic attach -> lift with the belt velocity blended to zero -> normal place to the commanded zone.
  Abort -> MISSED when the pick window is passed, the cube would hit the end stop, or IK / limits fail.
Honesty: perception uses only ESTIMATED K / T_base_cam; true poses are used for scoring only; all belt numbers are PLACEHOLDER.
"""
import math
import time
from typing import Dict, List, Optional

import numpy as np

from sim.task_runner import TaskError, T_of, rz, wrap90, M as TM  # noqa: E402,I001  (also puts src/* on sys.path)
from candidate_evaluator import Obstacle, evaluate, format_table, generate, rank
from candidate_evaluator.generate import pose_to_matrix
from candidate_evaluator.ik_backend import joint_margin
from cube_perception import cube_yaw, estimate_cube_pose, fold_yaw, identify_cubes, median_pose, to_base
from sim import trajectory as traj
from sim.pyb_world import Tinv

BELT = {
    "attach_closing_tol_m": 0.0075,     # (open width - cube edge)/2: grasp offset along the closing axis beyond this = fingers miss. PLACEHOLDER
    "plan_passes": 3,
    "t_star_guess_s": 2.0,
    "outlier_mm": 2.5,                  # frame rejected when its residual to the line fit exceeds this (after the first fit)
}


# ----------------------------------------------------------------------------- pure helpers (unit tested)
def fit_track(ts, xy, cfg, outlier_m=BELT["outlier_mm"] / 1000.0):
    """Least-squares linear motion of a cube from per-frame base-frame positions.

    ts (n,) sim times [s], xy (n,2) metres. The belt DIRECTION is the installation geometry of the lane (config); the
    belt SPEED is estimated: s_i = s0 + v (t_i - t_mean) along the flow (least squares, leave-one-out rejection of frames more than 2.5 mm off the line), the lateral
    position c is the mean (no lateral motion). -> dict(v_s, sigma_v, rms, n_used, s_ref, c_ref, x_ref, y_ref, t_ref,
    mask, v_xy_unconstrained) with the reference time t_ref = last frame time. sigma_v = residual sigma / sqrt(sum (t - t_mean)^2)
    (n - 2 degrees of freedom)."""
    ts = np.asarray(ts, float)
    xy = np.asarray(xy, float)
    n = len(ts)
    d = xy - np.asarray(cfg.start)[None]
    s, c = d @ cfg.dir, d @ cfg.nrm
    mask = np.ones(n, bool)

    def line(msk):
        tm_ = ts[msk].mean()
        A_ = np.vstack([np.ones(msk.sum()), ts[msk] - tm_]).T
        return np.linalg.lstsq(A_, s[msk], rcond=None)[0], tm_

    for _ in range(2):                                   # leave-one-out outlier rejection (needs >= 5 frames to start)
        if mask.sum() < 5:
            break
        idx = np.flatnonzero(mask)
        loo = []
        for i in idx:
            m2 = mask.copy()
            m2[i] = False
            cf, tm_ = line(m2)
            loo.append(abs(s[i] - (cf[0] + cf[1] * (ts[i] - tm_))))
        k = int(np.argmax(loo))
        if loo[k] <= outlier_m:
            break
        mask[idx[k]] = False
    coef, tm = line(mask)
    res = s - (coef[0] + coef[1] * (ts - tm))
    m = int(mask.sum())
    sxx = float(np.sum((ts[mask] - tm) ** 2))
    dof = max(m - 2, 1)
    rms = float(np.sqrt(np.sum(res[mask] ** 2) / dof))
    sigma_v = rms / math.sqrt(sxx) if sxx > 0 else float("inf")
    t_ref = float(ts[-1])
    s_ref = float(coef[0] + coef[1] * (t_ref - tm))
    c_ref = float(c[mask].mean())
    x_ref, y_ref = cfg.to_world(s_ref, c_ref)
    # unconstrained 2-D check (reported only)
    A2 = np.vstack([np.ones(m), ts[mask] - tm]).T
    cx, *_ = np.linalg.lstsq(A2, xy[mask, 0], rcond=None)
    cy, *_ = np.linalg.lstsq(A2, xy[mask, 1], rcond=None)
    return {"v_s": float(coef[1]), "sigma_v": float(sigma_v), "rms": rms, "n_used": m, "s_ref": s_ref, "c_ref": c_ref,
            "x_ref": x_ref, "y_ref": y_ref, "t_ref": t_ref, "mask": mask, "v_xy_unconstrained": (float(cx[1]), float(cy[1]))}


def ray_plane_xy(T_base_cam, T_cam_cube, z_plane):
    """Cube centre on the known resting plane: the camera RAY through the PnP centre is accurate (image-plane position),
    the PnP depth along it is not. Intersect the ray with the plane z = z_plane (belt top + edge/2). Uses only the ESTIMATED
    T_base_cam. -> (x, y) in the base frame."""
    T_base_cam = np.asarray(T_base_cam, float)
    o = T_base_cam[:3, 3]
    c = (T_base_cam @ np.asarray(T_cam_cube, float))[:3, 3]
    d = c - o
    k = (z_plane - o[2]) / d[2]
    p = o + k * d
    return float(p[0]), float(p[1])


def circ_mean_yaw(yaws):
    """Mean of cube yaws (period 90 deg) -> folded into [-pi/4, pi/4)."""
    a = 4.0 * np.asarray(yaws, float)
    return fold_yaw(float(math.atan2(np.mean(np.sin(a)), np.mean(np.cos(a))) / 4.0))


class BeltOps:
    """Belt operations for a TaskRunner (R)."""

    def __init__(self, R):
        self.R = R
        self.P = R.P
        self.w = R.w
        self.cfg = None

    # ------------------------------------------------------------------ setup / simple ops
    def ensure(self):
        belt = self.w.add_conveyor()
        if belt is None:
            raise TaskError("no conveyor: workspace.yaml has no conveyor: section")
        self.cfg = belt.cfg
        return belt

    def at_park(self):
        return float(np.max(np.abs(np.asarray(self.w.arm_q()) - np.asarray(self.cfg.park_q)))) < 1e-3

    def to_park(self):
        """Arm to the belt-side park pose, gripper open (GUI: visible motion; headless: teleport like the static tasks)."""
        w, P = self.w, self.P
        if w.cube_attached is not None:
            w.detach_cube()
        if self.at_park():
            w.set_gripper(0.0)
            return
        if P.args.gui:
            w.move_gripper(0.0, name="belt_open")
            P.move(self.cfg.park_q, "to_belt_park")
        else:
            w.set_gripper(0.0)
            w.set_arm(self.cfg.park_q)

    def set_running(self, on):
        belt = self.ensure()
        belt.running = bool(on)
        return belt

    def set_speed_cm_s(self, v):
        belt = self.ensure()
        belt.set_speed(v / 100.0)
        return belt

    def run_simple(self, task, rec):
        belt = self.ensure()
        if task.action == "belt_start":
            belt.running = True
            rec["detail"] = "belt running at %.1f cm/s" % (100 * belt.speed)
        elif task.action == "belt_stop":
            belt.running = False
            rec["detail"] = "belt stopped"
        else:
            belt.set_speed(task.value / 100.0)
            rec["detail"] = "belt speed set to %.1f cm/s%s" % (100 * belt.speed, "" if belt.running else " (belt stopped)")
        rec["outcome"] = "SUCCESS"
        self.P.log("  " + rec["detail"])
        return rec

    def feed(self, task, rec):
        """Put cube n at the belt start (random yaw, random lateral offset). Replaces the cube if it is elsewhere."""
        R, P, w, cfg = self.R, self.P, self.w, None
        self.ensure()
        cfg = self.cfg
        n = task.cube
        self.to_park()                           # the arm waits at the belt side BEFORE the cube appears
        if n in w.cubes:
            R.remove_cube(n)
        c = float(P.rng.uniform(-cfg.feed_cross, cfg.feed_cross))
        x, y = cfg.to_world(cfg.feed_s, c)
        yaw = float(P.rng.uniform(-math.pi, math.pi))
        w.spawn_cube(x, y, yaw, number=n, z=cfg.rest_z + 0.002)
        R.rec[n] = {"x": x, "y": y, "z": cfg.rest_z, "yaw": yaw, "src": "fed on the belt at the feed end (the belt moves it)",
                    "zone": None}
        R.release_supports(n)
        Tt = w.cube_pose_true(n)
        rec.update(true_x=Tt[0, 3], true_y=Tt[1, 3], true_yaw_deg=math.degrees(cube_yaw(Tt)),
                   outcome="SUCCESS", detail="%s cube fed at s=%.0f mm, lateral %+.1f mm, yaw %.0f deg; belt %s" % (
                       R.names[n], 1000 * cfg.feed_s, 1000 * c, math.degrees(yaw),
                       ("running %.1f cm/s" % (100 * w.belt.speed)) if w.belt.running else "STOPPED"))
        P.log("  " + rec["detail"])
        return rec

    # ------------------------------------------------------------------ perception (moving cube)
    def perceive_series(self, A):
        """N frames, sim time advancing between them (the belt moves the cube). -> (track list for cube A, ests for all
        cubes, info). Cube A uses the belt top as its support plane; the others the table (median over the frames)."""
        R, P, w, cfg = self.R, self.P, self.w, self.cfg
        N, dt = max(3, cfg.track_frames), cfg.track_dt
        per = {n: {"Ts": [], "ids": set(), "reasons": []} for n in R.cube_set}
        track = []
        img0, dets0 = None, {}
        for f in range(N):
            if f:
                w.idle(dt)
            t = w.sim_t
            img = P.grab()
            d = P.detect(img)
            if f == 0:
                img0, dets0 = img, d
            for n, dets in identify_cubes(list(d.items()), R.cube_set).items():
                pc = P.perc_cfg(TM["max_level"])
                if n == A:
                    pc = dict(pc, table_z=cfg.top_z, max_level=0)
                r = estimate_cube_pose(dets, P.K, P.dist, R.cube_set[n], pc)
                if r.valid:
                    per[n]["Ts"].append(r.T_cam_cube)
                    per[n]["ids"].update(r.tag_ids_used)
                    if n == A:
                        Tb = to_base(P.T_base_cam, r.T_cam_cube, cfg.top_z, R.edge, max_level=0)
                        x_r, y_r = ray_plane_xy(P.T_base_cam, r.T_cam_cube, cfg.rest_z)
                        track.append({"t": t, "x": x_r, "y": y_r, "z": float(Tb[2, 3]),
                                      "yaw": cube_yaw(Tb), "tags": sorted(r.tag_ids_used)})
                else:
                    per[n]["reasons"].append(r.invalid_reason)
        ests = {}
        for n, v in per.items():
            if n == A or not v["Ts"]:
                continue
            Tb = to_base(P.T_base_cam, median_pose(v["Ts"]), R.table_z, R.edge, max_level=TM["max_level"])
            ests[n] = {"x": float(Tb[0, 3]), "y": float(Tb[1, 3]), "z": float(Tb[2, 3]), "yaw": cube_yaw(Tb),
                       "T_cc": median_pose(v["Ts"]), "valid": len(v["Ts"]), "ids": sorted(v["ids"])}
        owned = {i for m in R.cube_set.values() for i in m.ids}
        info = {"img": img0, "dets": {i: c for i, c in dets0.items() if i in owned}, "reasons": per[A]["reasons"], "N": N}
        return track, ests, info

    # ------------------------------------------------------------------ planning
    def ideal(self, ctx, t):
        """Predicted cube CENTRE at sim time t: reference pose + estimated velocity (constant)."""
        return ctx["c_ref"] + ctx["v_vec"] * (t - ctx["t_ref"])

    def obstacles_at(self, ests, A, ctx, t_star, z_tcp):
        """Other cubes at time t* (perceived poses; cubes that ride the belt are moved with the estimated velocity)."""
        R, cfg = self.R, self.cfg
        fb = z_tcp + float(R.fp.get("finger_bottom_above_tcp_m", 0.0))
        out = []
        for n in R.in_scene():
            if n == A:
                continue
            pose, _ = R.locate(n, ests)
            if pose is None or pose["z"] + R.edge / 2 <= fb:
                continue
            x, y = pose["x"], pose["y"]
            s, c = cfg.from_world(x, y)
            if abs(c) < cfg.width / 2 + 0.02 and -0.02 < s < cfg.length + 0.02 and n in ests:
                shift = ctx["v_vec"] * (t_star - ctx["t_ref"])
                x, y = x + float(shift[0]), y + float(shift[1])
            out.append(Obstacle(x, y, pose["yaw"], R.edge, R.names.get(n, str(n))))
        return out

    def plan_intercept(self, c, ctx, q_start, extra_hover=0.0):
        """Segments for candidate c. -> (plan dict, "") or (None, reason)."""
        P, w, cfg, aux = self.P, self.w, self.cfg, self.P.aux
        lim = w.limits
        dt = lim.dt
        lo, hi = P.ik.joint_limits()
        T_g = pose_to_matrix(c.grasp_pose)
        R_tool = T_g[:3, :3]
        appr = c.approach_distance_m
        up = np.array([0.0, 0.0, appr])
        g_open, g_closed = 0.0, P.close_ang
        t_now = ctx["t_now"]
        T_h = cfg.hover_s + extra_hover
        seed = list(c.q_pregrasp) if c.q_pregrasp is not None else list(P.q_ready)
        T_join, q_pre = 1.5, None
        for _ in range(8):
            t_h0 = t_now + T_join
            p_pre = self.ideal(ctx, t_h0 + T_h / 2) + up
            Tp = T_of(R_tool, p_pre)
            ok, q, _ = P.ik.solve(Tp, seed)
            if not ok or joint_margin(q, lo, hi) < 0:
                return None, "NO_IK"
            q_pre = np.asarray(q, float)
            T_new = traj.joint_steps(q_start, q_pre, lim) * dt
            if abs(T_new - T_join) < 0.5 * dt:
                T_join = T_new
                break
            T_join = T_new
            seed = list(q)
        seg_join = traj.plan_joint("to_pregrasp", q_start, q_pre, lim, g_open)
        Tf = aux.fk_tcp(q_pre)
        seg_h, why = traj.plan_tracking("hover_track", q_pre, Tf[:3, 3], Tf[:3, :3], ctx["v_vec"], 0.0, 1.0, 0.0, T_h, lim, aux,
                                        g_open, g_open, scale=False)
        if seg_h is None:
            return None, why
        Th = aux.fk_tcp(seg_h.Q[-1])
        T_d = max(lim.t_min, traj.K_V * appr / cfg.descend_speed)
        seg_d, why = traj.plan_tracking("track_descend", seg_h.Q[-1], Th[:3, 3], Th[:3, :3], ctx["v_vec"], 1.0, 1.0, -appr, T_d,
                                        lim, aux, g_open, g_open)
        if seg_d is None:
            return None, why
        Td = aux.fk_tcp(seg_d.Q[-1])
        seg_g, why = traj.plan_tracking("track_close", seg_d.Q[-1], Td[:3, 3], Td[:3, :3], ctx["v_vec"], 1.0, 1.0, 0.0,
                                        lim.grip_time, lim, aux, g_open, g_closed, scale=False)
        if seg_g is None:
            return None, why
        Tg = aux.fk_tcp(seg_g.Q[-1])
        T_l = max(lim.t_min, traj.K_V * c.lift_distance_m / lim.cart_v)
        seg_l, why = traj.plan_tracking("lift_blend", seg_g.Q[-1], Tg[:3, 3], Tg[:3, :3], ctx["v_vec"], 1.0, 0.0, c.lift_distance_m,
                                        T_l, lim, aux, g_closed, g_closed)
        if seg_l is None:
            return None, why
        t_star = t_now + seg_join.T + seg_h.T + seg_d.T                # grasp contact (end of the tracking descent)
        t_att = t_star + seg_g.T                                         # gripper closed -> attach
        ideal_star = self.ideal(ctx, t_star)
        return {"segs": [seg_join, seg_h, seg_d, seg_g, seg_l], "q_pre": q_pre, "q_lift_end": seg_l.Q[-1],
                "T_pre": T_of(R_tool, Tf[:3, 3]), "T_grasp_end": Td, "T_lift_end": aux.fk_tcp(seg_l.Q[-1]),
                "t_star": t_star, "t_att": t_att, "T_join": seg_join.T, "T_hover": seg_h.T, "T_descend": seg_d.T,
                "align_mm": 1000 * float(np.linalg.norm((Td[:3, 3] - ideal_star)[:2])),
                "align_z_mm": 1000 * float(Td[2, 3] - ideal_star[2])}, ""

    # ------------------------------------------------------------------ the task
    def run_pick(self, task, idx, meta, rec, t0):
        R, P, w, a = self.R, self.P, self.w, self.P.args
        log = P.log
        belt = self.ensure()
        cfg = self.cfg
        A = task.cube
        names, edge = R.names, R.edge
        if A not in w.cubes:
            raise TaskError("%s cube is not in the scene (feed it first)" % names.get(A, A))
        x_lines = meta.setdefault("extra_lines", [])

        # ---- 1) wait at the belt park, perceive the moving cube
        P.phase = "perceive"
        self.to_park()
        track, ests, info = self.perceive_series(A)
        src = "perception (camera, %d frames over %.2f s, moving-cube line fit)" % (info["N"], (info["N"] - 1) * cfg.track_dt)
        rec["src_pick"], meta["src_pick"] = src, src
        head = ["task %02d  %s" % (idx, task.describe(names))]
        if len(track) < 3:
            rec["outcome"] = "PERCEPTION_FAIL"
            rec["detail"] = "%s cube seen (valid) in %d of %d frames (need 3)%s" % (
                names[A], len(track), info["N"], (": " + ",".join(sorted(set(info["reasons"])))) if info["reasons"] else "")
            P.show(info["img"], head + ["PERCEPTION_FAIL " + rec["detail"]], info["dets"])
            R._save(idx)
            return rec
        ts = [r["t"] for r in track]
        fit = fit_track(ts, [[r["x"], r["y"]] for r in track], cfg)
        yaw_e = circ_mean_yaw([r["yaw"] for r in track])
        ids = sorted({i for r in track for i in r["tags"]})
        z_e = cfg.rest_z
        # speed scoring (TRUE belt speed + true pose are SCORING ONLY)
        v_true = belt.v
        v_s = fit["v_s"]
        stationary = abs(v_s) < max(cfg.stationary_v, 2 * fit["sigma_v"])
        v_use = 0.0 if stationary else v_s
        Tt = w.cube_pose_true(A)
        s_true, c_true = cfg.from_world(Tt[0, 3], Tt[1, 3])
        rec.update(true_x=Tt[0, 3], true_y=Tt[1, 3], true_yaw_deg=math.degrees(cube_yaw(Tt)), est_x=fit["x_ref"], est_y=fit["y_ref"],
                   est_yaw_deg=math.degrees(yaw_e), pos_err_mm=1000 * math.hypot(fit["x_ref"] - Tt[0, 3], fit["y_ref"] - Tt[1, 3]),
                   yaw_err_deg=math.degrees(wrap90(yaw_e - cube_yaw(Tt))), tags=",".join(map(str, ids)),
                   belt_v_est_mm_s=1000 * v_s, belt_v_true_mm_s=1000 * v_true, belt_v_err_mm_s=1000 * (v_s - v_true),
                   belt_sigma_v_mm_s=1000 * fit["sigma_v"], belt_frames_used=fit["n_used"], belt_frames=info["N"],
                   belt_stationary=bool(stationary))
        x_lines += [
            "BELT TRACKING   : %d/%d frames used over %.3f s (sim time), line fit along the lane (belt direction from config, speed ESTIMATED)" % (
                fit["n_used"], info["N"], ts[-1] - ts[0]),
            "  v_est (along flow) = %.3f mm/s  +- %.3f (1 sigma; residual rms %.3f mm)   v_TRUE = %.3f mm/s (scoring only)   error = %+.3f mm/s%s" % (
                1000 * v_s, 1000 * fit["sigma_v"], 1000 * fit["rms"], 1000 * v_true, 1000 * (v_s - v_true),
                "   -> |v| < max(%.1f mm/s, 2 sigma): treated as STATIONARY (v = 0)" % (1000 * cfg.stationary_v) if stationary else ""),
            "  unconstrained 2-D fit velocity (check only) = (%.2f, %.2f) mm/s ; reference time t_ref = %.4f s: s = %.2f mm, lateral %.2f mm, TRUE s = %.2f mm" % (
                1000 * fit["v_xy_unconstrained"][0], 1000 * fit["v_xy_unconstrained"][1], fit["t_ref"], 1000 * fit["s_ref"],
                1000 * fit["c_ref"], 1000 * s_true)]
        log("  belt tracking: v_est %.2f +- %.2f mm/s (true %.2f, err %+.2f), %d/%d frames%s" % (
            1000 * v_s, 1000 * fit["sigma_v"], 1000 * v_true, 1000 * (v_s - v_true), fit["n_used"], info["N"],
            ", STATIONARY" if stationary else ""))
        ests[A] = {"x": fit["x_ref"], "y": fit["y_ref"], "z": z_e, "yaw": yaw_e, "T_cc": None, "valid": fit["n_used"], "ids": ids}
        lines = head + ["%s est s=%.0f mm  v_est %.1f +- %.1f mm/s (true %.1f)" % (names[A], 1000 * fit["s_ref"], 1000 * v_s,
                                                                                   1000 * fit["sigma_v"], 1000 * v_true)]
        P.show(info["img"], lines, info["dets"])

        # ---- 2) window pre-check (cube already past the pick window / against the stop)
        s0, s1 = cfg.window
        if fit["s_ref"] > s1:
            rec["outcome"] = "MISSED"
            rec["detail"] = "cube already beyond the pick window (s = %.0f mm > %.0f mm)%s" % (
                1000 * fit["s_ref"], 1000 * s1, ", against the end stop" if fit["s_ref"] > cfg.length - 0.06 else "")
            log("  " + rec["detail"])
            P.show(info["img"], lines + ["MISSED " + rec["detail"]], info["dets"])
            R._save(idx)
            return rec

        # ---- 3) target (place zone) + candidates on the PREDICTED pose, intercept plan
        sx, sy, zone = R.pick_slot(task.zone, A, ests)
        tgt = {"x": sx, "y": sy, "z": R.table_z + edge / 2 + 0.002, "yaw": zone["target"]["yaw_rad"]}
        src_t = "zone %d slot (%.3f, %.3f) from workspace.yaml place_zones" % (task.zone, sx, sy)
        rec["src_target"], meta["src_target"] = src_t, src_t
        ctx = {"t_now": w.sim_t, "t_ref": fit["t_ref"], "c_ref": np.array([fit["x_ref"], fit["y_ref"], z_e]),
               "v_vec": np.array([cfg.dir[0] * v_use, cfg.dir[1] * v_use, 0.0])}
        pcfg = dict(P.cfg["planning"], approach_distance_m=cfg.pregrasp_above)
        q_start = np.asarray(w.arm_q(), float)
        t_guess = ctx["t_now"] + BELT["t_star_guess_s"]
        chosen = plan = chosen_place = None
        fails: Dict[str, int] = {}
        res = cands = None
        extra_hover = 0.0
        for pas in range(BELT["plan_passes"]):
            e_g = self.ideal(ctx, t_guess)
            cands = generate((e_g[0], e_g[1], z_e, yaw_e), P.cfg["gripper"], pcfg, P.ws, "task%02d" % idx, edge,
                             obstacles=self.obstacles_at(ests, A, ctx, t_guess, z_e), extra_boxes=[cfg.box])
            evaluate(cands, P.ik, P.q_ready)
            res = rank(cands, P.cfg["ranking"])
            rec["nb_rejected"] = sum(c.rejection_reason == "NEIGHBOR_COLLISION" for c in cands)
            P._cands = [(c.candidate_id, c.rejection_reason or "ok", "", math.degrees(c.grasp_yaw_rad), c.score.total) for c in cands]
            if not res.ranked or res.selected_id is None:
                break
            T_est = T_of(rz(yaw_e), [e_g[0], e_g[1], z_e])
            obs_place = R.obstacles(ests, {A}, tgt["z"])
            order = [c for c in res.ranked if c.candidate_id == res.selected_id] + \
                    [c for c in res.ranked if c.candidate_id != res.selected_id]
            chosen = plan = chosen_place = None
            fails = {}
            for rank_i, c in enumerate(order):
                pl, why = R.plan_place(c, T_est, yaw_e, tgt, obs_place)
                if pl is None:
                    fails["place " + why] = fails.get("place " + why, 0) + 1
                    continue
                ip, why = self.plan_intercept(c, ctx, q_start, extra_hover)
                if ip is None:
                    key = why.split(" (")[0][:60]
                    fails[key] = fails.get(key, 0) + 1
                    continue
                chosen, plan, chosen_place, chosen_rank = c, ip, pl, rank_i
                break
            if chosen is None:
                break
            s_star = cfg.from_world(*self.ideal(ctx, plan["t_star"])[:2])[0]
            if v_use > 0 and s_star < s0 - 1e-4 and extra_hover == 0.0:      # cube not yet in the window: hover longer
                extra_hover = math.ceil((s0 - s_star) / v_use / lim_dt(w) ) * lim_dt(w)
                chosen = plan = None
                continue
            if abs(plan["t_star"] - t_guess) > 0.25 and pas < BELT["plan_passes"] - 1:
                t_guess = plan["t_star"]
                continue
            break
        rec["feasible"] = "%d/%d" % (len(res.ranked) if res is not None else 0, len(cands) if cands else 0)
        log(format_table(cands) if cands else "  (no candidates)")
        if chosen is None or plan is None:
            if res is not None and (not res.ranked or res.selected_id is None):
                why = sorted({c.rejection_reason for c in cands if c.rejection_reason})
                rec["outcome"] = "NO_FEASIBLE"
                rec["detail"] = "no feasible grasp on the predicted pose (%s)" % ", ".join(why)
            else:
                rec["outcome"] = "MISSED"
                rec["detail"] = "intercept impossible: " + ", ".join("%s x%d" % kv for kv in fails.items())
            log("  " + rec["detail"])
            P.show(info["img"], lines + [rec["outcome"] + " " + rec["detail"][:80]], info["dets"])
            R._save(idx)
            return rec
        c = chosen
        e_star = self.ideal(ctx, plan["t_star"])
        e_att = self.ideal(ctx, plan["t_att"])
        s_star = cfg.from_world(e_star[0], e_star[1])[0]
        s_att = cfg.from_world(e_att[0], e_att[1])[0]
        ext = cfg.edge / 2 * math.sqrt(2)
        rec.update(chosen_id=c.candidate_id, chosen_yaw_deg=math.degrees(c.grasp_yaw_rad), chosen_rank=chosen_rank + 1,
                   total=c.score.total, belt_t_star_s=plan["t_star"] - ctx["t_now"], belt_s_star_mm=1000 * s_star,
                   belt_align_mm=plan["align_mm"])
        x_lines += [
            "INTERCEPT PLAN  : grasp contact t* = now + %.3f s (join %.3f + hover %.3f + descend %.3f), predicted s(t*) = %.1f mm "
            "(window %.0f..%.0f mm), s(attach) = %.1f mm (end stop at %.0f mm - half diagonal %.0f mm)" % (
                plan["t_star"] - ctx["t_now"], plan["T_join"], plan["T_hover"], plan["T_descend"], 1000 * s_star, 1000 * s0, 1000 * s1,
                1000 * s_att, 1000 * cfg.length, 1000 * ext),
            "  planned tool-vs-predicted-cube alignment at t*: %.3f mm xy, %.3f mm z; extra hover %.3f s" % (
                plan["align_mm"], plan["align_z_mm"], extra_hover)]
        log("  chosen %s (rank %d), t* = +%.2f s (join %.2f + hover %.2f + descend %.2f), predicted s = %.0f mm" % (
            c.candidate_id, chosen_rank + 1, plan["t_star"] - ctx["t_now"], plan["T_join"], plan["T_hover"], plan["T_descend"], 1000 * s_star))
        if not (s0 - 1e-3 <= s_star <= s1 + 1e-3):
            rec["outcome"] = "MISSED"
            rec["detail"] = "pick window passed: grasp contact would be at s = %.0f mm (window %.0f..%.0f mm)" % (
                1000 * s_star, 1000 * s0, 1000 * s1)
        elif s_att + ext + cfg.stop_margin > cfg.length:
            rec["outcome"] = "MISSED"
            rec["detail"] = "cube would reach the end stop before the gripper is closed (s(attach) = %.0f mm)" % (1000 * s_att)
        if rec["outcome"] == "MISSED":
            log("  " + rec["detail"])
            P.show(info["img"], lines + ["MISSED " + rec["detail"][:80]], info["dets"])
            R._save(idx)
            return rec
        P._ik_targets = [
            ("pregrasp@hover", plan["T_pre"], q_start, plan["q_pre"]),
            ("grasp@t*", plan["T_grasp_end"], plan["segs"][1].Q[-1], plan["segs"][2].Q[-1]),
            ("lift_end", plan["T_lift_end"], plan["segs"][3].Q[-1], plan["q_lift_end"]),
            ("place_pre", chosen_place["T_pre"], plan["q_lift_end"], chosen_place["q_pre"]),
            ("place", chosen_place["T_place"], chosen_place["q_pre"], chosen_place["q_place"])]
        P.phase = "plan"
        g_closed = P.close_ang
        line_pdown = P.plan_line("place_descend", chosen_place["q_pre"], chosen_place["q_place"], g_closed)
        line_pup = P.plan_line("place_retreat", chosen_place["q_place"], chosen_place["q_pre"], 0.0)
        for sg in (line_pdown, line_pup):
            if sg.info.get("fallback"):
                log("  [cartesian fallback] %s -> joint quintic: %s" % (sg.name, sg.info["fallback"]))
        seg_join, seg_h, seg_d, seg_g, seg_l = plan["segs"]

        # ---- 4) execute
        w.activate_cube(A)
        others0 = {n: w.cube_pose_true(n)[:2, 3].copy() for n in w.cubes if n != A}
        P.phase = "intercept"
        P.play(seg_join)
        P.phase = "track"
        P.play(seg_h)
        P.play(seg_d)
        # true tool-vs-cube error at grasp contact (SCORING ONLY)
        rec["belt_track_err_mm"] = 1000 * float(np.linalg.norm(w.cube_pose_true(A)[:2, 3] - w.tcp_T()[:2, 3]))
        P.play(seg_g)
        T_tcp_cube = Tinv(w.tcp_T()) @ w.cube_pose_true(A)
        rec["grasp_offset_mm"] = float(np.linalg.norm(T_tcp_cube[:3, 3]) * 1000)
        off_close = abs(float(T_tcp_cube[1, 3]))              # tool y = closing axis
        rec["belt_closing_off_mm"] = 1000 * off_close
        s_c_true = cfg.from_world(*w.cube_pose_true(A)[:2, 3])[0]
        x_lines.append("GRASP (TRUE, scoring only): tool-vs-cube xy error at contact %.2f mm; after closing offset %.2f mm (closing axis %.2f mm, limit %.1f mm); "
                       "true cube s = %.1f mm (predicted %.1f mm)" % (rec["belt_track_err_mm"], rec["grasp_offset_mm"], 1000 * off_close,
                                                                       1000 * BELT["attach_closing_tol_m"], 1000 * s_c_true, 1000 * s_att))
        log("  grasp: tool-vs-cube xy %.2f mm at contact, offset %.2f mm after closing (closing axis %.2f mm)" % (
            rec["belt_track_err_mm"], rec["grasp_offset_mm"], 1000 * off_close))
        w.attach_cube()
        miss = off_close > BELT["attach_closing_tol_m"]
        P.phase = "lift"
        P.play(seg_l)
        P.phase = "place"
        P.move(chosen_place["q_pre"], "transfer")
        P.play(line_pdown)
        w.detach_cube()
        P.gripper_to(0.0, "open_gripper")
        P.phase = "retreat"
        P.play(line_pup)
        P.move(P.q_ready, "to_ready_2")
        P.move(cfg.park_q, "to_belt_park_end")
        for _ in range(TM["settle_steps"]):
            w.p.stepSimulation()

        # ---- 5) score (TRUE poses: scoring only)
        Tf = w.cube_pose_true(A)
        up = float(Tf[2, 2])
        rec["upright"] = bool(up >= TM["upright_cos"])
        tol = 1000 * float(P.ws["place_tol_m"])
        perr = 1000 * math.hypot(Tf[0, 3] - tgt["x"], Tf[1, 3] - tgt["y"])
        yerr = math.degrees(wrap90(cube_yaw(Tf) - tgt["yaw"]))
        ok = perr < tol and rec["upright"] and not miss
        rec["outcome"] = "SUCCESS" if ok else ("GRASP_MISS" if miss else "PLACE_FAIL")
        if miss:
            rec["detail"] += "closing-axis offset %.1f mm > %.1f mm (fingers would miss the cube); " % (
                1000 * off_close, 1000 * BELT["attach_closing_tol_m"])
        R.rec[A] = {"x": tgt["x"], "y": tgt["y"], "z": R.table_z + edge / 2, "yaw": tgt["yaw"],
                    "src": "last commanded place pose (zone %d)" % task.zone, "zone": task.zone}
        R.support[A] = None
        rec.update(place_err_mm=perr, place_yaw_err_deg=yerr)
        for n, p0 in others0.items():
            d = float(np.linalg.norm(w.cube_pose_true(n)[:2, 3] - p0)) * 1000
            if d > 2.0:
                log("  note: %s cube moved %.1f mm during this task" % (names[n], d))
                rec["detail"] += "%s moved %.1f mm; " % (names[n], d)
        log("  place err %.1f mm (tol %.0f), upright=%s -> %s  [%.1fs]" % (perr, tol, rec["upright"], rec["outcome"], time.time() - t0))
        P.show(info["img"], lines + ["chosen %s | grasp offset %.1f mm | err %.1f mm | %s" % (
            c.candidate_id, rec["grasp_offset_mm"], perr, rec["outcome"])], info["dets"])
        R._save(idx)
        if a.gui:
            P.show(P.grab(), [rec["outcome"] + " - %s, err %.1f mm" % (task.describe(names), perr)])
        return rec


def lim_dt(w):
    return w.limits.dt
