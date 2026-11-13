"""Per-run kinematics / dynamics validation log (TXT + CSV).  SIMULATION ONLY.  Independent of any GUI.

    kin = RunLog(aux_world, mode="headless", seed=0, run=1, t_base=world.sim_t)
    world.on_segment = kin.add_segment            # every played trajectory.Segment is recorded
    ...
    txt_path, csv_path = kin.write(out_dir, meta)   # computes FK validation + dynamics after the run

Contents: header, robot model, IK of the chosen candidate, FK validation (PyBullet link_tcp vs an INDEPENDENT numpy
FK built from the URDF joint origins), trajectory equations / coefficients / peaks vs limits, dynamics, summary.
Dynamics are MODEL values computed from the URDF inertials (the sim itself is kinematic): inverse dynamics via
PyBullet calculateInverseDynamics + a point-mass payload term. They are NOT measured torques.
"""
import csv
import math
import os
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
URDF = os.path.join(HERE, "urdf", "xarm5_gripper.urdf")
LABEL = "SIMULATION ONLY - kinematic sim; dynamics below are URDF MODEL values, not measurements"
G_VEC = np.array([0.0, 0.0, -9.81])
_GIT = []


def git_short():
    if not _GIT:
        try:
            r = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True, text=True,
                               timeout=3)
            _GIT.append(r.stdout.strip() if r.returncode == 0 and r.stdout.strip() else "unknown")
        except Exception:
            _GIT.append("unknown")
    return _GIT[0]


# ----------------------------------------------------------------------------- independent FK (numpy only)
def rpy_to_R(r, p, y):
    """URDF convention: R = Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]])
    return Rz @ Ry @ Rx


def axis_R(axis, a):
    k = np.asarray(axis, float)
    k = k / np.linalg.norm(k)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


def rot_angle(Ra, Rb):
    """Angle [rad] of Ra^T Rb, accurate near 0 (atan2 form)."""
    R = Ra.T @ Rb
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) * 0.5
    return math.atan2(float(np.linalg.norm(v)), (float(np.trace(R)) - 1) * 0.5)


def rpy_of(R):
    """roll, pitch, yaw [rad] (same atan2 form as the GUI kinematics panel)."""
    return (math.atan2(R[2, 1], R[2, 2]), math.atan2(-R[2, 0], math.hypot(R[2, 1], R[2, 2])),
            math.atan2(R[1, 0], R[0, 0]))


def _f3(s, default="0 0 0"):
    return [float(v) for v in (s if s is not None else default).split()]


def parse_urdf(path=URDF):
    """-> (joints dict name -> info, links dict name -> {mass, com}) from the URDF XML."""
    root = ET.parse(path).getroot()
    joints, links = {}, {}
    for j in root.findall("joint"):
        o, ax, lim = j.find("origin"), j.find("axis"), j.find("limit")
        joints[j.get("name")] = {
            "type": j.get("type"), "parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
            "xyz": _f3(o.get("xyz") if o is not None else None), "rpy": _f3(o.get("rpy") if o is not None else None),
            "axis": _f3(ax.get("xyz") if ax is not None else None, "1 0 0"),
            "lower": float(lim.get("lower")) if lim is not None and lim.get("lower") else None,
            "upper": float(lim.get("upper")) if lim is not None and lim.get("upper") else None,
            "velocity": float(lim.get("velocity")) if lim is not None and lim.get("velocity") else None,
            "effort": float(lim.get("effort")) if lim is not None and lim.get("effort") else None}
    for l in root.findall("link"):
        i = l.find("inertial")
        links[l.get("name")] = None if i is None else {
            "mass": float(i.find("mass").get("value")), "com": _f3(i.find("origin").get("xyz"))}
    return joints, links


class IndepFK:
    """link_base -> link_tcp forward kinematics from the URDF joint origins + axis rotations (numpy only)."""

    def __init__(self, urdf=URDF, base_pos=(0.0, 0.0, 0.0), tip="link_tcp", arm=("joint1", "joint2", "joint3",
                                                                                  "joint4", "joint5")):
        self.joints, self.links = parse_urdf(urdf)
        by_child = {j["child"]: n for n, j in self.joints.items()}
        chain, link = [], tip
        while link in by_child:
            jn = by_child[link]
            chain.append(jn)
            link = self.joints[jn]["parent"]
        self.root = link
        self.chain = chain[::-1]
        self.arm = list(arm)
        self.T_root = np.eye(4)
        self.T_root[:3, 3] = base_pos
        self._origin = {n: self._T(self.joints[n]) for n in self.chain}

    @staticmethod
    def _T(j):
        T = np.eye(4)
        T[:3, :3] = rpy_to_R(*j["rpy"])
        T[:3, 3] = j["xyz"]
        return T

    def fk(self, q5):
        T = self.T_root.copy()
        k = 0
        for n in self.chain:
            T = T @ self._origin[n]
            j = self.joints[n]
            if j["type"] in ("revolute", "continuous"):
                R = np.eye(4)
                R[:3, :3] = axis_R(j["axis"], float(q5[self.arm.index(n)]))
                T = T @ R
                k += 1
        return T


# ----------------------------------------------------------------------------- the run log
def _f(v, fmt="%.3f"):
    return "-" if v is None or v == "" else fmt % v


class RunLog:
    def __init__(self, aux, mode, seed, run, t_base=0.0, fk=None):
        """aux: SimWorld used for FK / dynamics queries (hidden aux world in the GUI, the world itself headless)."""
        self.aux, self.mode, self.seed, self.run = aux, mode, seed, run
        self.t_base = float(t_base)
        self.fk = fk or IndepFK(base_pos=(0.0, 0.0, aux.table_top_z))
        self.segs = []
        self.timestamp = time.strftime("%Y%m%d_%H%M%S")

    def add_segment(self, seg):
        self.segs.append(seg)

    # ------------------------------------------------------------ data assembly
    def _rows(self):
        rows = []
        for si, s in enumerate(self.segs):
            t = s.t0 - self.t_base + (np.arange(s.n) + 1) * s.dt
            for i in range(s.n):
                rows.append((t[i], si, s.phase, s.Q[i], s.QD[i], s.QDD[i], s.G[i], int(s.attached)))
        return rows

    def _dynamics(self, rows, cube_mass, pump=None):
        """tau_rb (rigid-body ID, arm joints), tau_pl (payload), fd_err (FD round trip), all per step."""
        w = self.aux
        p, rb, tcp = w.p, w.robot, w.tcp_link
        nm = len(w.movable)
        z = [0.0] * nm
        n = len(rows)
        tau_rb, tau_pl, fd_err = np.zeros((n, 5)), np.zeros((n, 5)), np.zeros(n)
        for i, (t, si, ph, q, qd, qdd, g, att) in enumerate(rows):
            if pump is not None and i % 150 == 0:
                pump()
            q11 = list(map(float, q)) + [float(g)] * (nm - 5)
            qd11 = list(map(float, qd)) + [0.0] * (nm - 5)
            qdd11 = list(map(float, qdd)) + [0.0] * (nm - 5)
            tau = np.array(p.calculateInverseDynamics(rb, q11, qd11, qdd11))
            h = np.array(p.calculateInverseDynamics(rb, q11, qd11, z))
            M = np.array(p.calculateMassMatrix(rb, q11))
            qdd_fd = np.linalg.solve(M, tau - h)
            fd_err[i] = float(np.max(np.abs(qdd_fd - np.array(qdd11))))
            tau_rb[i] = tau[:5]
            if att and cube_mass > 0:
                Jt = np.array(p.calculateJacobian(rb, tcp, [0, 0, 0], q11, z, z)[0])[:, :5]
                eps = 1e-6
                qp = [a + eps * b for a, b in zip(q11, qd11)]
                qm = [a - eps * b for a, b in zip(q11, qd11)]
                Jp = np.array(p.calculateJacobian(rb, tcp, [0, 0, 0], qp, z, z)[0])[:, :5]
                Jm = np.array(p.calculateJacobian(rb, tcp, [0, 0, 0], qm, z, z)[0])[:, :5]
                a_tcp = Jt @ qdd + ((Jp - Jm) / (2 * eps)) @ qd
                tau_pl[i] = Jt.T @ (cube_mass * (a_tcp - G_VEC))
        return tau_rb, tau_pl, fd_err

    def _fk_check(self, rows):
        """PyBullet link_tcp vs independent FK at every segment waypoint and every 10th step."""
        w = self.aux
        ends = set(int(v) for v in np.cumsum([s.n for s in self.segs]) - 1) if rows else set()
        idx = sorted(set(range(0, len(rows), 10)) | ends)
        out = {"n": 0, "max_pos_mm": 0.0, "max_rot_deg": 0.0, "wp": [], "end_pb": 0.0, "end_pb_rot": 0.0}
        for i in idx:
            q = rows[i][3]
            Tn = self.fk.fk(q)
            Tp = w.fk_tcp(q)
            dp, dr = 1000 * float(np.linalg.norm(Tn[:3, 3] - Tp[:3, 3])), math.degrees(rot_angle(Tn[:3, :3], Tp[:3, :3]))
            out["n"] += 1
            out["max_pos_mm"], out["max_rot_deg"] = max(out["max_pos_mm"], dp), max(out["max_rot_deg"], dr)
            if i in ends:
                out["wp"].append((rows[i][1], dp, dr))
        for s in self.segs:        # also the pose of the VISIBLE world after each segment (real played state)
            if s.T_tcp_end_pb is not None:
                Tn = self.fk.fk(s.Q[-1])
                out["end_pb"] = max(out["end_pb"], 1000 * float(np.linalg.norm(Tn[:3, 3] - s.T_tcp_end_pb[:3, 3])))
                out["end_pb_rot"] = max(out["end_pb_rot"], math.degrees(rot_angle(Tn[:3, :3], s.T_tcp_end_pb[:3, :3])))
        return out

    # ------------------------------------------------------------ writing
    def stem(self, meta):
        rec = meta["rec"]
        x, y, yaw = (int(round(1000 * rec["true_x"])), int(round(1000 * rec["true_y"])),
                     int(round(rec["true_yaw_deg"])))
        return "%s_run%02d_x%03d_y%+04d_yaw%+04d_%s" % (self.timestamp, self.run, x, y, yaw,
                                                         str(rec.get("outcome") or "ERROR"))

    def write(self, out_dir, meta, pump=None):
        os.makedirs(out_dir, exist_ok=True)
        stem = os.path.join(out_dir, self.stem(meta))
        rows = self._rows()
        w = self.aux
        cube_mass = float(meta.get("cube_mass", 0.0))
        tau_rb, tau_pl, fd_err = self._dynamics(rows, cube_mass, pump)
        tau = tau_rb + tau_pl
        fkc = self._fk_check(rows)
        # ---- csv
        with open(stem + ".csv", "w", newline="", encoding="utf-8") as f:
            wr = csv.writer(f)
            wr.writerow(["t_s", "segment", "phase"] + ["q%d_rad" % k for k in range(1, 6)] +
                        ["qd%d_rad_s" % k for k in range(1, 6)] + ["qdd%d_rad_s2" % k for k in range(1, 6)] +
                        ["tau%d_Nm" % k for k in range(1, 6)] + ["tcp_x_m", "tcp_y_m", "tcp_z_m", "roll_rad",
                                                                 "pitch_rad", "yaw_rad", "gripper_rad", "attached"])
            for i, (t, si, ph, q, qd, qdd, g, att) in enumerate(rows):
                T = self.fk.fk(q)
                rpy = rpy_of(T[:3, :3])
                wr.writerow(["%.6f" % t, self.segs[si].name, ph] + ["%.9g" % v for v in q] +
                            ["%.9g" % v for v in qd] + ["%.9g" % v for v in qdd] + ["%.9g" % v for v in tau[i]] +
                            ["%.9g" % v for v in T[:3, 3]] + ["%.9g" % v for v in rpy] + ["%.9g" % g, att])
        txt = self._txt(meta, rows, tau_rb, tau_pl, fd_err, fkc)
        with open(stem + ".txt", "w", encoding="utf-8") as f:
            f.write(txt)
        return stem + ".txt", stem + ".csv"

    # ------------------------------------------------------------ text
    def _txt(self, meta, rows, tau_rb, tau_pl, fd_err, fkc):
        w, fk = self.aux, self.fk
        rec = meta["rec"]
        lim = w.limits
        L = []
        A = L.append
        hr = "=" * 100

        def sec(t):
            A("")
            A(hr)
            A(t)
            A(hr)
        # ---- a) header
        A(hr)
        A("XarmAI per-run kinematics validation log   |   " + LABEL)
        A("SIMULATION ONLY | camera = C270 model + PLACEHOLDER noise | grasp = kinematic attach | gripper = stand-in | "
          "PLACEHOLDER limits")
        A(hr)
        A("timestamp        : %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
        A("run id           : %02d     mode: %s     seed: %s     git: %s" % (self.run, self.mode, self.seed, git_short()))
        A("calibration      : source = %s, timestamp = %s" % (meta.get("calib_source", "?"), meta.get("calib_timestamp", "?")))
        A("cube TRUE pose   : x %.1f mm  y %.1f mm  yaw %.2f deg   (ground truth, SCORING ONLY)" % (
            1000 * rec["true_x"], 1000 * rec["true_y"], rec["true_yaw_deg"]))
        if rec.get("est_x", "") != "":
            A("cube ESTIMATED   : x %.1f mm  y %.1f mm  yaw %.2f deg   (from camera + calibration)" % (
                1000 * rec["est_x"], 1000 * rec["est_y"], rec["est_yaw_deg"]))
            A("estimate error   : position %.2f mm   yaw %.2f deg (mod 90)" % (rec["pos_err_mm"], abs(rec["yaw_err_deg"])))
        else:
            A("cube ESTIMATED   : - (perception failed)")
        A("tags seen        : %s" % (rec.get("tags") or "none"))
        A("chosen candidate : %s   (yaw %s deg, rank %s, feasible %s)" % (
            rec.get("chosen_id", "-"), _f(rec.get("chosen_yaw_deg"), "%.0f"), rec.get("chosen_rank", "-") or "-",
            rec.get("feasible", "-") or "-"))
        A("outcome          : %s     place error: %s mm   place yaw error: %s deg   upright: %s" % (
            rec.get("outcome"), _f(rec.get("place_err_mm"), "%.2f"), _f(rec.get("place_yaw_err_deg"), "%.2f"),
            rec.get("upright", "-")))
        # ---- b) robot model
        sec("B. ROBOT MODEL  (xarm5_gripper.urdf; pos [rad], vel [rad/s], effort [N m]; PB = PyBullet getJointInfo)")
        A("%-26s %-9s %-10s %-34s %-24s %-22s %s" % ("joint", "type", "axis", "origin xyz [m]", "origin rpy [rad]",
                                                  "limit lo..hi / vel / effort", "PB match"))
        pbj = {w.p.getJointInfo(w.robot, i)[1].decode(): w.p.getJointInfo(w.robot, i) for i in range(w.p.getNumJoints(w.robot))}
        for n, j in fk.joints.items():
            lim_s = "-"
            match = "-"
            if j["type"] == "revolute":
                lim_s = "%.4f..%.4f / %.2f / %.1f" % (j["lower"], j["upper"], j["velocity"], j["effort"])
                i = pbj[n]
                match = "yes" if (abs(i[8] - j["lower"]) < 1e-6 and abs(i[9] - j["upper"]) < 1e-6 and
                                  abs(i[11] - j["velocity"]) < 1e-6 and abs(i[10] - j["effort"]) < 1e-6) else "NO"
            A("%-26s %-9s %-10s %-34s %-24s %-22s %s" % (
                "%s (%s->%s)" % (n, j["parent"], j["child"]), j["type"], "%g %g %g" % tuple(j["axis"]) if j["type"] != "fixed" else "-",
                "%.5f %.5f %.5f" % tuple(j["xyz"]), "%.5f %.5f %.5f" % tuple(j["rpy"]), lim_s, match))
        tcp = fk.joints["joint_tcp"]
        A("TCP offset (joint_tcp, xarm_gripper_base_link -> link_tcp): xyz = %.4f %.4f %.4f m, rpy = 0" % tuple(tcp["xyz"]))
        A("link masses [kg] (inertials from URDF; link_eef / link_tcp = 1e-6 kg placeholders): " + ", ".join(
            "%s %.4g" % (n, v["mass"]) for n, v in fk.links.items() if v))
        A("sim limits used for timing (PLACEHOLDER): vmax = URDF vel x scale = %s rad/s, amax = %s rad/s^2, "
          "t_min %.2f s, gripper %.2f s, Cartesian peak speed %.3f m/s" % (
              np.round(lim.vmax, 4).tolist(), np.round(lim.amax, 4).tolist(), lim.t_min, lim.grip_time, lim.cart_v))
        # ---- c) IK
        sec("C. INVERSE KINEMATICS  (target pose in link_base; q solution in deg; errors = independent FK(q) vs target)")
        if not meta.get("ik"):
            A("(no IK targets: run ended before a candidate was chosen)")
        for r in meta.get("ik", []):
            T = r["T"]
            Tf = fk.fk(r["q"])
            pe, re = 1000 * float(np.linalg.norm(Tf[:3, 3] - T[:3, 3])), math.degrees(rot_angle(Tf[:3, :3], T[:3, :3]))
            r["_pe"], r["_re"] = pe, re
            rpy = np.degrees(rpy_of(T[:3, :3]))
            A("%-10s target xyz = (%.2f, %.2f, %.2f) mm   rpy = (%.2f, %.2f, %.2f) deg" % (
                r["name"], *(1000 * T[:3, 3]), *rpy))
            A("           seed q   = %s deg" % np.round(np.degrees(r["seed"]), 3).tolist())
            A("           solution = %s deg" % np.round(np.degrees(r["q"]), 3).tolist())
            A("           FK(q) error: pos %.4f mm, rot %.4f deg   | IK tries: %s | re-solve identical: %s" % (
                pe, re, r.get("tries", "-"), r.get("same", "-")))
        A("")
        A("All candidates (IK status of pregrasp -> grasp -> lift; ok = reachable inside joint limits):")
        for c in meta.get("cands", []):
            A("   %-14s yaw %7.1f deg   %-12s %s" % (c[0], c[3], c[1], ("rejected: " + c[2]) if c[2] else ""))
        # ---- d) FK validation
        sec("D. FORWARD KINEMATICS VALIDATION  (PyBullet link_tcp vs independent numpy FK from URDF origins + axes)")
        A("FK chain: " + " -> ".join(fk.chain))
        A("checked %d poses (every segment waypoint + every 10th step)" % fkc["n"])
        A("max position difference : %.3e mm" % fkc["max_pos_mm"])
        A("max rotation difference : %.3e deg" % fkc["max_rot_deg"])
        A("visible-world tcp pose after each segment vs numpy FK: max %.3e mm / %.3e deg" % (fkc["end_pb"], fkc["end_pb_rot"]))
        A("per waypoint (segment end): " + ", ".join("%s: %.1e mm/%.1e deg" % (self.segs[si].name, a, b)
                                                      for si, a, b in fkc["wp"]))
        # ---- e) trajectory
        sec("E. TRAJECTORY SEGMENTS  (sim dt = 1/240 s; sim time = steps x dt, independent of the GUI speed slider)")
        A("quintic time scaling: s(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5, tau = t/T, t in [0,T]")
        A("  q_j(t)   = q0_j + D_j*(10 tau^3 - 15 tau^4 + 6 tau^5),   D_j = q1_j - q0_j")
        A("  qd_j(t)  = D_j/T*(30 tau^2 - 60 tau^3 + 30 tau^4)")
        A("  qdd_j(t) = D_j/T^2*(60 tau - 180 tau^2 + 120 tau^3)")
        A("  expanded: q = q0 + a3 t^3 + a4 t^4 + a5 t^5,  a3 = 10 D/T^3, a4 = -15 D/T^4, a5 = 6 D/T^5")
        A("  joint segment time: T = max_j max(1.875|D_j|/vmax_j, sqrt(5.7735|D_j|/amax_j)), >= t_min, rounded up to whole steps")
        exceeded = []
        for si, s in enumerate(self.segs):
            A("")
            A("[%d] %s   phase=%s   type=%s   start t=%.4f s   T=%.4f s   steps=%d   cube attached=%s" % (
                si, s.name, s.phase, s.kind, s.t0 - self.t_base, s.T, s.n, s.attached))
            if s.info.get("fallback"):
                A("    FALLBACK to joint quintic (Cartesian line not possible): %s" % s.info["fallback"])
            if s.kind == "gripper-quintic":
                D = s.g1 - s.g0
                A("    g(t) = g0 + Dg*s(tau): g0 = %.6f rad, Dg = %.6f rad ; a3 = %.6g, a4 = %.6g, a5 = %.6g (rad/s^n)" % (
                    s.g0, D, 10 * D / s.T ** 3, -15 * D / s.T ** 4, 6 * D / s.T ** 5))
                A("    arm joints held at q = %s deg" % np.round(np.degrees(s.q0), 3).tolist())
                continue
            D = s.q1 - s.q0
            if s.kind == "cartesian-quintic":
                p0, p1 = s.info["p0"], s.info["p1"]
                dp = p1 - p0
                A("    TCP: p(t) = p0 + (p1 - p0)*(10 tau^3 - 15 tau^4 + 6 tau^5), orientation R0 -> R1 along the same s "
                  "(constant if R0 == R1; R0-R1 angle %.4f deg)" % math.degrees(rot_angle(s.info["R0"], s.info["R1"])))
                A("    p0 = (%.4f, %.4f, %.4f) mm   p1 = (%.4f, %.4f, %.4f) mm   length = %.3f mm   peak TCP speed = %.4f m/s" % (
                    *(1000 * p0), *(1000 * p1), 1000 * s.info["length_m"], K_V_PEAK * s.info["length_m"] / s.T))
                A("    p coefficients a3 = 10 dp/T^3, a4 = -15 dp/T^4, a5 = 6 dp/T^5 [m/s^n] per axis x,y,z:")
                A("      a3 = %s   a4 = %s   a5 = %s" % tuple(
                    np.array2string(c * dp / s.T ** k, precision=5, separator=", ") for c, k in
                    ((10, 3), (-15, 4), (6, 5))))
                A("    q(t) comes from per-step IK (Newton polish, seed = previous q), NOT from the polynomial below; "
                  "qd, qdd = central differences of the sampled q. worst IK pos err %.2e mm, time-scaling attempts %d" % (
                      1000 * s.info.get("ik_worst_pos_err_m", 0.0), s.info.get("attempts", 1)))
                A("    (for reference, the joint polynomial between the same waypoints would use:)")
            A("    joint   q0 [rad]      D [rad]       a3 [rad/s^3]   a4 [rad/s^4]   a5 [rad/s^5]")
            for j in range(5):
                A("    J%d  %12.6f  %12.6f  %13.6g  %13.6g  %13.6g" % (j + 1, s.q0[j], D[j], 10 * D[j] / s.T ** 3,
                                                                    -15 * D[j] / s.T ** 4, 6 * D[j] / s.T ** 5))
            pv, pa = s.peak()
            ok = (pv <= lim.vmax * (1 + 1e-9)) & (pa <= lim.amax * (1 + 1e-9))
            A("    peak |qd| [rad/s]   : %s   (limit %.3f)   peak |qdd| [rad/s^2]: %s   (limit %.3f)  -> %s" % (
                np.round(pv, 4).tolist(), lim.vmax[0], np.round(pa, 4).tolist(), lim.amax[0],
                "OK" if ok.all() else "EXCEEDED"))
            if not ok.all():
                exceeded.append(s.name)
        # ---- f) dynamics
        sec("F. DYNAMICS  (MODEL values from URDF inertials; the sim is kinematic - NOT measured torques)")
        A("inverse dynamics: tau = M(q) qdd + C(q,qd) qd + g(q) via PyBullet calculateInverseDynamics, all %d movable joints "
          "(gripper joints: zero vel/acc, held at the gripper angle)" % len(w.movable))
        A("payload: while the cube is attached, tau_payload = J_v^T * m_cube * (a_tcp - g_vec), m_cube = %.4f kg, g_vec = "
          "(0, 0, -9.81), a_tcp = J_v qdd + Jdot qd.  POINT-MASS approximation at the TCP (no cube inertia, no offset)." % meta.get("cube_mass", 0.0))
        eff = np.array([fk.joints[n]["effort"] for n in fk.arm])
        tau = tau_rb + tau_pl
        pk = np.abs(tau).max(0) if len(tau) else np.zeros(5)
        pk_rb = np.abs(tau_rb).max(0) if len(tau) else np.zeros(5)
        A("")
        A("joint   peak|tau| total [N m]   (rigid body only)   effort limit [N m]   ratio   status")
        over = []
        for j in range(5):
            st = "OK" if pk[j] <= eff[j] else "EXCEEDED"
            if st != "OK":
                over.append("J%d" % (j + 1))
            A("J%d      %14.3f        %10.3f          %10.1f        %5.2f   %s" % (j + 1, pk[j], pk_rb[j], eff[j], pk[j] / eff[j], st))
        A("")
        A("peak |tau| [N m] per segment (J1..J5):")
        i0 = 0
        for si, s in enumerate(self.segs):
            sl = slice(i0, i0 + s.n)
            A("   [%d] %-16s %s" % (si, s.name, np.round(np.abs(tau[sl]).max(0), 3).tolist()))
            i0 += s.n
        fd = float(fd_err.max()) if len(fd_err) else 0.0
        A("")
        A("forward-dynamics round trip: qdd_fd = M(q)^-1 (tau - h(q,qd)),  h = ID(q,qd,0), M = calculateMassMatrix (rigid body, no payload)")
        A("max |qdd_fd - qdd| over all steps and all %d joints = %.3e rad/s^2" % (len(w.movable), fd))
        # ---- g) summary
        sec("G. SUMMARY")
        ttot = rows[-1][0] if rows else 0.0
        max_ik = max([r.get("_pe", 0.0) for r in meta.get("ik", [])], default=0.0)
        max_ik_r = max([r.get("_re", 0.0) for r in meta.get("ik", [])], default=0.0)
        fb = [s.name for s in self.segs if s.info.get("fallback")]
        A("total sim time        : %.3f s (%d steps, %d segments)" % (ttot, len(rows), len(self.segs)))
        A("max FK diff           : %.3e mm / %.3e deg (PyBullet vs independent FK)" % (fkc["max_pos_mm"], fkc["max_rot_deg"]))
        A("max IK error          : %.4f mm / %.4f deg (chosen candidate + place targets, independent FK)" % (max_ik, max_ik_r))
        A("max FD round-trip err : %.3e rad/s^2" % fd)
        A("velocity/acc limits   : %s" % ("all segments within limits" if not exceeded else "EXCEEDED in " + ", ".join(exceeded)))
        A("torque vs effort      : %s" % ("all joints within URDF effort limits (model values)" if not over else "EXCEEDED on " + ", ".join(over)))
        A("Cartesian fallbacks   : %s" % ("none" if not fb else ", ".join(fb)))
        A("any limit exceeded    : %s" % ("YES" if (exceeded or over) else "no"))
        self.summary = {"time_s": ttot, "fk_mm": fkc["max_pos_mm"], "fd": fd, "exceeded": exceeded, "over": over,
                        "fallbacks": fb, "peak_tau": pk, "max_ik_mm": max_ik}
        return "\n".join(L) + "\n"


K_V_PEAK = 1.875
