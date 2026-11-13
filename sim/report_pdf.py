"""PDF report generator for the simulated pick-and-place runs.  SIMULATION ONLY.

    from sim.report_pdf import build_report
    path = build_report(log_paths, "results/reports")      # log_paths = kin_log TXT (or CSV) files

Reads the per-run TXT + CSV written by sim/kin_log.py. matplotlib PdfPages only. A log that fails to parse is
skipped and listed on the cover page; one bad file never aborts the report.
"""
import csv
import datetime
import json
import math
import os
import re
import subprocess
import textwrap
import traceback

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
FOOTER = "SIMULATION ONLY — model values, PLACEHOLDER limits"
PAGE = (11.69, 8.27)  # A4 landscape, inches
MAX_PTS = 1500
JCOL = ["#0072B2", "#D55E00", "#009E73", "#CC79A7", "#E69F00"]   # Okabe-Ito
INK, MUTED, GRID = "#1f2933", "#6b7785", "#d9dee4"
OUT_COL = {"SUCCESS": "#009E73"}
FAIL_COL = "#D55E00"

plt.rcParams.update({
    "font.size": 9, "axes.titlesize": 10.5, "axes.titleweight": "bold", "axes.labelsize": 9,
    "axes.edgecolor": MUTED, "axes.labelcolor": INK, "text.color": INK, "xtick.color": INK, "ytick.color": INK,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "axes.spines.top": False,
    "axes.spines.right": False, "legend.frameon": False, "legend.fontsize": 8, "pdf.fonttype": 42,
})


# ----------------------------------------------------------------------------------------------- parsing
def _num(s, default=None):
    try:
        return float(s)
    except (TypeError, ValueError):
        return default


def _grab(pat, text, group=1, flags=0):
    m = re.search(pat, text, flags)
    return m.group(group) if m else None


def _floats(s):
    return [float(v) for v in re.findall(r"[-+]?\d+\.?\d*(?:[eE][-+]?\d+)?", s or "")]


def parse_txt(path):
    """kin_log TXT -> dict. Raises on unusable file."""
    with open(path, encoding="utf-8", errors="replace") as f:
        txt = f.read()
    if "per-run kinematics validation log" not in txt:
        raise ValueError("not a kin_log TXT")
    L = txt.splitlines()
    d = {"path": path, "stem": os.path.splitext(os.path.basename(path))[0], "txt": txt}
    d["timestamp"] = _grab(r"^timestamp\s*:\s*(.+)$", txt, flags=re.M) or "?"
    m = re.search(r"run id\s*:\s*(\d+)\s+mode:\s*(\S+)\s+seed:\s*(\S+)\s+git:\s*(\S+)", txt)
    d["run"], d["mode"], d["seed"], d["git"] = (int(m.group(1)), m.group(2), m.group(3), m.group(4)) if m else (0, "?", "?", "?")
    m = re.search(r"cube TRUE pose\s*:\s*x ([-\d.]+) mm\s+y ([-\d.]+) mm\s+yaw ([-\d.]+) deg", txt)
    d["true"] = tuple(float(v) for v in m.groups()) if m else None
    m = re.search(r"cube ESTIMATED\s*:\s*x ([-\d.]+) mm\s+y ([-\d.]+) mm\s+yaw ([-\d.]+) deg", txt)
    d["est"] = tuple(float(v) for v in m.groups()) if m else None
    m = re.search(r"estimate error\s*:\s*position ([-\d.]+) mm\s+yaw ([-\d.]+) deg", txt)
    d["pos_err"] = float(m.group(1)) if m else None
    d["yaw_err"] = float(m.group(2)) if m else None
    m = re.search(r"outcome\s*:\s*(\S+)\s+place error:\s*(\S+) mm\s+place yaw error:\s*(\S+) deg\s+upright:\s*(\S+)", txt)
    d["outcome"] = m.group(1) if m else "UNKNOWN"
    d["place_err"] = _num(m.group(2)) if m else None
    d["place_yaw_err"] = _num(m.group(3)) if m else None
    d["upright"] = m.group(4) if m else "-"
    d["calib"] = _grab(r"^calibration\s*:\s*(.+)$", txt, flags=re.M) or "?"
    # limits (PLACEHOLDER)
    m = re.search(r"vmax = .*?= \[([^\]]*)\] rad/s, amax = \[([^\]]*)\] rad/s\^2", txt)
    d["vmax"] = _floats(m.group(1)) if m else None
    d["amax"] = _floats(m.group(2)) if m else None
    d["effort"] = []
    for m in re.finditer(r"^J(\d)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+(\w+)\s*$", txt, re.M):
        if len(d["effort"]) < 5:
            d["effort"].append(float(m.group(4)))
    d["qlim"] = []
    for k in range(1, 6):
        m = re.search(r"^joint%d \(.*?\)\s+revolute\s+\S+ \S+ \S+\s+.*?\s+(-?[\d.]+)\.\.(-?[\d.]+) /" % k, txt, re.M)
        d["qlim"].append((float(m.group(1)), float(m.group(2))) if m else None)
    # segments
    segs, cur = [], None
    in_e = False
    for ln in L:
        if ln.startswith("E. TRAJECTORY SEGMENTS"):
            in_e = True
            continue
        if ln.startswith("F. DYNAMICS"):
            in_e = False
        if not in_e:
            continue
        m = re.match(r"^\[(\d+)\] (\S+)\s+phase=(\S+)\s+type=(\S+)\s+start t=([-\d.]+) s\s+T=([-\d.]+) s\s+steps=(\d+)"
                     r"\s+cube attached=(\S+)", ln)
        if m:
            cur = {"i": int(m.group(1)), "name": m.group(2), "phase": m.group(3), "kind": m.group(4),
                   "t0": float(m.group(5)), "T": float(m.group(6)), "steps": int(m.group(7)),
                   "attached": m.group(8), "lines": [], "peakv": None, "peaka": None}
            segs.append(cur)
        elif cur is not None and ln.strip():
            if ln.startswith("="):
                continue
            cur["lines"].append(ln.rstrip())
            m = re.search(r"peak \|qd\| \[rad/s\]\s*:\s*\[([^\]]*)\].*?peak \|qdd\| \[rad/s\^2\]:\s*\[([^\]]*)\]", ln)
            if m:
                cur["peakv"], cur["peaka"] = _floats(m.group(1)), _floats(m.group(2))
    d["segs"] = segs
    # peak torque per segment
    pt = {}
    for m in re.finditer(r"^\s+\[(\d+)\] (\S+)\s+\[([^\]]*)\]\s*$", txt, re.M):
        pt[int(m.group(1))] = _floats(m.group(3))
    d["seg_tau"] = pt
    # summary block
    sm = txt.split("G. SUMMARY")[-1] if "G. SUMMARY" in txt else ""
    d["summary_lines"] = [l.rstrip() for l in sm.splitlines() if ":" in l and not l.startswith("=")]
    d["max_fk"] = _grab(r"max FK diff\s*:\s*(.+)", sm)
    d["max_ik"] = _grab(r"max IK error\s*:\s*(.+)", sm)
    d["max_fd"] = _grab(r"max FD round-trip err\s*:\s*(.+)", sm)
    d["lim_vel"] = _grab(r"velocity/acc limits\s*:\s*(.+)", sm)
    d["lim_tau"] = _grab(r"torque vs effort\s*:\s*(.+)", sm)
    d["any_exceeded"] = _grab(r"any limit exceeded\s*:\s*(.+)", sm)
    d["total_time"] = _grab(r"total sim time\s*:\s*(.+)", sm)
    d["tau_table"] = [l.rstrip() for l in L if re.match(r"^J\d\s+[-\d.]+\s+[-\d.]+\s+[-\d.]+\s+[-\d.]+\s+\w+\s*$", l)][:5]
    if not segs:
        raise ValueError("no trajectory segments in TXT")
    return d


def load_csv(path):
    """kin_log CSV -> dict of numpy arrays (downsample not applied here)."""
    with open(path, newline="", encoding="utf-8") as f:
        rd = csv.reader(f)
        head = next(rd)
        rows = list(rd)
    if not rows:
        raise ValueError("empty CSV")
    col = {h: i for i, h in enumerate(head)}

    def arr(names):
        return np.array([[float(r[col[n]]) for n in names] for r in rows])
    out = {"t": arr(["t_s"])[:, 0],
           "seg": [r[col["segment"]] for r in rows],
           "q": arr(["q%d_rad" % k for k in range(1, 6)]),
           "qd": arr(["qd%d_rad_s" % k for k in range(1, 6)]),
           "qdd": arr(["qdd%d_rad_s2" % k for k in range(1, 6)]),
           "tau": arr(["tau%d_Nm" % k for k in range(1, 6)]),
           "tcp": arr(["tcp_x_m", "tcp_y_m", "tcp_z_m"]) * 1000.0,
           "grip": arr(["gripper_rad"])[:, 0],
           "att": arr(["attached"])[:, 0]}
    return out


def load_run(path):
    """path (TXT or CSV) -> run dict with 'csv' arrays. Raises on failure."""
    base = os.path.splitext(path)[0]
    d = parse_txt(base + ".txt")
    d["data"] = load_csv(base + ".csv")
    return d


def _read_yaml(path):
    try:
        import yaml
        with open(path, encoding="utf-8") as f:
            return yaml.safe_load(f)
    except Exception:
        return None


def load_workspace():
    ws = _read_yaml(os.path.join(ROOT, "config", "workspace", "workspace.yaml"))
    return ws if isinstance(ws, dict) else None


def load_calib():
    p = os.path.join(ROOT, "results", "sim_calib", "sim_calib.json")
    try:
        with open(p, encoding="utf-8") as f:
            c = json.load(f)
        return {"ts": c.get("timestamp"), "rms": c["intr"]["rms"], "views": c["intr"].get("views"),
                "cam_pos": c["ext"]["cam_pos_mm"], "cam_rot": c["ext"]["cam_rot_deg"],
                "undist": c["intr"].get("undist_mean_px")}
    except Exception:
        return None


def git_short():
    try:
        r = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True, text=True, timeout=3)
        return r.stdout.strip() if r.returncode == 0 and r.stdout.strip() else "unknown"
    except Exception:
        return "unknown"


# ----------------------------------------------------------------------------------------------- page helpers
class _Doc:
    def __init__(self, pdf):
        self.pdf, self.n = pdf, 0

    def fig(self, title=None, sub=None):
        f = plt.figure(figsize=PAGE)
        f.patch.set_facecolor("white")
        if title:
            f.text(0.04, 0.945, title, fontsize=15, fontweight="bold", color=INK, va="center")
        if sub:
            f.text(0.04, 0.915, sub, fontsize=8.5, color=MUTED, va="center")
        return f

    def save(self, f):
        self.n += 1
        f.add_artist(plt.Line2D([0.04, 0.96], [0.04, 0.04], color=GRID, lw=0.8, transform=f.transFigure))
        f.text(0.04, 0.022, FOOTER, fontsize=7.5, color=MUTED, va="center")
        f.text(0.96, 0.022, "Page %d" % self.n, fontsize=7.5, color=MUTED, va="center", ha="right")
        self.pdf.savefig(f)
        if getattr(self, "png_dir", None):
            try:
                f.savefig(os.path.join(self.png_dir, "page_%02d.png" % self.n), dpi=70)
            except Exception:
                pass
        plt.close(f)


def _stride(n, cap=MAX_PTS):
    return max(1, int(math.ceil(n / float(cap))))


def _shade_segments(ax, t, seg, label=False):
    """Alternate-shade segment spans; optionally write names along the top."""
    if not len(t):
        return
    starts = [0] + [i for i in range(1, len(seg)) if seg[i] != seg[i - 1]] + [len(seg)]
    for k in range(len(starts) - 1):
        a, b = starts[k], starts[k + 1] - 1
        t0 = t[a] - (0 if k else 0)
        t1 = t[b]
        if k % 2 == 0:
            ax.axvspan(t0, t1, color="#eef2f6", lw=0, zorder=0)
        if label:
            ax.text((t0 + t1) / 2, 1.01, seg[a], transform=ax.get_xaxis_transform(), rotation=55, fontsize=6.5,
                    ha="left", va="bottom", color=MUTED)


def _mono_text(f, rect, text, fs=6.6, title=None):
    ax = f.add_axes(rect)
    ax.axis("off")
    if title:
        ax.text(0, 1.0, title, fontsize=9.5, fontweight="bold", va="top", transform=ax.transAxes)
    ax.text(0, 0.96 if title else 1.0, text, family="monospace", fontsize=fs, va="top", transform=ax.transAxes,
            linespacing=1.25)
    return ax


# ----------------------------------------------------------------------------------------------- pages
def page_cover(doc, runs, bad, title, calib):
    f = doc.fig()
    f.text(0.06, 0.80, title, fontsize=26, fontweight="bold", color=INK)
    f.text(0.06, 0.755, "xArm 5 cube pick-and-place — per-run kinematics / dynamics validation",
           fontsize=12.5, color=MUTED)
    ok = sum(1 for r in runs if r["outcome"] == "SUCCESS")
    lines = [("Generated", datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")),
             ("Git hash (report tool)", git_short()),
             ("Git hash (logs)", ", ".join(sorted({r["git"] for r in runs})) or "-"),
             ("Runs in report", "%d  (%d success)%s" % (len(runs), ok, ("  |  %d log(s) skipped" % len(bad)) if bad else ""))]
    if runs:
        lines.append(("Run time span", "%s  ..  %s" % (runs[0]["timestamp"], runs[-1]["timestamp"])))
    if calib:
        lines += [("Calibration (sim_calib.json)", "%s" % calib["ts"]),
                  ("Intrinsics RMS reprojection", "%.3f px  (%s views)" % (calib["rms"], calib["views"])),
                  ("T_base_cam error vs truth", "%.2f mm / %.2f deg" % (calib["cam_pos"], calib["cam_rot"]))]
    else:
        lines.append(("Calibration", "sim_calib.json not found"))
    y = 0.66
    for k, v in lines:
        f.text(0.06, y, k, fontsize=10.5, color=MUTED)
        f.text(0.34, y, v, fontsize=10.5, color=INK, fontweight="bold")
        y -= 0.04
    y -= 0.02
    f.text(0.06, y, "Honesty labels", fontsize=11.5, fontweight="bold")
    labels = ["SIMULATION ONLY — nothing here is a hardware measurement.",
              "Camera = C270 model from public spec + PLACEHOLDER noise / blur / distortion.",
              "Grasp = kinematic attach (no contact physics). Gripper = xArm Gripper stand-in (real gripper unidentified).",
              "Velocity / acceleration / effort limits are PLACEHOLDER values (URDF limits x scale).",
              "Dynamics = URDF MODEL values (inverse dynamics from URDF inertials + point-mass payload), not measured torques.",
              "Ground-truth cube pose is used for scoring only; the pipeline works from camera + calibration estimates."]
    for s in labels:
        y -= 0.034
        f.text(0.07, y, "•  " + s, fontsize=9.5)
    if bad:
        y -= 0.05
        f.text(0.06, y, "Skipped logs (parse failed)", fontsize=10.5, fontweight="bold", color=FAIL_COL)
        for p, e in bad[:6]:
            y -= 0.03
            f.text(0.07, y, "%s: %s" % (os.path.basename(p)[:70], str(e)[:80]), fontsize=8, color=FAIL_COL)
    doc.save(f)


def _fmt(v, fmt="%.2f"):
    return "-" if v is None else fmt % v


def page_summary(doc, runs):
    f = doc.fig("Summary", "%d runs" % len(runs))
    n = len(runs)
    ok = [r for r in runs if r["outcome"] == "SUCCESS"]
    pe = np.array([r["pos_err"] for r in runs if r["pos_err"] is not None])
    pl = np.array([r["place_err"] for r in runs if r["place_err"] is not None])
    rate = 100.0 * len(ok) / n if n else 0.0
    kpis = [("Success rate", "%.0f %%" % rate, "%d / %d runs" % (len(ok), n)),
            ("Pose error (mean / max)", "%s / %s mm" % (_fmt(pe.mean() if len(pe) else None), _fmt(pe.max() if len(pe) else None)),
             "cube estimate vs truth"),
            ("Place error (mean / max)", "%s / %s mm" % (_fmt(pl.mean() if len(pl) else None), _fmt(pl.max() if len(pl) else None)),
             "cube centre vs place target"),
            ("Limits exceeded", "%d run(s)" % sum(1 for r in runs if (r.get("any_exceeded") or "").strip().upper().startswith("YES")),
             "velocity / acc / effort (placeholder)")]
    for i, (k, v, s) in enumerate(kpis):
        x = 0.04 + i * 0.235
        f.add_artist(Rectangle((x, 0.76), 0.22, 0.11, transform=f.transFigure, fc="#f5f7fa", ec=GRID, lw=0.8))
        f.text(x + 0.012, 0.848, k, fontsize=8, color=MUTED)
        f.text(x + 0.012, 0.805, v, fontsize=15, fontweight="bold")
        f.text(x + 0.012, 0.775, s, fontsize=7, color=MUTED)
    ax1 = f.add_axes([0.06, 0.36, 0.26, 0.34])
    ax2 = f.add_axes([0.40, 0.36, 0.26, 0.34])
    ax3 = f.add_axes([0.74, 0.36, 0.22, 0.34])
    for ax, data, ttl, col in ((ax1, pe, "Pose error (estimate vs truth)", "#0072B2"),
                               (ax2, pl, "Place error (cube vs target)", "#CC79A7")):
        if len(data):
            ax.hist(data, bins=min(12, max(3, len(data))), color=col, edgecolor="white")
        else:
            ax.text(0.5, 0.5, "no data", ha="center", transform=ax.transAxes, color=MUTED)
        ax.set_title(ttl)
        ax.set_xlabel("mm")
        ax.set_ylabel("runs")
        ax.yaxis.get_major_locator().set_params(integer=True)
    outs = {}
    for r in runs:
        outs[r["outcome"]] = outs.get(r["outcome"], 0) + 1
    names = sorted(outs, key=lambda k: (k != "SUCCESS", k))
    ax3.bar(range(len(names)), [outs[k] for k in names], width=0.5, color=[OUT_COL.get(k, FAIL_COL) for k in names])
    ax3.set_xticks(range(len(names)))
    ax3.set_xticklabels([k[:12] for k in names], rotation=20, ha="right", fontsize=7.5)
    ax3.set_xlim(-0.8, max(len(names) - 0.2, 1.8))
    ax3.set_title("Runs by outcome")
    ax3.set_ylabel("runs")
    ax3.yaxis.get_major_locator().set_params(integer=True)
    # compact table (first rows) - full table follows
    f.text(0.04, 0.30, "Full per-run table on the next page%s." % ("s" if n > 20 else ""), fontsize=9, color=MUTED)
    doc.save(f)
    # full table pages
    per = 20
    for k in range(0, n, per):
        g = doc.fig("Runs table", "runs %d–%d of %d" % (k + 1, min(n, k + per), n))
        _runs_table(g, [0.04, 0.07, 0.92, 0.80], runs[k:k + per], fs=8)
        doc.save(g)


def _runs_table(f, rect, runs, fs=8):
    ax = f.add_axes(rect)
    ax.axis("off")
    cols = ["Timestamp", "Run", "True x,y [mm] / yaw [deg]", "Est x,y [mm] / yaw [deg]", "Pose err [mm]",
            "Place err [mm]", "Outcome"]
    rows, colors = [], []
    for r in runs:
        t, e = r["true"], r["est"]
        rows.append([r["timestamp"], "%02d" % r["run"],
                     "%.1f, %.1f / %.1f" % t if t else "-", "%.1f, %.1f / %.1f" % e if e else "-",
                     _fmt(r["pos_err"]), _fmt(r["place_err"]), r["outcome"]])
        colors.append(OUT_COL.get(r["outcome"], FAIL_COL))
    if not rows:
        return
    tb = ax.table(cellText=rows, colLabels=cols, loc="upper center", cellLoc="center",
                  colWidths=[0.15, 0.05, 0.22, 0.22, 0.10, 0.10, 0.12])
    tb.auto_set_font_size(False)
    tb.set_fontsize(fs)
    tb.scale(1, 1.35 if len(rows) > 10 else 1.7)
    for (i, j), c in tb.get_celld().items():
        c.set_edgecolor(GRID)
        if i == 0:
            c.set_facecolor("#e6ebf0")
            c.set_text_props(fontweight="bold")
        else:
            c.set_facecolor("white" if i % 2 else "#f7f9fb")
            if j == 6:
                c.set_text_props(color=colors[i - 1], fontweight="bold")


def _zone(ax, z, label, col):
    try:
        x, y = z["x_m"], z["y_m"]
        ax.add_patch(Rectangle((x[0] * 1000, y[0] * 1000), (x[1] - x[0]) * 1000, (y[1] - y[0]) * 1000, fc=col,
                               alpha=0.10, ec=col, lw=1.2, ls="--", zorder=1))
        ax.text(x[0] * 1000 + 3, y[1] * 1000 - 4, label, fontsize=8, color=col, va="top")
    except Exception:
        pass


def page_scatter(doc, runs, ws):
    f = doc.fig("Pick zone: true cube positions", "colour = place error [mm]; zones from config/workspace/workspace.yaml "
                "(PLACEHOLDER, unmeasured)" if ws else "workspace.yaml not found: zones not drawn")
    ax = f.add_axes([0.07, 0.10, 0.62, 0.75])
    if ws:
        _zone(ax, ws.get("cube_workspace_box", {}), "PICK zone", "#0072B2")
        _zone(ax, ws.get("place_zone", {}), "PLACE zone", "#D55E00")
        pt = ws.get("place_target")
        if pt:
            ax.plot(pt["x_m"] * 1000, pt["y_m"] * 1000, marker="+", ms=12, mew=2, color="#D55E00")
            ax.annotate("place target", (pt["x_m"] * 1000, pt["y_m"] * 1000), (6, 6), textcoords="offset points",
                        fontsize=8, color="#D55E00")
    ax.plot(0, 0, marker="s", color=INK, ms=7)
    ax.annotate("robot base", (0, 0), (6, -12), textcoords="offset points", fontsize=8)
    pts = [(r["true"][0], r["true"][1], r["place_err"], r["run"], r["outcome"]) for r in runs if r["true"]]
    if pts:
        x = np.array([p[0] for p in pts])
        y = np.array([p[1] for p in pts])
        c = np.array([np.nan if p[2] is None else p[2] for p in pts])
        have = ~np.isnan(c)
        sc = None
        if have.any():
            sc = ax.scatter(x[have], y[have], c=c[have], cmap="viridis_r", s=70, edgecolor="white", lw=0.8, zorder=3)
        if (~have).any():
            ax.scatter(x[~have], y[~have], marker="x", c=FAIL_COL, s=60, zorder=3, label="no place error (failed)")
            ax.legend(loc="lower right")
        for p in pts[:60]:
            ax.annotate("%d" % p[3], (p[0], p[1]), (4, 4), textcoords="offset points", fontsize=6.5, color=MUTED)
        if sc is not None:
            cb = f.colorbar(sc, ax=ax, fraction=0.04, pad=0.02)
            cb.set_label("place error [mm]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x in link_base [mm]")
    ax.set_ylabel("y in link_base [mm]")
    ax.set_title("Top view (+x forward, +y left)")
    ax2 = f.add_axes([0.76, 0.10, 0.20, 0.75])
    ax2.axis("off")
    pl = [r["place_err"] for r in runs if r["place_err"] is not None]
    txt = "Runs plotted: %d\n" % len(pts)
    if pl:
        txt += "Place err mean: %.2f mm\nPlace err max : %.2f mm\n" % (np.mean(pl), np.max(pl))
    if ws and "place_tol_m" in ws:
        txt += "Place tol (placeholder): %.1f mm\n" % (ws["place_tol_m"] * 1000)
    ax2.text(0, 0.98, txt, va="top", fontsize=9, family="monospace")
    doc.save(f)


def _run_title(r):
    t = r["true"]
    return "Run %02d  |  %s  |  %s" % (r["run"], r["timestamp"], r["outcome"]), (
        "cube true x %.1f  y %.1f  yaw %.1f   |   pose err %s mm   place err %s mm   |   mode %s   seed %s   git %s" % (
            (t if t else (0, 0, 0)) + (_fmt(r["pos_err"]), _fmt(r["place_err"]), r["mode"], r["seed"], r["git"])))


def pages_run(doc, r, coeffs=True):
    dat = r["data"]
    n = len(dat["t"])
    s = _stride(n)
    t = dat["t"][::s]
    seg = dat["seg"][::s]
    ttl, sub = _run_title(r)
    # ---------------- page 1: joint plots
    f = doc.fig(ttl, sub)
    gs = f.add_gridspec(2, 2, left=0.07, right=0.97, top=0.78, bottom=0.12, hspace=0.42, wspace=0.17)
    qlim = r["qlim"]
    specs = [("q", "Joint angles q(t)", "rad", None), ("qd", "Joint velocities qd(t)", "rad/s", "vmax"),
             ("qdd", "Joint accelerations qdd(t)", "rad/s²", "amax"), ("tau", "Joint torques tau(t) (model)", "N m", "effort")]
    for k, (key, name, unit, lim) in enumerate(specs):
        ax = f.add_subplot(gs[k // 2, k % 2])
        _shade_segments(ax, t, seg, label=(k < 2))
        Y = dat[key][::s]
        for j in range(5):
            ax.plot(t, Y[:, j], color=JCOL[j], lw=1.1, label="J%d" % (j + 1))
            if key == "q" and qlim[j]:
                for v in qlim[j]:
                    ax.axhline(v, color=JCOL[j], ls="--", lw=0.6, alpha=0.5)
            elif lim and r.get(lim) is not None and len(r[lim]) > j:
                v = r[lim][j]
                if key != "tau":
                    ax.axhline(v, color=JCOL[j], ls="--", lw=0.7, alpha=0.7)
                    ax.axhline(-v, color=JCOL[j], ls="--", lw=0.7, alpha=0.7)
                else:
                    ax.axhline(v, color=JCOL[j], ls="--", lw=0.7, alpha=0.7)
                    ax.axhline(-v, color=JCOL[j], ls="--", lw=0.7, alpha=0.7)
        if key == "q":
            ax.set_ylim(auto=True)
            lo, hi = np.nanmin(Y), np.nanmax(Y)
            pad = 0.15 * max(hi - lo, 0.2)
            ax.set_ylim(lo - pad, hi + pad)    # keep limit lines from squashing the data
        elif key in ("qd", "qdd"):
            lo, hi = np.nanmin(Y), np.nanmax(Y)
            lim_v = r["vmax" if key == "qd" else "amax"]
            top = max(abs(lo), abs(hi), (max(lim_v) if lim_v else 0)) * 1.15
            ax.set_ylim(-top, top)
        else:
            lo, hi = np.nanmin(Y), np.nanmax(Y)
            ef = max(r["effort"]) if r["effort"] else 0
            top = max(abs(lo), abs(hi)) * 1.2
            ax.set_ylim(-top if lo < 0 else -0.05 * top, top)
        ax.set_title(name, pad=40 if k < 2 else 6)
        ax.set_xlabel("sim time [s]")
        ax.set_ylabel(unit)
        if k == 0:
            ax.legend(ncol=5, loc="lower left", bbox_to_anchor=(0, 1.22) if False else (0.0, -0.01), fontsize=7)
    f.text(0.07, 0.05, "dashed lines: joint limits (q), PLACEHOLDER vmax / amax, URDF effort limit (tau; may lie outside the "
           "plotted range when far above peak). Shaded bands = trajectory segments.", fontsize=7, color=MUTED)
    doc.save(f)
    # ---------------- page 2: TCP + validation
    f = doc.fig(ttl, "TCP path and validation numbers  |  " + sub.split("|")[0].strip())
    ax = f.add_axes([0.07, 0.50, 0.30, 0.32])
    tcp = dat["tcp"][::s]
    att = dat["att"][::s] > 0.5
    ax.plot(tcp[:, 0], tcp[:, 1], color=MUTED, lw=1.0, label="TCP path")
    if att.any():
        ax.plot(tcp[att, 0], tcp[att, 1], color="#D55E00", lw=2.0, label="cube attached")
    if r["true"]:
        ax.plot(r["true"][0], r["true"][1], marker="*", ms=11, color="#0072B2", ls="", label="cube true")
    ws = _WS.get("ws")
    if ws and isinstance(ws.get("place_target"), dict):
        ax.plot(ws["place_target"]["x_m"] * 1000, ws["place_target"]["y_m"] * 1000, marker="+", ms=10, mew=2,
                color="#009E73", ls="", label="place target")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("y [mm]")
    ax.set_title("TCP path, top view")
    ax.legend(loc="best", fontsize=7)
    ax = f.add_axes([0.44, 0.50, 0.53, 0.32])
    _shade_segments(ax, t, seg, label=True)
    ax.plot(t, tcp[:, 2], color="#0072B2", lw=1.3)
    ax.set_xlabel("sim time [s]")
    ax.set_ylabel("TCP z [mm]")
    ax.set_title("TCP height z(t)", pad=40)
    # validation text
    vt = []
    A = vt.append
    A("total sim time        : %s" % (r["total_time"] or "-"))
    A("max FK diff           : %s" % (r["max_fk"] or "-"))
    A("max IK error          : %s" % (r["max_ik"] or "-"))
    A("max FD round-trip err : %s" % (r["max_fd"] or "-"))
    A("velocity/acc limits   : %s" % (r["lim_vel"] or "-"))
    A("torque vs effort      : %s" % (r["lim_tau"] or "-"))
    A("any limit exceeded    : %s" % (r["any_exceeded"] or "-"))
    A("calibration           : %s" % r["calib"])
    A("")
    A("joint  peak|tau| [N m]  rigid  effort [N m]  ratio  status")
    for l in r["tau_table"]:
        A(l)
    vt = [w for l in vt for w in ((textwrap.wrap(l, 70, subsequent_indent=" " * 25) or [""]) if ":" in l[:26] else [l])]
    _mono_text(f, [0.05, 0.07, 0.43, 0.38], "\n".join(vt), fs=6.8, title="Validation (from kin_log TXT)")
    # segment table
    st = ["%-3s %-15s %-17s %6s %6s %9s %9s" % ("#", "segment", "type", "T [s]", "att", "max|qd|", "max|qdd|")]
    for sg in r["segs"]:
        st.append("%-3d %-15s %-17s %6.3f %6s %9s %9s" % (
            sg["i"], sg["name"][:15], sg["kind"][:17], sg["T"], sg["attached"],
            "%.3f" % max(sg["peakv"]) if sg["peakv"] else "-", "%.3f" % max(sg["peaka"]) if sg["peaka"] else "-"))
    vmax = max(r["vmax"]) if r["vmax"] else None
    amax = max(r["amax"]) if r["amax"] else None
    st.append("")
    st.append("PLACEHOLDER limits: vmax %s rad/s, amax %s rad/s^2" % (vmax, amax))
    _mono_text(f, [0.52, 0.07, 0.46, 0.38], "\n".join(st), fs=7.2, title="Segments")
    doc.save(f)
    # ---------------- page 3: coefficients
    if coeffs:
        blocks = []
        for sg in r["segs"]:
            head = "[%d] %s  (%s, T=%.4f s)" % (sg["i"], sg["name"], sg["kind"], sg["T"])
            body = []
            for ln in sg["lines"]:
                if "q(t) comes from per-step IK" in ln:
                    body.append("    q(t) from per-step IK (qd, qdd analytic via J^+, see kin log)")
                elif ln.strip().startswith("(for reference"):
                    body.append("    joint polynomial for same waypoints:")
                elif "peak |qd|" in ln:
                    continue
                else:
                    body.append(ln[:92])
            blocks.append([head] + body + [""])
        lines = [l for b in blocks for l in b]
        per_col = 66
        cols = [lines[i:i + per_col] for i in range(0, len(lines), per_col)][:2]
        f = doc.fig(ttl, "Segment equations and coefficients (from kin_log TXT); s(tau)=10tau^3-15tau^4+6tau^5, "
                         "a3=10D/T^3, a4=-15D/T^4, a5=6D/T^5")
        for k, col in enumerate(cols):
            _mono_text(f, [0.04 + k * 0.49, 0.06, 0.47, 0.82], "\n".join(col), fs=5.6)
        if len(lines) > 2 * per_col:
            f.text(0.96, 0.06, "... truncated, see TXT log", ha="right", fontsize=7, color=FAIL_COL)
        doc.save(f)


_WS = {}


def page_appendix(doc):
    f = doc.fig("Method and equations", "how the simulated trajectories and model values are produced")
    ax = f.add_axes([0.05, 0.07, 0.90, 0.80])
    ax.axis("off")
    blocks = [
        ("Quintic time scaling (joint segments and gripper)",
         [r"$s(\tau)=10\tau^{3}-15\tau^{4}+6\tau^{5},\quad \tau=t/T,\ t\in[0,T]$",
          r"$q_j(t)=q_{0j}+D_j\,s(\tau),\quad D_j=q_{1j}-q_{0j}$",
          r"$\dot q_j=\frac{D_j}{T}(30\tau^{2}-60\tau^{3}+30\tau^{4}),\quad "
          r"\ddot q_j=\frac{D_j}{T^{2}}(60\tau-180\tau^{2}+120\tau^{3})$",
          r"Expanded: $q=q_0+a_3t^3+a_4t^4+a_5t^5$,  $a_3=10D/T^3,\ a_4=-15D/T^4,\ a_5=6D/T^5$",
          r"Duration: $T=\max_j\max\!\left(1.875|D_j|/v_{max,j},\ \sqrt{5.7735|D_j|/a_{max,j}}\right)\geq t_{min}$, "
          r"rounded up to whole steps (dt = 1/240 s)"]),
        ("Cartesian straight line (descend, lift, place descend / retreat)",
         [r"$p(t)=p_0+(p_1-p_0)\,s(\tau)$,  orientation interpolated $R_0\to R_1$ along the same $s$",
          r"$q(t)$ from per-step inverse kinematics (Newton polish, seed = previous $q$); "
          r"$\dot q,\ddot q$ analytic: weighted least-squares $J^+$ applied to the quintic twist",
          r"Peak TCP speed $=1.875\,L/T$ for line length $L$; if the line is infeasible the segment falls back to a joint quintic"]),
        ("Inverse and forward dynamics (model values)",
         [r"$\tau=M(q)\ddot q+C(q,\dot q)\dot q+g(q)$  via PyBullet calculateInverseDynamics, URDF inertials",
          r"Payload while attached: $\tau_{pl}=J_v^{T}\,m_{cube}(a_{tcp}-g),\ a_{tcp}=J_v\ddot q+\dot J_v\dot q$ "
          r"(point mass at the TCP)",
          r"Forward-dynamics round trip: $\ddot q_{fd}=M^{-1}(\tau-h(q,\dot q))$, $h=ID(q,\dot q,0)$; "
          r"the max $|\ddot q_{fd}-\ddot q|$ is reported per run"]),
        ("Kinematics validation",
         [r"PyBullet link_tcp pose vs an independent numpy FK built from URDF joint origins and axes",
          r"IK error = independent FK(q) vs the target pose (position in mm, rotation in deg)"]),
        ("Notes and honesty",
         [r"The simulation is kinematic: torques are NOT measured, they are URDF-model values.",
          r"Velocity, acceleration and effort limits shown as dashed lines are PLACEHOLDER values (URDF limits x scale).",
          r"Gripper = xArm Gripper stand-in; grasp = kinematic attach; camera noise = PLACEHOLDER. SIMULATION ONLY."]),
    ]
    y = 1.0
    for head, lines in blocks:
        ax.text(0, y, head, fontsize=10.5, fontweight="bold", va="top", transform=ax.transAxes, color="#0072B2")
        y -= 0.04
        for ln in lines:
            ax.text(0.02, y, ln, fontsize=9, va="top", transform=ax.transAxes)
            y -= 0.045
        y -= 0.025
    doc.save(f)


# ----------------------------------------------------------------------------------------------- entry points
def build_report(log_paths, out_dir, title=None, png_dir=None):
    """Build the PDF from kin_log files (TXT or CSV paths). Returns the PDF path. Never raises on one bad log."""
    os.makedirs(out_dir, exist_ok=True)
    runs, bad = [], []
    seen = set()
    for p in log_paths:
        base = os.path.splitext(str(p))[0]
        if base in seen:
            continue
        seen.add(base)
        try:
            runs.append(load_run(str(p)))
        except Exception as e:
            bad.append((str(p), "%s: %s" % (type(e).__name__, e)))
    runs.sort(key=lambda r: r["stem"])
    ws = load_workspace()
    _WS["ws"] = ws
    calib = load_calib()
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = os.path.join(out_dir, "%s_xarm_sim_report.pdf" % stamp)
    title = title or "XarmAI simulation report"
    with PdfPages(out, metadata={"Title": title, "Subject": "SIMULATION ONLY"}) as pdf:
        doc = _Doc(pdf)
        if png_dir:
            os.makedirs(png_dir, exist_ok=True)
            doc.png_dir = png_dir
        page_cover(doc, runs, bad, title, calib)
        steps = [lambda: page_summary(doc, runs), lambda: page_scatter(doc, runs, ws)]
        for r in runs:
            steps.append(lambda r=r: pages_run(doc, r))
        steps.append(lambda: page_appendix(doc))
        for st in steps:
            try:
                st()
            except Exception:
                plt.close("all")
                f = doc.fig("Page skipped (internal error)")
                f.text(0.05, 0.8, traceback.format_exc()[-900:], family="monospace", fontsize=6, va="top", color=FAIL_COL)
                doc.save(f)
    return out
