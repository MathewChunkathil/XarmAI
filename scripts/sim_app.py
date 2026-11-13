"""XarmAI simulation control app (Dear PyGui + clean PyBullet 3D window).  SIMULATION ONLY.

    .venv\\Scripts\\python -I scripts\\sim_app.py

Two windows side by side: PyBullet 3D view (left, view only) and the Dear PyGui control panel (right).
Calibration is cached in results/sim_calib/sim_calib.json (loaded in ~1 s; "Recalibrate" reruns it in a
background thread). Pick & Place = perceive (C270 sim camera + AprilTags) -> candidates -> IK -> pick -> place.
Scored headless batch stays in scripts/sim_demo.py.

Threading: PyBullet GUI client and Dear PyGui both live in the MAIN thread. During a (blocking) pick-and-place the
UI is pumped from the SimWorld.on_step hook, so the window never freezes. Calibration runs in a worker thread with
its own headless SimWorld (separate DIRECT client).
Test hook: XARM_APP_TEST=1 (set sliders, press Pick & Place, save screenshots to XARM_APP_SHOTS, exit).
"""
import time as _t0mod
_T_IMPORT0 = _t0mod.perf_counter()
import argparse
import collections
import ctypes
import math
import os
import re
import sys
import threading
import time
import traceback
from ctypes import wintypes

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import sim_demo as sd  # noqa: E402  (adds ROOT + src/* to sys.path)
from sim import calib_cache as cc  # noqa: E402

import dearpygui.dearpygui as dpg  # noqa: E402
import pybullet as pb  # noqa: E402

# ------------------------------------------------------------------ look
BG = (17, 19, 25)
PANEL = (27, 30, 39)
PANEL2 = (38, 42, 54)
TEXT = (228, 232, 240)
MUTED = (137, 146, 164)
ACCENT = (64, 150, 255)
GREEN = (74, 205, 125)
RED = (240, 96, 96)
AMBER = (246, 188, 70)
CHIP = {"Calibrating": AMBER, "Ready": (70, 130, 220), "Running": ACCENT, "Done": GREEN, "Failed": RED}
OUTCOME_COL = {"SUCCESS": GREEN}

try:        # crisp rendering: work in physical pixels, scale the design (made at 96 dpi) by S
    ctypes.windll.user32.SetProcessDPIAware()
    S = max(1.0, ctypes.windll.user32.GetDpiForSystem() / 96.0)
    SCR_W, SCR_H = ctypes.windll.user32.GetSystemMetrics(0), ctypes.windll.user32.GetSystemMetrics(1)
except Exception:
    S, SCR_W, SCR_H = 1.0, 1920, 1080


def sc(v):
    return int(round(v * S))


VP_W, VP_H = sc(980), min(sc(1010), SCR_H - 70)
PB_W, PB_H = max(700, min(1300, SCR_W - VP_W - 40)), min(SCR_H - 110, 1100)
LEFT_W = sc(380)
CAM_W, CAM_H = sc(528), sc(297)
DEG = "\u00b0"


class Cancel(Exception):
    """Raised inside the calibration worker to stop it on app exit."""


def place_pybullet_window(x, y):
    """Best effort: move the PyBullet GUI window (own process id, title contains 'Bullet')."""
    try:
        u = ctypes.windll.user32
        pid, found = os.getpid(), []

        @ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
        def cb(hwnd, _):
            p = wintypes.DWORD()
            u.GetWindowThreadProcessId(hwnd, ctypes.byref(p))
            if p.value == pid and u.IsWindowVisible(hwnd):
                n = u.GetWindowTextLengthW(hwnd)
                buf = ctypes.create_unicode_buffer(n + 1)
                u.GetWindowTextW(hwnd, buf, n + 1)
                if "Bullet" in buf.value:
                    found.append(hwnd)
            return True
        u.EnumWindows(cb, 0)
        for h in found:
            u.SetWindowPos(h, 0, int(x), int(y), 0, 0, 0x0001 | 0x0004)     # NOSIZE | NOZORDER
            u.SetWindowTextW(h, "XarmAI — 3D View (Simulation)")
        return bool(found)
    except Exception:
        return False


def make_theme():
    with dpg.theme() as th:
        with dpg.theme_component(dpg.mvAll):
            col = {
                dpg.mvThemeCol_WindowBg: BG, dpg.mvThemeCol_ChildBg: PANEL, dpg.mvThemeCol_PopupBg: PANEL2,
                dpg.mvThemeCol_Text: TEXT, dpg.mvThemeCol_TextDisabled: (96, 104, 120),
                dpg.mvThemeCol_Border: (52, 57, 72), dpg.mvThemeCol_FrameBg: PANEL2,
                dpg.mvThemeCol_FrameBgHovered: (50, 56, 72), dpg.mvThemeCol_FrameBgActive: (58, 66, 86),
                dpg.mvThemeCol_Button: (48, 54, 70), dpg.mvThemeCol_ButtonHovered: (62, 70, 92),
                dpg.mvThemeCol_ButtonActive: (74, 84, 110), dpg.mvThemeCol_SliderGrab: ACCENT,
                dpg.mvThemeCol_SliderGrabActive: (120, 185, 255), dpg.mvThemeCol_Header: PANEL2,
                dpg.mvThemeCol_HeaderHovered: (50, 56, 72), dpg.mvThemeCol_TableHeaderBg: PANEL2,
                dpg.mvThemeCol_TableBorderLight: (44, 49, 62), dpg.mvThemeCol_TableBorderStrong: (52, 57, 72),
                dpg.mvThemeCol_TableRowBgAlt: (31, 35, 45), dpg.mvThemeCol_PlotHistogram: ACCENT,
                dpg.mvThemeCol_ScrollbarBg: PANEL, dpg.mvThemeCol_ScrollbarGrab: (70, 76, 96),
                dpg.mvThemeCol_Separator: (52, 57, 72),
            }
            for k, v in col.items():
                dpg.add_theme_color(k, v, category=dpg.mvThemeCat_Core)
            for k, v in {dpg.mvStyleVar_FrameRounding: sc(7), dpg.mvStyleVar_ChildRounding: sc(10),
                         dpg.mvStyleVar_GrabRounding: sc(7), dpg.mvStyleVar_ScrollbarRounding: sc(8),
                         dpg.mvStyleVar_GrabMinSize: sc(16), dpg.mvStyleVar_ChildBorderSize: 1,
                         dpg.mvStyleVar_ScrollbarSize: sc(10),
                         dpg.mvStyleVar_FrameBorderSize: 0}.items():
                dpg.add_theme_style(k, v, category=dpg.mvThemeCat_Core)
            for k, v in {dpg.mvStyleVar_WindowPadding: (14, 10), dpg.mvStyleVar_FramePadding: (9, 4),
                         dpg.mvStyleVar_ItemSpacing: (9, 6), dpg.mvStyleVar_CellPadding: (8, 4)}.items():
                dpg.add_theme_style(k, sc(v[0]), sc(v[1]), category=dpg.mvThemeCat_Core)
        with dpg.theme_component(dpg.mvPlot):
            dpg.add_theme_color(dpg.mvPlotCol_PlotBg, (21, 24, 31), category=dpg.mvThemeCat_Plots)
            dpg.add_theme_color(dpg.mvPlotCol_FrameBg, PANEL, category=dpg.mvThemeCat_Plots)
            dpg.add_theme_color(dpg.mvPlotCol_PlotBorder, (52, 57, 72), category=dpg.mvThemeCat_Plots)
            dpg.add_theme_color(dpg.mvPlotCol_LegendBg, (27, 30, 39, 200), category=dpg.mvThemeCat_Plots)
            dpg.add_theme_style(dpg.mvPlotStyleVar_PlotPadding, sc(8), sc(6), category=dpg.mvThemeCat_Plots)
    return th


def button_theme(base, hover, active, text=(255, 255, 255), rounding=8):
    rounding = sc(rounding)
    with dpg.theme() as th:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, base, category=dpg.mvThemeCat_Core)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, hover, category=dpg.mvThemeCat_Core)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, active, category=dpg.mvThemeCat_Core)
            dpg.add_theme_color(dpg.mvThemeCol_Text, text, category=dpg.mvThemeCat_Core)
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, rounding, category=dpg.mvThemeCat_Core)
    return th


def f1(v, fmt="%.1f"):
    return "-" if v in ("", None) else fmt % v


class App:
    def __init__(self, test=False):
        self.t_start = time.perf_counter()
        self.test = test
        self.shots = os.environ.get("XARM_APP_SHOTS", os.path.join(ROOT, "results", "sim_app", "shots"))
        self.req = collections.deque()
        self.logq = collections.deque()
        self.log_lines = 0
        self.busy = False
        self.status = "Calibrating"
        self.k = 0
        self.hist = []
        self.t_pump = self.t_live = self.t_plot = self.t_cam = 0.0
        self.cam_frozen = False
        self.fps = 0.0
        self._fps_t, self._fps_n = time.perf_counter(), 0
        self.step_n = 0
        self.t_render = self.t_live_cost = 0.0
        self.run_t0 = self.run_s0 = 0.0
        self.ready_t = None
        self.hist_q = collections.deque(maxlen=240)      # (t, [5 deg])
        self.calib = {"state": "none", "done": 0, "total": 1, "stage": "", "result": None, "error": None}
        self.cancel = False
        self.worker = None
        self.test_stage = 0
        self.cam_ms = 0.0
        self.cam_dirty = True
        self.shot0 = False
        self.test_t = 0.0
        self.test_mid = False
        self.test_result = None

        sd.log = self.tee                                   # Pipeline log() -> app log (thread-safe deque)
        a = argparse.Namespace(gui=True, auto=True, app=True, seed=0, runs=1, calib_poses=20, board_poses=20,
                               out=os.path.join(ROOT, "results", "sim_app"), use_calib_cache=False, hold=False, window=(PB_W, PB_H))
        self.args = a
        self.P = sd.Pipeline(a)
        self.w = self.P.world
        self.P.on_frame = self.on_frame
        self.w.on_step = self.on_step
        self.zones = self.w.add_zone_overlays()
        self.pump_hook_ik()
        self.chash = self.P.calib_hash()
        box, m = self.P.ws["cube_workspace_box"], sd.S["cube_margin_m"]
        self.bx = (1000 * (box["x_m"][0] + m), 1000 * (box["x_m"][1] - m))
        self.by = (1000 * (box["y_m"][0] + m), 1000 * (box["y_m"][1] - m))
        self.build_ui()
        self.applied = self.slider_vals()
        self.w.spawn_cube(*self.applied)
        self.w.set_arm(sd.S["park_q"])
        place_pybullet_window(0, 0)
        d = cc.load(self.chash)
        if d is not None:
            self.install_calib(d, "cache")
        else:
            self.start_calibration()
        self.set_status("Ready" if d is not None else "Calibrating")

    # ------------------------------------------------------------ plumbing
    def tee(self, *a):
        s = " ".join(str(x) for x in a)
        print(s, flush=True)
        for ln in s.split("\n"):
            if ln.strip() and not (ln.startswith(("id ", "----")) or re.match(r"run\d+-c\d+", ln)):
                self.logq.append(ln[:150])

    def pump_hook_ik(self):
        """IK/candidate evaluation can take seconds with no sim steps: keep the window alive from IK calls."""
        orig = self.P.ik.solve

        def solve(*a, **k):
            self.pump()
            return orig(*a, **k)
        self.P.ik.solve = solve
        ref = self.P.aux.ik_refine          # per-step IK of the pre-computed straight lines (hidden aux world)

        def refine(*a, **k):
            self.pump()
            return ref(*a, **k)
        self.P.aux.ik_refine = refine
        self.P.kin_pump = self.pump         # keep the UI alive while the per-run kin log is computed

    def pump(self, force=False):
        """Render one Dear PyGui frame (throttled). Raises UserStop when either window is closed."""
        now = time.perf_counter()
        if not force and now - self.t_pump < 0.014:
            return
        self.t_pump = now
        if not dpg.is_dearpygui_running() or not self.w.is_connected():
            raise sd.UserStop("window closed")
        self.update_live(now)
        self.flush_log()
        t1 = time.perf_counter()
        dpg.render_dearpygui_frame()
        self.t_render += time.perf_counter() - t1
        self.t_live_cost += t1 - now
        self._fps_n += 1
        if now - self._fps_t > 1.0:
            self.fps = self._fps_n / (now - self._fps_t)
            self._fps_n, self._fps_t = 0, now
            dpg.set_value("t_fps", "UI %.0f fps" % self.fps)

    def on_step(self):
        self.step_n += 1
        self.pump()
        close = os.environ.get("XARM_APP_TEST_CLOSE")        # test hook: simulate closing a window mid-run
        if close and self.busy and self.P.phase == "place":
            if close == "dpg":
                dpg.stop_dearpygui()
            else:
                self.w.p.disconnect()
        if self.test and not self.test_mid and self.busy and self.P.phase in ("place",):
            self.test_mid = True
            self.take_shots("02_mid_motion")

    def on_frame(self, img, dets, T):
        out = img.copy()
        for tid, c in (dets or {}).items():
            cv2.polylines(out, [c.astype(np.int32).reshape(-1, 1, 2)], True, (90, 255, 140), 3)
            cv2.putText(out, "id%d" % tid, (int(c[0, 0]), int(c[0, 1]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 1.1,
                        (90, 255, 140), 3)
        if T is not None and self.P.K is not None:
            rv, _ = cv2.Rodrigues(T[:3, :3])
            cv2.drawFrameAxes(out, self.P.K, self.P.dist, rv, T[:3, 3], 0.05, 5)
        self.set_cam(out)
        self.pump()

    def preview(self):
        """Cheap idle preview of the C270 view: same view/projection, 640x360, no distortion/noise/JPEG
        (the full camera model is used for perception and after each run)."""
        cam = self.P.cam
        self.P.sync_aux()
        img = self.P.aux.p.getCameraImage(640, 360, cam.view, cam.proj, renderer=pb.ER_TINY_RENDERER, shadow=0)[2]
        return cv2.cvtColor(np.asarray(img, np.uint8).reshape(360, 640, 4)[:, :, :3], cv2.COLOR_RGB2BGR)

    def set_cam(self, bgr):
        r = cv2.resize(bgr, (CAM_W, CAM_H), interpolation=cv2.INTER_AREA)
        self.cam_view[..., :3] = r[..., ::-1] * np.float32(1 / 255.0)

    # ------------------------------------------------------------ UI
    def build_ui(self):
        dpg.create_context()
        dpg.configure_app(manual_callback_management=False)
        font_ok = False
        for path in ("C:/Windows/Fonts/segoeui.ttf", "C:/Windows/Fonts/arial.ttf"):
            if os.path.exists(path):
                try:
                    with dpg.font_registry():
                        with dpg.font(path, sc(16)) as f:
                            dpg.add_font_range_hint(dpg.mvFontRangeHint_Default)
                            dpg.add_font_range(0x2010, 0x2030)
                        big = dpg.add_font(path, sc(24))
                    dpg.bind_font(f)
                    font_ok = True
                    break
                except Exception:
                    pass
        self.big = big if font_ok else None
        dpg.bind_theme(make_theme())
        th_primary = button_theme((46, 124, 246), (70, 146, 255), (34, 100, 210))
        th_ghost = button_theme((48, 54, 70), (62, 70, 92), (74, 84, 110), rounding=8)
        self.chip_th = {k: button_theme(c, c, c, text=(20, 22, 28), rounding=14) for k, c in CHIP.items()}

        flat = np.zeros(CAM_W * CAM_H * 4, np.float32)
        self.cam_view = flat.reshape(CAM_H, CAM_W, 4)
        self.cam_view[..., 3] = 1.0
        self.cam_view[..., :3] = 0.07
        with dpg.texture_registry():
            dpg.add_raw_texture(CAM_W, CAM_H, flat, format=dpg.mvFormat_Float_rgba, tag="tex_cam")
        self._flat = flat

        def title(text, sub=None):
            dpg.add_text(text.upper(), color=ACCENT)
            dpg.add_separator()

        def kv(label, tag, w=118):
            with dpg.group(horizontal=True):
                dpg.add_text(label, color=MUTED)
                dpg.add_text("-", tag=tag)

        with dpg.window(tag="main", no_title_bar=True, no_move=True, no_resize=True, no_scrollbar=True,
                        no_collapse=True):
            # ---- header
            with dpg.group(horizontal=True):
                t = dpg.add_text("XarmAI \u2014 Pick & Place (Simulation)")
                if self.big:
                    dpg.bind_item_font(t, self.big)
                dpg.add_spacer(width=sc(20))
                dpg.add_button(label="Calibrating", tag="chip", width=sc(130), height=sc(30))
                dpg.add_spacer(width=sc(10))
                dpg.add_text("SIMULATION ONLY", color=AMBER, tag="t_sim")
            dpg.add_spacer(height=sc(2))
            with dpg.group(horizontal=True):
                # ================= left column
                with dpg.group():
                    with dpg.child_window(width=LEFT_W, height=sc(178), no_scrollbar=True):
                        title("Cube pose")
                        for lab, tag, rng, dv, fmt in (
                                ("X", "sl_x", self.bx, 0.5 * sum(self.bx), "%.0f mm"),
                                ("Y", "sl_y", self.by, 0.5 * sum(self.by), "%.0f mm"),
                                ("Yaw", "sl_w", (-180, 180), 0.0, "%.0f " + DEG)):
                            with dpg.group(horizontal=True):
                                dpg.add_text(lab, color=MUTED)
                                dpg.add_spacer(width=sc(18 if lab == "Yaw" else 34))
                                dpg.add_slider_float(tag=tag, min_value=rng[0], max_value=rng[1], default_value=dv,
                                                     format=fmt, width=-1, callback=self.on_slider)
                        with dpg.group(horizontal=True):
                            b = dpg.add_button(label="Random pose", width=(LEFT_W - sc(38)) // 2, height=sc(34),
                                               tag="b_rand", callback=lambda: self.req.append("random"))
                            c = dpg.add_button(label="Centre", width=-1, height=sc(34), tag="b_centre",
                                               callback=lambda: self.req.append("centre"))
                            dpg.bind_item_theme(b, th_ghost)
                            dpg.bind_item_theme(c, th_ghost)
                    with dpg.child_window(width=LEFT_W, height=sc(192), no_scrollbar=True):
                        title("Actions")
                        pb_ = dpg.add_button(label="Pick & Place", width=-1, height=sc(50), tag="b_pick",
                                             callback=lambda: self.req.append("pick"))
                        dpg.bind_item_theme(pb_, th_primary)
                        if self.big:
                            dpg.bind_item_font(pb_, self.big)
                        with dpg.group(horizontal=True):
                            for lab, tag, r in (("Home", "b_home", "home"), ("Recalibrate", "b_recal", "recal")):
                                bb = dpg.add_button(label=lab, width=(LEFT_W - sc(38)) // 2 if r == "home" else -1,
                                                    height=sc(34), tag=tag, callback=lambda s, a, u: self.req.append(u),
                                                    user_data=r)
                                dpg.bind_item_theme(bb, th_ghost)
                        with dpg.group(horizontal=True):
                            dpg.add_text("Speed", color=MUTED)
                            dpg.add_slider_float(tag="sl_speed", min_value=0.25, max_value=4.0, default_value=1.0,
                                                 format="%.2fx", width=-1, callback=self.on_speed)
                    with dpg.child_window(width=LEFT_W, height=sc(340), no_scrollbar=True):
                        title("Last run")
                        dpg.add_text("-", tag="r_outcome", color=MUTED)
                        if self.big:
                            dpg.bind_item_font("r_outcome", self.big)
                        for lab, tag in (("True pose", "r_true"), ("Estimated", "r_est"), ("Error", "r_err"),
                                         ("Tag IDs seen", "r_tags"), ("Grasp", "r_grasp"), ("Place error", "r_place")):
                            kv(lab + ":", tag)
                        dpg.add_spacer(height=sc(2))
                        with dpg.child_window(width=-1, height=-1, border=False):
                            with dpg.table(tag="t_hist", header_row=True, row_background=True, borders_innerH=True,
                                           policy=dpg.mvTable_SizingStretchProp, scrollY=True, height=-1):
                                for c_ in ("Run", "Outcome", "Pos mm", "Place mm"):
                                    dpg.add_table_column(label=c_)
                    with dpg.child_window(width=LEFT_W, height=sc(172), no_scrollbar=True):
                        title("Calibration")
                        kv("Source:", "c_src")
                        kv("Intrinsics RMS:", "c_rms")
                        kv("T_base_cam error:", "c_ext")
                        dpg.add_progress_bar(tag="c_bar", width=-1, height=sc(22), overlay="")
                # ================= right column
                with dpg.group():
                    with dpg.child_window(width=-1, height=CAM_H + sc(60), no_scrollbar=True):
                        title("Camera - C270 (simulated)")
                        dpg.add_image("tex_cam", width=CAM_W, height=CAM_H)
                    with dpg.child_window(width=-1, height=sc(410), no_scrollbar=True):
                        with dpg.group(horizontal=True):
                            dpg.add_text("KINEMATICS", color=ACCENT)
                            dpg.add_spacer(width=sc(10))
                            dpg.add_text("Phase:", color=MUTED)
                            dpg.add_text("idle", tag="k_phase")
                        dpg.add_separator()
                        with dpg.group(horizontal=True):
                            with dpg.table(tag="t_kin", header_row=True, row_background=True, width=sc(300),
                                           borders_innerH=True, policy=dpg.mvTable_SizingStretchProp):
                                for c_ in ("Joint", "Angle", "deg/s", "Limits"):
                                    dpg.add_table_column(label=c_)
                                for i in range(5):
                                    with dpg.table_row():
                                        dpg.add_text("J%d" % (i + 1), color=MUTED)
                                        dpg.add_text("-", tag="k_a%d" % i)
                                        dpg.add_text("-", tag="k_v%d" % i)
                                        lo, hi = np.degrees(self.w.arm_lo[i]), np.degrees(self.w.arm_hi[i])
                                        dpg.add_text("%.0f..%.0f" % (lo, hi), color=MUTED)
                            with dpg.group():
                                for lab, tag in (("TCP x", "k_x"), ("TCP y", "k_y"), ("TCP z", "k_z"),
                                                 ("Roll", "k_r"), ("Pitch", "k_p"), ("Yaw", "k_yaw"),
                                                 ("Gripper", "k_g")):
                                    with dpg.group(horizontal=True):
                                        dpg.add_text(lab, color=MUTED)
                                        dpg.add_text("-", tag=tag)
                        with dpg.plot(height=-1, width=-1, no_menus=True, no_box_select=True, no_mouse_pos=True):
                            dpg.add_plot_legend(location=dpg.mvPlot_Location_NorthEast, outside=True)
                            dpg.add_plot_axis(dpg.mvXAxis, label="s", tag="ax_x")
                            dpg.add_plot_axis(dpg.mvYAxis, label="deg", tag="ax_y")
                            for i, col in enumerate(((255, 110, 110), (255, 180, 80), (120, 220, 120),
                                                     (90, 170, 255), (190, 130, 255))):
                                s = dpg.add_line_series([], [], label="J%d" % (i + 1), parent="ax_y",
                                                        tag="ser%d" % i)
                                with dpg.theme() as lt:
                                    with dpg.theme_component(dpg.mvLineSeries):
                                        dpg.add_theme_color(dpg.mvPlotCol_Line, col, category=dpg.mvThemeCat_Plots)
                                        dpg.add_theme_style(dpg.mvPlotStyleVar_LineWeight, 2.0 * S,
                                                            category=dpg.mvThemeCat_Plots)
                                dpg.bind_item_theme(s, lt)
                            dpg.set_axis_limits("ax_x", -10, 0)
                    with dpg.child_window(width=-1, height=-1, tag="log_win"):
                        with dpg.group(horizontal=True):
                            dpg.add_text("LOG", color=ACCENT)
                            dpg.add_spacer(width=sc(10))
                            dpg.add_text("", tag="t_fps", color=MUTED)
                        dpg.add_separator()
        dpg.create_viewport(title="XarmAI - Pick & Place (Simulation)", width=VP_W, height=VP_H,
                            x_pos=PB_W + 24, y_pos=0, vsync=False)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("main", True)

    def set_status(self, st):
        self.status = st
        dpg.configure_item("chip", label=st)
        dpg.bind_item_theme("chip", self.chip_th[st])

    def set_busy(self, busy):
        self.busy = busy
        can_calib = self.calib["state"] != "running"
        dpg.configure_item("b_pick", enabled=(not busy) and self.calibrated() and can_calib)
        dpg.configure_item("b_home", enabled=not busy)
        dpg.configure_item("b_recal", enabled=(not busy) and can_calib)

    def calibrated(self):
        return self.P.K is not None

    def slider_vals(self):
        return (dpg.get_value("sl_x") / 1000.0, dpg.get_value("sl_y") / 1000.0, math.radians(dpg.get_value("sl_w")))

    def on_slider(self):
        pass

    def on_speed(self):
        self.w.speed = float(dpg.get_value("sl_speed"))

    # ------------------------------------------------------------ live widgets
    def update_live(self, now):
        if now - self.t_live < 0.07:
            return
        self.t_live = now
        w = self.w
        try:
            q, qd = np.degrees(w.arm_q()), np.degrees(w.qd)
            T = w.tcp_T()
            g = math.degrees(w.p.getJointState(w.robot, w.grip_j[0])[0])
        except pb.error:
            return
        R = T[:3, :3]
        roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
        pitch = math.degrees(math.atan2(-R[2, 0], math.hypot(R[2, 1], R[2, 2])))
        yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        for i in range(5):
            dpg.set_value("k_a%d" % i, "%7.1f" % q[i])
            dpg.set_value("k_v%d" % i, "%6.1f" % qd[i])
        for tag, v in (("k_x", "%.0f mm" % (1000 * T[0, 3])), ("k_y", "%.0f mm" % (1000 * T[1, 3])),
                       ("k_z", "%.0f mm" % (1000 * T[2, 3])), ("k_r", "%.1f%s" % (roll, DEG)),
                       ("k_p", "%.1f%s" % (pitch, DEG)), ("k_yaw", "%.1f%s" % (yaw, DEG)),
                       ("k_g", "%.1f%s" % (g, DEG))):
            dpg.set_value(tag, v)
        dpg.set_value("k_phase", self.P.phase)
        self.hist_q.append((now, list(q)))
        if now - self.t_plot > 0.2:
            self.t_plot = now
            ts = [t - now for t, _ in self.hist_q]
            for i in range(5):
                dpg.set_value("ser%d" % i, [ts, [v[i] for _, v in self.hist_q]])
            dpg.fit_axis_data("ax_y")
            dpg.set_axis_limits("ax_x", -10, 0)
        # calibration progress
        c = self.calib
        if c["state"] == "running":
            dpg.set_value("c_bar", min(1.0, c["done"] / max(1, c["total"])))
            dpg.configure_item("c_bar", overlay="%s  %d/%d" % (c["stage"], c["done"], c["total"]))

    def flush_log(self):
        n = 0
        while self.logq and n < 40:
            ln = self.logq.popleft()
            dpg.add_text(ln, parent="log_win", color=(190, 198, 214), wrap=0)
            self.log_lines += 1
            n += 1
            if self.log_lines > 200:
                kids = dpg.get_item_children("log_win", 1)
                dpg.delete_item(kids[2])        # keep the header group + separator
                self.log_lines -= 1
        if n:
            dpg.set_y_scroll("log_win", -1.0)

    # ------------------------------------------------------------ calibration
    def install_calib(self, d, source):
        self.P.apply_calib(d)
        self.P.calib_info = {"source": source, "timestamp": d["timestamp"]}
        self.w.set_arm(sd.S["park_q"])       # apply_calib leaves the arm at ready; idle pose = park
        intr, ext = d["intr"], d["ext"]
        dpg.set_value("c_src", "cache, %s" % d["timestamp"] if source == "cache" else "fresh, %s" % d["timestamp"])
        dpg.set_value("c_rms", "%.3f px  (%d views)" % (intr["rms"], intr["views"]))
        dpg.set_value("c_ext", "%.1f mm / %.2f%s  (%d poses)" % (ext["cam_pos_mm"], ext["cam_rot_deg"], DEG,
                                                                 ext["poses"]))
        dpg.set_value("c_bar", 1.0)
        dpg.configure_item("c_bar", overlay="calibrated")
        self.ready_t = time.perf_counter() - _T_IMPORT0
        self.tee("[app] calibration %s ready (%s)" % (source, d["timestamp"]))
        self.set_busy(False)

    def start_calibration(self):
        c = self.calib
        c.update(state="running", done=0, total=self.args.board_poses + self.args.calib_poses, stage="starting",
                 result=None, error=None)
        dpg.set_value("c_src", "calibrating in background ...")
        dpg.set_value("c_bar", 0.0)
        self.set_status("Calibrating")
        self.set_busy(False)
        self.worker = threading.Thread(target=self.calib_worker, daemon=True)
        self.worker.start()
        self.set_busy(self.busy)

    def calib_worker(self):
        c = self.calib
        hp = None
        try:
            hargs = argparse.Namespace(**vars(self.args))
            hargs.gui, hargs.app = False, False
            hp = sd.Pipeline(hargs)         # headless SimWorld (pb.DIRECT), its own client
            nb, off = hargs.board_poses, {"intrinsics": 0, "extrinsics": hargs.board_poses}

            def prog(stage, done, total):
                if self.cancel:
                    raise Cancel()
                c["stage"] = "Intrinsics" if stage == "intrinsics" else "Extrinsics"
                c["done"] = off[stage] + done
            hp.on_progress = prog
            intr = hp.calibrate_intrinsics()
            ext = hp.calibrate_extrinsics()
            hp.world.set_arm(hp.q_ready)
            close = hp.closed_angle()
            d = cc.save(intr, ext, close, self.chash)
            c["result"] = {"intr": intr, "ext": ext, "K": hp.K, "dist": hp.dist, "T_base_cam": hp.T_base_cam,
                           "close_ang": close, "timestamp": d["timestamp"]}
            c["state"] = "done"
        except Cancel:
            c["state"] = "cancelled"
        except Exception as e:
            c["error"] = "%s: %s" % (type(e).__name__, e)
            c["state"] = "error"
            self.tee("[calib] ERROR " + c["error"])
            self.tee(traceback.format_exc().splitlines()[-3])
        finally:
            if hp is not None:
                hp.world.close()

    def poll_calibration(self):
        c = self.calib
        if c["state"] == "done":
            c["state"] = "applied"
            self.install_calib(c["result"], "fresh")
            self.set_status("Ready")
        elif c["state"] == "error":
            c["state"] = "failed"
            dpg.set_value("c_src", "FAILED: " + str(c["error"])[:60])
            dpg.configure_item("c_bar", overlay="failed")
            self.set_status("Failed")
            self.set_busy(False)

    # ------------------------------------------------------------ actions
    def cube_follow(self):
        cur = self.slider_vals()
        if cur != self.applied:
            self.w.set_cube_pose(cur[0], cur[1], cur[2], settle=0)
            self.applied = cur
            self.cam_frozen = False
            self.cam_dirty = True
            if self.status in ("Done", "Failed") and self.calibrated():
                self.set_status("Ready")

    def set_sliders(self, x, y, yaw_deg):
        dpg.set_value("sl_x", float(np.clip(1000 * x, *self.bx)))
        dpg.set_value("sl_y", float(np.clip(1000 * y, *self.by)))
        dpg.set_value("sl_w", float(yaw_deg))

    def handle(self, r):
        if self.busy:
            return
        if r == "random":
            x, y, yaw = self.P.sample_cube()
            self.set_sliders(x, y, math.degrees(yaw))
        elif r == "centre":
            self.set_sliders(0.5 * sum(self.bx) / 1000, 0.5 * sum(self.by) / 1000, 0.0)
        elif r == "home":
            self.do_home()
        elif r == "recal":
            if self.calib["state"] != "running":
                self.start_calibration()
        elif r == "pick":
            if self.calibrated() and self.calib["state"] != "running":
                self.do_pick()

    def do_home(self):
        self.set_busy(True)
        self.set_status("Running")
        self.P.phase = "home"
        try:
            self.w.detach_cube()
            self.w.move_gripper(0.0, name="home_open")
            self.w.move_joints(self.w.arm_q(), sd.S["park_q"], name="home")
            cur = self.slider_vals()
            self.w.set_cube_pose(*cur, settle=0)
            self.applied = cur
        finally:
            self.P.phase = "idle"
            self.set_busy(False)
        self.set_status("Ready")
        self.cam_frozen = False
        self.cam_dirty = True

    def do_pick(self):
        self.set_busy(True)
        self.set_status("Running")
        cur = self.slider_vals()
        self.k += 1
        self.tee("--- run %d: cube x=%.0f y=%.0f mm yaw=%.0f%s ---" % (self.k, 1000 * cur[0], 1000 * cur[1],
                                                                     math.degrees(cur[2]), DEG))
        self.step_n, self.run_t0 = 0, time.perf_counter()
        self.t_render = self.t_live_cost = 0.0
        rec = None
        try:
            rec = self.P.run_one(self.k, self.P.close_ang, cur, respawn=False)
        except (sd.UserStop, pb.error):
            self.P.phase = "idle"
            dpg.set_value("k_phase", "idle")
            raise
        except Exception as e:
            self.tee("[run] ERROR %s: %s" % (type(e).__name__, e))
            self.tee(traceback.format_exc().splitlines()[-2][:150])
            try:
                self.w.detach_cube()
            except Exception:
                pass
        self.P.phase = "idle"
        dpg.set_value("k_phase", "idle")
        dt = time.perf_counter() - self.run_t0
        self.tee("[perf] run %.1fs, %d sim steps (%.0f steps/s), UI %.0f fps, dpg render %.1fs, widget upd %.1fs" % (
            dt, self.step_n, self.step_n / max(dt, 1e-6), self.fps, self.t_render, self.t_live_cost))
        self.applied = cur
        self.cam_frozen = True
        self.show_result(rec)
        self.set_busy(False)
        self.test_result = rec["outcome"] if rec else "ERROR"

    def show_result(self, rec):
        if rec is None:
            self.set_status("Failed")
            dpg.set_value("r_outcome", "ERROR")
            dpg.configure_item("r_outcome", color=RED)
            return
        oc = rec["outcome"]
        ok = oc == "SUCCESS"
        dpg.set_value("r_outcome", oc)
        dpg.configure_item("r_outcome", color=GREEN if ok else RED)
        dpg.set_value("r_true", "x %.0f  y %.0f mm   yaw %.1f%s" % (1000 * rec["true_x"], 1000 * rec["true_y"],
                                                                  rec["true_yaw_deg"], DEG))
        if rec["est_x"] != "":
            dpg.set_value("r_est", "x %.0f  y %.0f mm   yaw %.1f%s" % (1000 * rec["est_x"], 1000 * rec["est_y"],
                                                                      rec["est_yaw_deg"], DEG))
            dpg.set_value("r_err", "%.1f mm / %.2f%s" % (rec["pos_err_mm"], abs(rec["yaw_err_deg"]), DEG))
        else:
            dpg.set_value("r_est", "-")
            dpg.set_value("r_err", "-")
        dpg.set_value("r_tags", ("id " + rec["tags"].replace(",", ", id ")) if rec["tags"] else "none")
        if rec.get("chosen_id"):
            dpg.set_value("r_grasp", "%s  yaw %.0f%s  (rank %s, %s feasible)" % (
                rec["chosen_id"], rec["chosen_yaw_deg"], DEG, rec["chosen_rank"], rec["feasible"]))
        else:
            dpg.set_value("r_grasp", "-")
        dpg.set_value("r_place", ("%.1f mm" % rec["place_err_mm"]) if rec["place_err_mm"] != "" else "-")
        self.hist.insert(0, rec)
        dpg.delete_item("t_hist", children_only=True, slot=1)
        for r in self.hist[:30]:
            with dpg.table_row(parent="t_hist"):
                dpg.add_text(str(r["run"]))
                dpg.add_text(r["outcome"], color=GREEN if r["outcome"] == "SUCCESS" else RED)
                dpg.add_text(f1(r["pos_err_mm"]))
                dpg.add_text(f1(r["place_err_mm"]))
        self.set_status("Done" if ok else "Failed")

    # ------------------------------------------------------------ screenshots (test hook)
    def take_shots(self, name):
        os.makedirs(self.shots, exist_ok=True)
        for _ in range(3):
            dpg.render_dearpygui_frame()
        dpg.output_frame_buffer(file=os.path.join(self.shots, name + "_dpg.png"))
        for _ in range(3):
            dpg.render_dearpygui_frame()
            time.sleep(0.03)
        try:
            wd, ht, view, proj = self.w.p.getDebugVisualizerCamera()[:4]
            img = self.w.p.getCameraImage(wd, ht, view, proj, renderer=pb.ER_BULLET_HARDWARE_OPENGL, shadow=1)[2]
            rgb = np.asarray(img, np.uint8).reshape(ht, wd, 4)[:, :, :3]
            cv2.imwrite(os.path.join(self.shots, name + "_3d.png"), rgb[:, :, ::-1])
        except Exception as e:
            self.tee("[shot] 3d failed %s" % e)
        try:
            from PIL import ImageGrab
            ImageGrab.grab(all_screens=True).save(os.path.join(self.shots, name + "_screen.png"))
        except Exception as e:
            self.tee("[shot] screen failed %s" % e)
        try:
            u = ctypes.windll.user32
            rc = wintypes.RECT()
            hw = []

            @ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
            def cb(h, _):
                pid = wintypes.DWORD()
                u.GetWindowThreadProcessId(h, ctypes.byref(pid))
                if pid.value == os.getpid() and u.IsWindowVisible(h):
                    u.GetWindowRect(h, ctypes.byref(rc))
                    n = u.GetWindowTextLengthW(h)
                    bf = ctypes.create_unicode_buffer(n + 1)
                    u.GetWindowTextW(h, bf, n + 1)
                    hw.append("%s %d,%d,%d,%d" % (bf.value[:12], rc.left, rc.top, rc.right, rc.bottom))
                return True
            u.EnumWindows(cb, 0)
            self.tee("[diag] windows %s | dpg vp %dx%d | dpi_aware=%s" % (
                hw, dpg.get_viewport_client_width(), dpg.get_viewport_client_height(), u.IsProcessDPIAware()))
        except Exception as e:
            self.tee("[diag] %s" % e)
        self.tee("[shot] saved " + name)

    def test_tick(self, now):
        s = self.test_stage
        if s == 0 and self.calib["state"] == "running" and not self.shot0 and now - self.t_start > 12:
            self.shot0 = True
            self.take_shots("00_calibrating")
        if s == 0:
            if self.calibrated() and self.calib["state"] != "running" and not self.busy:
                self.test_t, self.test_stage = now, 1
                self.set_sliders(0.38, 0.05, 25.0)
        elif s == 1 and now - self.test_t > 1.5:
            self.tee("[perf] idle: UI %.0f fps, camera refresh %.0f ms per frame (4 Hz)" % (self.fps, self.cam_ms))
            self.take_shots("01_idle")
            self.test_stage = 2
            self.req.append("pick")
        elif s == 2 and self.test_result is not None:
            self.test_t, self.test_stage = now, 3
        elif s == 3 and now - self.test_t > 1.0:
            self.take_shots("03_after_place")
            self.test_stage = 4

    # ------------------------------------------------------------ main loop
    def run(self):
        tl = time.perf_counter()
        try:
            while dpg.is_dearpygui_running() and self.w.is_connected():
                now = time.perf_counter()
                self.pump(force=True)
                self.poll_calibration()
                while self.req:
                    self.handle(self.req.popleft())
                if not self.busy:
                    self.cube_follow()
                    if not self.cam_frozen and now - self.t_cam > 0.25 and (self.cam_dirty or now - self.t_cam > 2.0):
                        self.t_cam = now
                        self.cam_dirty = False
                        tg = time.perf_counter()
                        self.set_cam(self.preview())
                        self.cam_ms = 0.8 * self.cam_ms + 0.2 * 1000 * (time.perf_counter() - tg)
                if self.test:
                    self.test_tick(now)
                    if self.test_stage == 4:
                        return 0 if self.test_result == "SUCCESS" else 2
                    if now - self.t_start > 600:
                        self.tee("[test] timeout")
                        return 3
                el = time.perf_counter() - tl
                tl = time.perf_counter()
                time.sleep(max(0.0, 1 / 60 - (time.perf_counter() - now)))
        except (sd.UserStop, pb.error) as e:
            if isinstance(e, pb.error) and "not connected" not in str(e).lower():
                raise
        return 0

    def close(self):
        self.cancel = True
        if self.worker is not None:
            self.worker.join(timeout=8)
        try:
            self.P.close()
        except Exception:
            pass
        try:
            dpg.destroy_context()
        except Exception:
            pass


def main():
    t0 = time.perf_counter()
    sys.setswitchinterval(0.002)        # keep the UI responsive while the calibration worker thread computes
    app = App(test=bool(os.environ.get("XARM_APP_TEST")))
    code = 0
    try:
        sd.log("[app] constructed in %.1f s" % (time.perf_counter() - t0))
        code = app.run()
    finally:
        app.close()
    if app.ready_t is not None:
        print("[app] first Ready at %.1f s after launch" % app.ready_t, flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
