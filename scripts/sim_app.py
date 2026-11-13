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
from sim import commands as cmd_mod  # noqa: E402
from sim import zones as zn  # noqa: E402
from sim.task_runner import TaskRunner, M as TM  # noqa: E402
from sim.voice import VoiceInput, read_wav_16k, MAX_RECORD_S  # noqa: E402

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
TASK_COL = {"queued": MUTED, "running": ACCENT, "done": GREEN, "failed": RED, "skipped": AMBER}
DEFAULT_POS = {1: (0.375, 0.05), 2: (0.30, -0.03), 3: (0.45, 0.13)}        # mm-free defaults, >= 0.11 m apart
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
CAM_W, CAM_H = sc(400), sc(225)
PREV_W, PREV_H = 400, 225        # idle preview render size (aux TinyRenderer)
PROFILE = float(os.environ.get("XARM_APP_PROFILE", "0") or 0)     # seconds of idle profiling, then exit
PROFILE_AFTER = 4.0                                              # seconds after Ready before measuring
PROFILE_CUBES = [int(v) for v in os.environ.get("XARM_APP_START_CUBES", "1").split(",")]    # cubes enabled at start
DEG = "\u00b0"
MIC_IDLE = "🎤 Hold to talk / SPACE"


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
        self.voice = None                               # VoiceInput, created once self.tee exists (below)
        self.voice_state = "idle"                       # idle | listening | transcribing
        self.voice_q = collections.deque()              # (text, info, error) from the transcription worker thread
        self.mic_src = ""                               # "button" | "key"
        self.voice_heard = ""
        self._ui_cache = {}                             # tag -> last value / config (skip redundant dpg calls)
        self.prof = collections.defaultdict(float)       # XARM_APP_PROFILE=<s>: idle time per frame part
        self._fps_t, self._fps_n = time.perf_counter(), 0
        self.step_n = 0
        self.t_render = self.t_live_cost = 0.0
        self.run_t0 = self.run_s0 = 0.0
        self.ready_t = self.ready_abs = None
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
        self.voice = VoiceInput(log=self.tee)
        a = argparse.Namespace(gui=True, auto=True, app=True, seed=0, runs=1, calib_poses=20, board_poses=20,
                               out=os.path.join(ROOT, "results", "sim_app"), use_calib_cache=False, hold=False, window=(PB_W, PB_H))
        self.args = a
        self.P = sd.Pipeline(a)
        self.w = self.P.world
        self.R = TaskRunner(self.P)
        self.names = self.R.names
        self.sel = 1
        self.cmd_stop = False
        self.task_rows = []                                 # [(text, state, err)] of the running / last command
        self.cs = {n: {"on": n in PROFILE_CUBES, "x": 1000 * DEFAULT_POS[n][0], "y": 1000 * DEFAULT_POS[n][1], "yaw": 0.0}
                   for n in self.names}
        self.P.on_frame = self.on_frame
        self.w.on_step = self.on_step
        self.zones = self.w.add_zone_overlays()
        self.belt = self.w.add_conveyor()                   # conveyor (visible world; the hidden camera world gets a copy on sync)
        if self.belt is not None:
            self.zones += self.belt.add_window_overlay()
        self.t_belt, self.belt_acc, self.track_steps, self.test_track = time.perf_counter(), 0.0, 0, False
        self.pump_hook_ik()
        self.chash = self.P.calib_hash()
        box, m = self.P.ws["cube_workspace_box"], sd.S["cube_margin_m"]
        self.bx = (1000 * (box["x_m"][0] + m), 1000 * (box["x_m"][1] - m))
        self.by = (1000 * (box["y_m"][0] + m), 1000 * (box["y_m"][1] - m))
        self.build_ui()
        self.voice.start_loading()                          # background thread: the UI never blocks on the model
        self.applied = {}
        self.sync_cube_widgets()
        for n, c in self.cs.items():
            if c["on"]:
                self.R.put_cube(n, c["x"] / 1000, c["y"] / 1000, math.radians(c["yaw"]))
                self.applied[n] = self.state_tuple(n)
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
        self.update_voice_ui()
        self.flush_log()
        t1 = time.perf_counter()
        dpg.render_dearpygui_frame()
        self.t_render += time.perf_counter() - t1
        self.t_live_cost += t1 - now
        self.prof["live+log"] += t1 - now
        self.prof["dpg_render"] += time.perf_counter() - t1
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
        if self.P.phase == "track":
            self.track_steps += 1
        if self.test and not self.test_track and self.busy and self.P.phase == "track" and self.track_steps >= 150:
            self.test_track = True
            self.take_shots("02a_mid_tracking")
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
        w, h = PREV_W, PREV_H           # TinyRenderer cost ~ pixels: 400x225 ~ 35 ms vs 640x360 ~ 95 ms
        img = self.P.aux.p.getCameraImage(w, h, cam.view, cam.proj, renderer=pb.ER_TINY_RENDERER, shadow=0)[2]
        return cv2.cvtColor(np.asarray(img, np.uint8).reshape(h, w, 4)[:, :, :3], cv2.COLOR_RGB2BGR)

    def set_cam(self, bgr):
        r = bgr if bgr.shape[:2] == (CAM_H, CAM_W) else cv2.resize(
            bgr, (CAM_W, CAM_H), interpolation=cv2.INTER_AREA if bgr.shape[1] > CAM_W else cv2.INTER_LINEAR)
        np.multiply(r[..., ::-1], np.float32(1 / 255.0), out=self.cam_view[..., :3])

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
        self.emoji_font = None
        try:
            if os.path.exists("C:/Windows/Fonts/seguiemj.ttf"):
                with dpg.font_registry():
                    with dpg.font("C:/Windows/Fonts/seguiemj.ttf", sc(13)) as ef:
                        dpg.add_font_range_hint(dpg.mvFontRangeHint_Default)
                        dpg.add_font_range(0x1F3A4, 0x1F3A4)
                self.emoji_font = ef
        except Exception:
            self.emoji_font = None
        dpg.bind_theme(make_theme())
        th_primary = button_theme((46, 124, 246), (70, 146, 255), (34, 100, 210))
        th_ghost = button_theme((48, 54, 70), (62, 70, 92), (74, 84, 110), rounding=8)
        self.chip_th = {k: button_theme(c, c, c, text=(20, 22, 28), rounding=14) for k, c in CHIP.items()}
        self.cube_th = {}
        for n in self.names:        # swatch buttons: dim = not selected, full colour + white text = selected
            r_, g_, b_ = self.R.cube_set[n].rgb
            dim = (int(r_ * 0.38) + 14, int(g_ * 0.38) + 14, int(b_ * 0.38) + 14)
            self.cube_th[n] = {False: button_theme(dim, (r_ // 2, g_ // 2, b_ // 2), (r_, g_, b_), text=(200, 205, 215), rounding=8),
                               True: button_theme((r_, g_, b_), (min(255, r_ + 25), min(255, g_ + 25), min(255, b_ + 25)),
                                                  (r_, g_, b_), text=(255, 255, 255), rounding=8)}
        self.task_th = None

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
            with dpg.group(horizontal=True):
                dpg.add_text("CALIBRATION", color=ACCENT)
                dpg.add_progress_bar(tag="c_bar", width=sc(190), height=sc(16), overlay="")
                dpg.add_text("-", tag="c_src")
                dpg.add_text("", tag="c_rms", color=MUTED)
                dpg.add_text("", tag="c_ext", color=MUTED)
            dpg.add_spacer(height=sc(2))
            with dpg.group(horizontal=True):
                # ================= left column
                with dpg.group():
                    with dpg.child_window(width=LEFT_W, height=sc(262), no_scrollbar=True):
                        title("Cubes & layout")
                        with dpg.group(horizontal=True):
                            for n in self.names:
                                with dpg.group():
                                    bsel = dpg.add_button(label="%s (%d)" % (self.names[n].title(), n), tag="cb_%d" % n,
                                                          width=(LEFT_W - sc(40)) // 3, height=sc(28),
                                                          callback=self.on_select, user_data=n)
                                    dpg.bind_item_theme(bsel, self.cube_th[n][n == self.sel])
                                    dpg.add_checkbox(label="in scene", tag="ck_%d" % n, default_value=self.cs[n]["on"],
                                                     callback=self.on_toggle, user_data=n)
                        dpg.add_text("Sliders move: red cube", tag="t_sel", color=MUTED)
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
                            bw = (LEFT_W - sc(44)) // 3
                            for lab, tag, r in (("Random pose", "b_rand", "random"), ("Centre", "b_centre", "centre"),
                                                ("Random layout", "b_layout", "layout")):
                                bb = dpg.add_button(label=lab, width=bw, height=sc(32), tag=tag,
                                                    callback=lambda s_, a_, u: self.req.append(u), user_data=r)
                                dpg.bind_item_theme(bb, th_ghost)
                    with dpg.child_window(width=LEFT_W, height=sc(284), no_scrollbar=True):
                        title("Commands")
                        dpg.add_input_text(tag="cmd_in", hint="place the green cube in zone 1 and stack red on green",
                                           width=-1, on_enter=True, callback=self.on_cmd_enter)
                        with dpg.group(horizontal=True):
                            br = dpg.add_button(label="Run", tag="b_run", width=sc(70), height=sc(30),
                                                callback=lambda: self.submit_command(dpg.get_value("cmd_in"), "text"))
                            dpg.bind_item_theme(br, th_primary)
                            bs = dpg.add_button(label="Stop after current task", tag="b_stop", width=-1, height=sc(30),
                                                callback=self.on_stop, enabled=False)
                            dpg.bind_item_theme(bs, th_ghost)
                        # voice: hold the button (or SPACE) to talk; release -> faster-whisper -> self.submit_command(.., "voice")
                        with dpg.group(horizontal=True):
                            bm = dpg.add_button(label=MIC_IDLE, tag="b_mic", width=sc(196), height=sc(28), enabled=False)
                            dpg.bind_item_theme(bm, th_ghost)
                            if self.emoji_font:
                                dpg.bind_item_font(bm, self.emoji_font)
                            with dpg.tooltip("b_mic"):
                                dpg.add_text("Voice: loading...", tag="mic_tip")
                            dpg.add_checkbox(label="Auto-run after voice", tag="ck_auto", default_value=True)
                        dpg.add_progress_bar(tag="v_level", width=-1, height=sc(20), default_value=0.0, overlay="Voice: loading...")
                        dpg.add_text("", tag="v_heard", wrap=LEFT_W - sc(30), color=(190, 198, 214), show=False)
                        dpg.add_text("", tag="cmd_echo", wrap=LEFT_W - sc(30), color=(190, 198, 214))
                        dpg.add_text("", tag="cmd_err", color=RED, wrap=LEFT_W - sc(30), show=False)
                        with dpg.child_window(width=-1, height=-1, border=False):
                            with dpg.table(tag="t_task", header_row=True, row_background=True, borders_innerH=True,
                                           policy=dpg.mvTable_SizingStretchProp, scrollY=True, height=-1):
                                for c_, wd in (("#", 0.3), ("Task", 3.4), ("Status", 1.1), ("Err mm", 0.9)):
                                    dpg.add_table_column(label=c_, init_width_or_weight=wd)
                    with dpg.child_window(width=LEFT_W, height=sc(158), no_scrollbar=True):
                        title("Actions")
                        with dpg.group(horizontal=True):
                            pb_ = dpg.add_button(label="Pick & Place", width=(LEFT_W - sc(38)) * 2 // 3, height=sc(40),
                                                 tag="b_pick", callback=lambda: self.req.append("pick"))
                            dpg.bind_item_theme(pb_, th_primary)
                            if self.big:
                                dpg.bind_item_font(pb_, self.big)
                            dpg.add_text("to zone")
                            dpg.add_combo(("1", "2", "3"), default_value="1", tag="cb_zone", width=-1)
                        with dpg.group(horizontal=True):
                            for lab, tag, r in (("Home", "b_home", "home"), ("Recalibrate", "b_recal", "recal")):
                                bb = dpg.add_button(label=lab, width=(LEFT_W - sc(38)) // 2 if r == "home" else -1,
                                                    height=sc(30), tag=tag, callback=lambda s, a, u: self.req.append(u),
                                                    user_data=r)
                                dpg.bind_item_theme(bb, th_ghost)
                        with dpg.group(horizontal=True):
                            dpg.add_text("Speed", color=MUTED)
                            dpg.add_slider_float(tag="sl_speed", min_value=0.25, max_value=4.0, default_value=1.0,
                                                 format="%.2fx", width=-1, callback=self.on_speed)
                    with dpg.child_window(width=LEFT_W, height=sc(142), no_scrollbar=True):
                        with dpg.group(horizontal=True):
                            dpg.add_text("CONVEYOR", color=ACCENT)
                            dpg.add_spacer(width=sc(8))
                            dpg.add_text("stopped", tag="belt_state", color=MUTED)
                        dpg.add_separator()
                        with dpg.group(horizontal=True):
                            bb = dpg.add_button(label="Start belt", tag="b_belt", width=sc(104), height=sc(28),
                                                callback=self.on_belt_toggle)
                            dpg.bind_item_theme(bb, th_primary)
                            dpg.add_slider_float(tag="sl_belt", min_value=0.0, max_value=6.0, format="%.1f cm/s", width=-1,
                                                 default_value=100 * (self.belt.speed if self.belt else 0.03),
                                                 callback=self.on_belt_speed)
                        with dpg.group(horizontal=True):
                            dpg.add_text("Feed", color=MUTED)
                            dpg.add_combo(tuple(self.names[n] for n in self.names), default_value=self.names[1], tag="cb_feed",
                                          width=sc(86))
                            bf = dpg.add_button(label="Feed cube", tag="b_feed", width=-1, height=sc(28), callback=self.on_feed)
                            dpg.bind_item_theme(bf, th_ghost)
                        with dpg.group(horizontal=True):
                            dpg.add_checkbox(label="Pick from belt to zone", tag="ck_beltpick", default_value=False)
                            dpg.add_combo(("1", "2", "3"), default_value="1", tag="cb_beltzone", width=-1)
                # ================= right column
                with dpg.group():
                    with dpg.child_window(width=-1, height=CAM_H + sc(56), no_scrollbar=True):
                        title("Camera - C270 (simulated)")
                        dpg.add_image("tex_cam", width=CAM_W, height=CAM_H)
                    with dpg.child_window(width=-1, height=sc(176), no_scrollbar=True):
                        title("Last run")
                        with dpg.group(horizontal=True):
                            dpg.add_text("-", tag="r_outcome", color=MUTED)
                            if self.big:
                                dpg.bind_item_font("r_outcome", self.big)
                            dpg.add_spacer(width=sc(14))
                            dpg.add_text("", tag="r_grasp", color=MUTED)
                        with dpg.group(horizontal=True):
                            with dpg.group():
                                kv("True pose:", "r_true")
                                kv("Estimated:", "r_est")
                            dpg.add_spacer(width=sc(18))
                            with dpg.group():
                                kv("Pose error:", "r_err")
                                kv("Place error:", "r_place")
                        dpg.add_text("", tag="belt_info", wrap=sc(540), color=(190, 198, 214))
                    with dpg.child_window(width=-1, height=sc(262), no_scrollbar=True):
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
        with dpg.item_handler_registry(tag="mic_handlers"):
            dpg.add_item_activated_handler(callback=lambda: self.mic_press("button"))
            dpg.add_item_deactivated_handler(callback=lambda: self.mic_release("button"))
        dpg.bind_item_handler_registry("b_mic", "mic_handlers")
        with dpg.handler_registry():
            dpg.add_key_down_handler(dpg.mvKey_Spacebar, callback=lambda: self.mic_press("key"))
            dpg.add_key_release_handler(dpg.mvKey_Spacebar, callback=lambda: self.mic_release("key"))
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
        ok = (not busy) and self.calibrated() and can_calib
        dpg.configure_item("b_pick", enabled=ok)
        dpg.configure_item("b_run", enabled=ok)
        dpg.configure_item("b_home", enabled=not busy)
        for t in ("b_belt", "sl_belt", "b_feed", "cb_feed", "ck_beltpick", "cb_beltzone"):
            dpg.configure_item(t, enabled=(not busy) and (self.belt is not None) and (ok or t in ("b_belt", "sl_belt")))
        dpg.configure_item("b_recal", enabled=(not busy) and can_calib)
        for t in ("b_rand", "b_centre", "b_layout"):
            dpg.configure_item(t, enabled=not busy)
        if not busy:
            dpg.configure_item("b_stop", enabled=False)

    def calibrated(self):
        return self.P.K is not None

    def state_tuple(self, n):
        c = self.cs[n]
        return (c["x"] / 1000.0, c["y"] / 1000.0, math.radians(c["yaw"]))

    def on_slider(self):
        """Sliders act on the selected cube."""
        c = self.cs[self.sel]
        c["x"], c["y"], c["yaw"] = float(dpg.get_value("sl_x")), float(dpg.get_value("sl_y")), float(dpg.get_value("sl_w"))

    def sync_cube_widgets(self):
        """Selected cube -> sliders, swatch highlight, label; enabled flags -> checkboxes."""
        c = self.cs[self.sel]
        dpg.set_value("sl_x", float(np.clip(c["x"], *self.bx)))
        dpg.set_value("sl_y", float(np.clip(c["y"], *self.by)))
        dpg.set_value("sl_w", float(c["yaw"]))
        dpg.set_value("t_sel", "Sliders move: %s cube%s" % (self.names[self.sel],
                                                           "" if c["on"] else "  (not in scene)"))
        for n in self.names:
            dpg.bind_item_theme("cb_%d" % n, self.cube_th[n][n == self.sel])
            dpg.set_value("ck_%d" % n, self.cs[n]["on"])

    def on_select(self, sender, app_data, user_data):
        self.sel = int(user_data)
        self.sync_cube_widgets()

    def on_toggle(self, sender, app_data, user_data):
        self.req.append(("toggle", int(user_data), bool(app_data)))

    def set_cube_state(self, n, x_m, y_m, yaw_deg):
        c = self.cs[n]
        c["x"], c["y"], c["yaw"] = 1000 * x_m, 1000 * y_m, float(yaw_deg)
        if n == self.sel:
            self.sync_cube_widgets()

    def cubes_from_world(self):
        """After robot motion: copy the (physics) cube poses into the slider state so nothing snaps back."""
        for n, body in self.w.cubes.items():
            T = self.w.cube_pose_true(n)
            c = self.cs[n]
            c["x"], c["y"], c["yaw"] = 1000 * T[0, 3], 1000 * T[1, 3], math.degrees(sd.cube_yaw(T))
            self.applied[n] = self.state_tuple(n)

    def on_speed(self):
        self.w.speed = float(dpg.get_value("sl_speed"))

    # ------------------------------------------------------------ conveyor
    def on_belt_toggle(self):
        if self.belt is None or self.busy:
            return
        self.belt.running = not self.belt.running
        self.t_belt = time.perf_counter()
        self.tee("[belt] %s at %.1f cm/s" % ("running" if self.belt.running else "stopped", 100 * self.belt.speed))

    def on_belt_speed(self):
        if self.belt is None or self.busy:
            return
        self.belt.set_speed(float(dpg.get_value("sl_belt")) / 100.0)

    def on_feed(self):
        if self.belt is None or self.busy:
            return
        inv = {v: k for k, v in self.names.items()}
        n = inv[dpg.get_value("cb_feed")]
        auto = bool(dpg.get_value("ck_beltpick"))
        zone = int(dpg.get_value("cb_beltzone"))
        self.req.append(("feed", n, auto, zone))

    def do_feed(self, n, auto, zone):
        tasks = [cmd_mod.Task("feed", n, text="Feed button")]
        if auto:
            tasks.append(cmd_mod.Task("pick_belt", n, zone=zone, text="Feed button (auto-pick)"))
        self.task_rows = [[t.describe(self.names), "queued", ""] for t in tasks]
        self.refresh_tasks()
        dpg.set_value("cmd_echo", "Understood: " + "; ".join("%d) %s" % (i + 1, t.describe(self.names)) for i, t in enumerate(tasks)))
        dpg.configure_item("cmd_echo", show=True)
        dpg.configure_item("cmd_err", show=False)
        self.do_command(tasks, "Feed button", "button")

    def belt_idle_advance(self, now):
        """Idle belt animation: sim time follows wall time (x playback speed) while nothing else moves."""
        b = self.belt
        dt_wall = min(now - self.t_belt, 0.1)
        self.t_belt = now
        if b is None or not b.running or b.v <= 0.0:
            return
        self.belt_acc += dt_wall * 240.0 * self.w.speed
        k = int(self.belt_acc)
        if k <= 0:
            return
        self.belt_acc -= k
        self.w.advance_fast(k)
        self.cubes_from_world()
        sel = self.cs[self.sel]
        if self.sel in self.w.cubes and b.cfg.from_world(sel["x"] / 1000, sel["y"] / 1000)[0] > -0.02 and abs(
                b.cfg.from_world(sel["x"] / 1000, sel["y"] / 1000)[1]) < b.cfg.width:
            self.sync_cube_widgets()
        self.cam_frozen = False
        self.cam_dirty = True

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
        if self.belt is not None:
            self._val("belt_state", ("running %.1f cm/s" % (100 * self.belt.speed)) if self.belt.running else "stopped")
            self._cfg("b_belt", label="Stop belt" if self.belt.running else "Start belt")
            self._val("sl_belt", 100 * self.belt.speed)
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
        dpg.set_value("c_rms", "| RMS %.3f px (%d views)" % (intr["rms"], intr["views"]))
        dpg.set_value("c_ext", "| T_base_cam err %.1f mm / %.2f%s (%d poses)" % (ext["cam_pos_mm"], ext["cam_rot_deg"], DEG,
                                                                                ext["poses"]))
        dpg.set_value("c_bar", 1.0)
        dpg.configure_item("c_bar", overlay="calibrated")
        self.ready_t = time.perf_counter() - _T_IMPORT0
        self.ready_abs = time.perf_counter()
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
        moved = False
        for n, c in self.cs.items():
            if not c["on"] or n not in self.w.cubes:
                continue
            cur = self.state_tuple(n)
            if cur != self.applied.get(n):
                self.R.put_cube(n, cur[0], cur[1], cur[2], settle=0)
                self.applied[n] = cur
                moved = True
        if moved:
            self.cam_frozen = False
            self.cam_dirty = True
            if self.status in ("Done", "Failed") and self.calibrated():
                self.set_status("Ready")

    def enabled(self):
        return [n for n, c in self.cs.items() if c["on"]]

    def toggle_cube(self, n, on):
        self.cs[n]["on"] = on
        if on:
            self.R.put_cube(n, *self.state_tuple(n))
            self.applied[n] = self.state_tuple(n)
        else:
            self.R.remove_cube(n)
            self.applied.pop(n, None)
        self.sync_cube_widgets()
        self.cam_frozen = False
        self.cam_dirty = True

    def random_pose_for(self, n):
        """Random pose for cube n keeping the minimum centre spacing to the other cubes in the scene."""
        others = [(self.cs[k]["x"] / 1000, self.cs[k]["y"] / 1000) for k in self.enabled() if k != n]
        for _ in range(200):
            x, y, yaw = self.P.sample_cube()
            if all(math.hypot(x - ox, y - oy) >= TM["cube_min_spacing_m"] for ox, oy in others):
                return x, y, yaw
        return x, y, yaw

    def handle(self, r):
        if self.busy:
            return
        if isinstance(r, tuple):
            if r[0] == "toggle":
                self.toggle_cube(r[1], r[2])
            elif r[0] == "cmd":
                if self.calibrated() and self.calib["state"] != "running":
                    self.do_command(*r[1:])
            elif r[0] == "feed":
                if self.calibrated() and self.calib["state"] != "running":
                    self.do_feed(*r[1:])
            return
        if r == "random":
            x, y, yaw = self.random_pose_for(self.sel)
            self.set_cube_state(self.sel, x, y, math.degrees(yaw))
        elif r == "centre":
            self.set_cube_state(self.sel, 0.5 * sum(self.bx) / 1000, 0.5 * sum(self.by) / 1000, 0.0)
        elif r == "layout":
            lay = self.R.random_layout(self.enabled(), self.P.rng)
            if lay is None:
                self.tee("[layout] the enabled cubes do not fit with the %.0f mm minimum spacing" % (1000 * TM["cube_min_spacing_m"]))
            else:
                for n, (x, y, yaw) in lay.items():
                    self.set_cube_state(n, x, y, math.degrees(yaw))
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
        finally:
            self.P.phase = "idle"
            self.set_busy(False)
        self.set_status("Ready")
        self.cam_frozen = False
        self.cam_dirty = True

    def after_motion(self):
        self.P.phase = "idle"
        dpg.set_value("k_phase", "idle")
        for n in self.names:                                  # cubes fed / removed by the conveyor tasks
            self.cs[n]["on"] = n in self.w.cubes
        self.cubes_from_world()
        self.sync_cube_widgets()
        self.cam_frozen = True

    def do_pick(self):
        """Pick the selected cube and place it in the zone chosen next to the button (default zone 1)."""
        n = self.sel
        if n not in self.w.cubes:
            self.tee("[pick] %s cube is not in the scene (tick 'in scene')" % self.names[n])
            return
        zone = int(dpg.get_value("cb_zone"))
        self.set_busy(True)
        self.set_status("Running")
        cur = self.state_tuple(n)
        self.k += 1
        self.tee("--- run %d: %s cube x=%.0f y=%.0f mm yaw=%.0f%s -> zone %d ---" % (
            self.k, self.names[n], 1000 * cur[0], 1000 * cur[1], math.degrees(cur[2]), DEG, zone))
        self.step_n, self.run_t0 = 0, time.perf_counter()
        self.t_render = self.t_live_cost = 0.0
        rec = None
        try:
            rec = self.R.run_task(cmd_mod.Task("place", n, zone=zone, text="Pick & Place"), self.k, "Pick & Place button")
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
        dt = time.perf_counter() - self.run_t0
        self.tee("[perf] run %.1fs, %d sim steps (%.0f steps/s), UI %.0f fps, dpg render %.1fs, widget upd %.1fs" % (
            dt, self.step_n, self.step_n / max(dt, 1e-6), self.fps, self.t_render, self.t_live_cost))
        self.after_motion()
        self.show_result(rec)
        self.set_busy(False)
        self.test_result = rec["outcome"] if rec else "ERROR"

    # ------------------------------------------------------------ commands (text now, voice later)
    def on_cmd_enter(self):
        self.submit_command(dpg.get_value("cmd_in"), "text")

    def on_stop(self):
        self.cmd_stop = True
        dpg.configure_item("b_stop", enabled=False)
        self.tee("[command] stop requested: the current task finishes, the rest is skipped")

    def submit_command(self, text, source="text"):
        """Entry point for typed AND spoken commands (the voice agent calls submit_command(text, source="voice")).
        Parses (rule-based, sim/commands.py), shows the echo / errors, queues the tasks. Returns True if queued.
        Thread-safe enough: it only reads the scene and appends to the request queue."""
        res = self.R.parse(text)
        self.show_parse(res)
        self.tee("[command/%s] %r -> %s%s" % (source, text, res.echo or "-", ("  ERRORS: " + "; ".join(res.errors)) if res.errors else ""))
        if source != "text":
            dpg.set_value("cmd_in", text)
        if not res.ok:
            return False
        if self.busy or not self.calibrated() or self.calib["state"] == "running":
            dpg.set_value("cmd_err", "busy or not calibrated - command not started")
            dpg.configure_item("cmd_err", show=True)
            return False
        self.task_rows = [[t.describe(self.names), "queued", ""] for t in res.tasks]
        self.refresh_tasks()
        self.req.append(("cmd", res.tasks, text, source))
        return True

    # ------------------------------------------------------------ voice (push-to-talk)
    def _cfg(self, tag, **kw):
        """configure_item only when something changed (skips redundant dpg calls: fps)."""
        key = tuple(sorted(kw.items()))
        if self._ui_cache.get(("c", tag, tuple(sorted(kw)))) != key:
            self._ui_cache[("c", tag, tuple(sorted(kw)))] = key
            dpg.configure_item(tag, **kw)

    def _val(self, tag, v):
        k = ("v", tag)
        if self._ui_cache.get(k) != v:
            self._ui_cache[k] = v
            dpg.set_value(tag, v)

    def mic_can_record(self):
        v = self.voice
        return v.mic_ok and v.ready and not self.busy and self.calibrated() and self.calib["state"] != "running"

    def mic_press(self, src):
        if self.voice_state != "idle" or not self.mic_can_record():
            return
        if src == "key" and (dpg.is_item_active("cmd_in") or dpg.is_item_focused("cmd_in")):
            return                                         # typing a space in the text box
        if self.voice.start_recording():
            self.voice_state, self.mic_src = "listening", src
            self.tee("[voice] listening (%s) ..." % src)

    def mic_release(self, src):
        if self.voice_state != "listening" or src != self.mic_src:
            return
        self.finish_recording(self.voice.stop_recording())

    def finish_recording(self, audio):
        """audio (float32 mono 16 kHz) -> worker-thread transcription; the result comes back through self.voice_q."""
        if len(audio) < 0.3 * 16000:
            self.voice_state = "idle"
            self.voice_heard = "Too short - hold the button while you speak"
            return
        self.voice_state = "transcribing"
        self.voice.transcribe_async(audio, lambda text, info, err: self.voice_q.append((text, info, err)))

    def poll_voice(self):
        while self.voice_q:
            text, info, err = self.voice_q.popleft()
            self.voice_state = "idle"
            if err:
                self.voice_heard = "Voice error: %s" % err[:80]
                continue
            if not text:
                self.voice_heard = "Heard: (nothing recognised)"
                self.tee("[voice] heard nothing")
                continue
            self.voice_heard = 'Heard: "%s"   (%.2f s, %s)' % (text, info.get("seconds", 0.0), self.voice.device.split()[0])
            self.tee("[voice] heard %r (%.2f s for %.1f s audio, %s)" % (text, info.get("seconds", 0), info.get("audio_s", 0),
                                                                     info.get("device", "")))
            dpg.set_value("cmd_in", text)
            if dpg.get_value("ck_auto"):
                self.submit_command(text, source="voice")
            else:
                self.show_parse(self.R.parse(text))
                self.voice_heard += "\nReview / edit, then press Run."

    def update_voice_ui(self):
        v = self.voice
        listening = self.voice_state == "listening"
        if listening and v.timed_out:
            self.mic_release(self.mic_src)
            listening = False
        ok = self.mic_can_record() or listening
        self._cfg("b_mic", enabled=ok)
        if not v.mic_ok:
            tip = v.mic_error or "no microphone"
        elif v.status == "error":
            tip = "Voice model failed: " + v.error
        elif not v.ready:
            tip = "Voice model is loading ..."
        elif self.busy:
            tip = "Disabled while a task is running"
        elif not self.calibrated():
            tip = "Disabled until calibration is done"
        else:
            tip = "Hold, speak, release (or hold SPACE). Offline faster-whisper + rule parser, no LLM."
        self._val("mic_tip", tip)
        self._cfg("b_mic", label="🎤 Release to send" if listening else MIC_IDLE)
        if listening:
            txt, col = "Listening...  %.1f / %.0f s" % (v.rec_seconds(), MAX_RECORD_S), AMBER
        elif self.voice_state == "transcribing":
            txt, col = "Transcribing...", ACCENT
        elif not v.mic_ok:
            txt, col = "Voice: no microphone", RED
        else:
            txt, col = v.status_text(), (GREEN if v.ready else (RED if v.status == "error" else MUTED))
        self._cfg("v_level", overlay=txt)
        self._val("v_level", float(v.level) if listening else 0.0)
        self._val("v_heard", self.voice_heard)
        self._cfg("v_heard", show=bool(self.voice_heard))

    def show_parse(self, res):
        dpg.set_value("cmd_echo", ("Understood: " + res.echo) if res.echo else "")
        dpg.set_value("cmd_err", "\n".join(res.errors))
        dpg.configure_item("cmd_err", show=bool(res.errors))
        dpg.configure_item("cmd_echo", show=bool(res.echo))

    def refresh_tasks(self):
        dpg.delete_item("t_task", children_only=True, slot=1)
        for i, (txt, st, err) in enumerate(self.task_rows, 1):
            with dpg.table_row(parent="t_task"):
                dpg.add_text(str(i), color=MUTED)
                dpg.add_text(txt, wrap=sc(190))
                dpg.add_text(st, color=TASK_COL.get(st, TEXT))
                dpg.add_text(err, color=MUTED)

    def do_command(self, tasks, text, source="text"):
        self.set_busy(True)
        self.set_status("Running")
        self.cmd_stop = False
        dpg.configure_item("b_stop", enabled=True)
        self.step_n, self.run_t0 = 0, time.perf_counter()

        def on_status(i, state, rec):
            if 1 <= i <= len(self.task_rows):
                row = self.task_rows[i - 1]
                row[1] = state
                if rec and rec.get("place_err_mm", "") != "":
                    row[2] = "%.1f" % rec["place_err_mm"]
                elif rec and rec.get("detail"):
                    row[2] = "-"
                if state == "queued":
                    pass
                self.refresh_tasks()
            if rec is not None and state in ("done", "failed"):
                self.show_result(rec)
            if state == "running":
                self.tee("--- task %d/%d: %s ---" % (i, len(tasks), self.task_rows[i - 1][0]))
        recs = []
        try:
            recs = self.R.execute(tasks, text, on_status=on_status, should_stop=lambda: self.cmd_stop)
        except (sd.UserStop, pb.error):
            self.P.phase = "idle"
            dpg.set_value("k_phase", "idle")
            raise
        except Exception as e:
            self.tee("[command] ERROR %s: %s" % (type(e).__name__, e))
            self.tee(traceback.format_exc().splitlines()[-2][:150])
            try:
                self.w.detach_cube()
            except Exception:
                pass
        for j in range(len(recs), len(self.task_rows)):        # tasks never reached (stopped)
            if self.task_rows[j][1] in ("queued", "running"):
                self.task_rows[j][1] = "skipped"
        self.refresh_tasks()
        self.after_motion()
        ok = bool(recs) and all(r.get("outcome") == "SUCCESS" for r in recs) and len(recs) == len(tasks)
        self.tee("[command] done: %d/%d tasks SUCCESS (%.1f s)" % (
            sum(r.get("outcome") == "SUCCESS" for r in recs), len(tasks), time.perf_counter() - self.run_t0))
        self.set_busy(False)
        self.set_status("Done" if ok else "Failed")
        self.test_result = "SUCCESS" if ok else "FAIL"

    def show_result(self, rec, history_only=False):
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
        if rec.get("chosen_id"):
            dpg.set_value("r_grasp", "%s  yaw %.0f%s  (rank %s, %s feasible)" % (
                rec["chosen_id"], rec["chosen_yaw_deg"], DEG, rec["chosen_rank"], rec["feasible"]))
        else:
            dpg.set_value("r_grasp", "-")
        dpg.set_value("r_place", ("%.1f mm" % rec["place_err_mm"]) if rec["place_err_mm"] != "" else "-")
        if rec.get("detail"):
            dpg.set_value("r_grasp", rec["detail"][:70])
        if rec.get("belt_v_est_mm_s") is not None:
            dpg.set_value("belt_info", "Belt: v_est %.1f +- %.1f (true %.1f) mm/s | %d/%d frames | contact %s mm | grasp %s mm" % (
                rec["belt_v_est_mm_s"], rec["belt_sigma_v_mm_s"], rec["belt_v_true_mm_s"], rec["belt_frames_used"],
                rec["belt_frames"], ("%.1f" % rec["belt_track_err_mm"]) if rec.get("belt_track_err_mm") not in (None, "") else "-",
                ("%.1f" % rec["grasp_offset_mm"]) if rec.get("grasp_offset_mm") not in (None, "") else "-"))
        self.hist.insert(0, rec)
        if not self.busy:
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
            if os.environ.get("XARM_APP_TEST_VOICE") or os.environ.get("XARM_APP_NO_SCREEN"):
                raise RuntimeError("full-desktop capture disabled for the voice test (dpg + 3d only)")
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
        cmd_text = os.environ.get("XARM_APP_COMMAND") or (os.environ.get("XARM_APP_TEST_VOICE") and "voice")
        if s == 0:
            if self.calibrated() and self.calib["state"] != "running" and not self.busy:
                self.test_t, self.test_stage = now, 1
                if cmd_text:
                    nums = [int(v) for v in os.environ.get("XARM_APP_CUBES", "1,2,3").split(",")]
                    for n in self.names:
                        if (n in nums) != self.cs[n]["on"]:
                            self.req.append(("toggle", n, n in nums))
                    if os.environ.get("XARM_APP_BELT_IDLE") and self.belt is not None:      # test hook: belt running + a fed cube riding it (idle animation)
                        self.belt.set_speed(float(os.environ["XARM_APP_BELT_IDLE"]) / 100.0)
                        self.belt.running = True
                        self.req.append(("feed", 1, False, 2))
                    lay = os.environ.get("XARM_APP_LAYOUT")
                    if lay:
                        for n, t in zip(nums, lay.split(";")):
                            x, y, yw = (float(v) for v in t.split(","))
                            self.set_cube_state(n, x, y, yw)
                    else:
                        self.req.append("layout")
                else:
                    self.set_sliders(0.38, 0.05, 25.0)
        elif s == 1 and now - self.test_t > (9.0 if os.environ.get("XARM_APP_BELT_IDLE") else 4.0) and not self.busy:
            self.tee("[perf] idle: UI %.0f fps, camera refresh %.0f ms per frame (4 Hz)" % (self.fps, self.cam_ms))
            self.take_shots("01_idle")
            self.test_stage = 2
            if os.environ.get("XARM_APP_TEST_VOICE"):
                self.test_stage = 5
            elif cmd_text:
                dpg.set_value("cmd_in", cmd_text)
                self.submit_command(cmd_text, "text")
            else:
                self.req.append("pick")
        elif s == 5 and self.voice.ready:
            # voice test hook: feed the wav as if it was spoken (no microphone needed)
            v = self.voice
            self.voice_state, self.mic_src = "listening", "test"
            v.recording, v.t_rec0, v.level = True, now - 2.4, 0.55
            self.test_t, self.test_stage = now, 6
        elif s == 6 and now - self.test_t > 0.6:
            v = self.voice
            self.take_shots("v1_listening")
            v.recording, v.level = False, 0.0
            audio = read_wav_16k(os.environ["XARM_APP_TEST_VOICE"])
            self.tee("[test] feeding %s (%.1f s) as spoken audio" % (os.environ["XARM_APP_TEST_VOICE"], len(audio) / 16000))
            self.finish_recording(audio)
            self.test_t, self.test_stage = now, 7
        elif s == 7 and self.voice_heard and self.voice_state == "idle":
            self.take_shots("v2_heard")
            self.test_stage = 2
        elif s == 2 and self.test_result is not None:
            self.test_t, self.test_stage = now, 3
        elif s == 3 and now - self.test_t > 1.0:
            self.take_shots("03_after_place")
            self.test_stage = 4

    def set_sliders(self, x, y, yaw_deg):
        self.set_cube_state(self.sel, x, y, yaw_deg)

    # ------------------------------------------------------------ main loop
    def run(self):
        tl = time.perf_counter()
        try:
            while dpg.is_dearpygui_running() and self.w.is_connected():
                now = time.perf_counter()
                self.pump(force=True)
                self.poll_calibration()
                self.poll_voice()
                while self.req:
                    self.handle(self.req.popleft())
                if not self.busy:
                    self.belt_idle_advance(now)
                    tf = time.perf_counter()
                    self.cube_follow()
                    self.prof["cube_follow"] += time.perf_counter() - tf
                    if not self.cam_frozen and now - self.t_cam > 0.3 and (self.cam_dirty or now - self.t_cam > 5.0):
                        self.t_cam = now
                        self.cam_dirty = False
                        tg = time.perf_counter()
                        self.set_cam(self.preview())
                        self.cam_ms = 0.8 * self.cam_ms + 0.2 * 1000 * (time.perf_counter() - tg)
                        self.prof["preview"] += time.perf_counter() - tg
                        self.prof["n_preview"] += 1
                if self.test:
                    self.test_tick(now)
                    if self.test_stage == 4:
                        return 0 if self.test_result == "SUCCESS" else 2
                    if now - self.t_start > 600:
                        self.tee("[test] timeout")
                        return 3
                el = time.perf_counter() - tl
                tl = time.perf_counter()
                ts = time.perf_counter()
                time.sleep(max(0.0, 1 / 60 - (time.perf_counter() - now)))
                self.prof["sleep"] += time.perf_counter() - ts
                self.prof["frames"] += 1
                if PROFILE and self.ready_abs is not None and now < self.ready_abs + PROFILE_AFTER:
                    self.prof.clear()
                if PROFILE and self.ready_abs is not None and now > self.ready_abs + PROFILE_AFTER + PROFILE:
                    self.report_profile()
                    return 0
        except (sd.UserStop, pb.error) as e:
            if isinstance(e, pb.error) and "not connected" not in str(e).lower():
                raise
        return 0

    def report_profile(self):
        cam = self.P.cam
        tt = {"sync_aux": [], "getCameraImage": [], "convert+resize": []}
        for _ in range(5):
            t0 = time.perf_counter()
            self.P.sync_aux()
            t1 = time.perf_counter()
            img = self.P.aux.p.getCameraImage(PREV_W, PREV_H, cam.view, cam.proj, renderer=pb.ER_TINY_RENDERER, shadow=0)[2]
            t2 = time.perf_counter()
            self.set_cam(cv2.cvtColor(np.asarray(img, np.uint8).reshape(PREV_H, PREV_W, 4)[:, :, :3], cv2.COLOR_RGB2BGR))
            t3 = time.perf_counter()
            for k, v in zip(tt, (t1 - t0, t2 - t1, t3 - t2)):
                tt[k].append(1000 * v)
        self.tee("[profile] preview parts ms: " + ", ".join("%s %.1f" % (k, np.mean(v)) for k, v in tt.items()) +
                 "; cubes in world %s, aux %s" % (sorted(self.w.cubes), sorted(self.P.aux.cubes)))
        n = max(1.0, self.prof["frames"])
        self.tee("[profile] idle %d frames; ms/frame: %s; previews %d, preview ms %.1f; UI fps %.1f" % (
            n, ", ".join("%s %.2f" % (k, 1000 * v / n) for k, v in self.prof.items() if k not in ("frames", "n_preview", "preview")),
            self.prof["n_preview"], 1000 * self.prof["preview"] / max(1.0, self.prof["n_preview"]), self.fps))

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
