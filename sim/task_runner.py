"""Multi-cube task runner: perceive -> candidates (neighbour filter) -> pick -> place / stack.  SIMULATION ONLY.

Works on top of a `sim_demo.Pipeline` (world, hidden aux world, calibration, IK, kin log). The Pipeline's single-cube
`run_one` (the scored batch) is NOT touched; this module adds the multi-cube / zone / stacking tasks.

Honesty rules kept: perception uses only the ESTIMATED K / T_base_cam; ground truth (true cube poses) is used for
scoring and for placing props only. Where a pose is NOT perceived (a cube outside the camera view) the runner falls back
to the last COMMANDED / RECORDED pose and says so in the task record (`src_*` fields, kin-log header).
All numbers in M are PLACEHOLDER (sim demo parameters, not hardware facts).
"""
import math
import os
import sys
import time
from typing import Callable, Dict, List, Optional

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (ROOT, *(os.path.join(ROOT, "src", k) for k in ("cube_perception", "candidate_evaluator"))):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from candidate_evaluator import Obstacle, evaluate, format_table, generate, rank  # noqa: E402
from candidate_evaluator.generate import pose_to_matrix  # noqa: E402
from candidate_evaluator.ik_backend import joint_margin  # noqa: E402
from candidate_evaluator.neighbors import neighbor_hit, tool_yaw_of  # noqa: E402
from cube_perception import (cube_yaw, estimate_cube_pose, fold_yaw, identify_cubes, load_cube_set,  # noqa: E402
                             median_pose, to_base)
from sim import commands as cmd  # noqa: E402
from sim import zones as zn  # noqa: E402
from sim.pyb_world import Tinv  # noqa: E402

M = {
    "cube_min_spacing_m": 0.11,     # random layout: min centre distance. PLACEHOLDER: open finger outer edge 79 mm + margin + half cube 25 mm = 108 mm
    "cube_margin_m": 0.03,          # layout: keep cube centres this far inside the pick zone
    "stack_settle_steps": 480,      # physics steps after releasing a stacked cube
    "max_level": 2,                 # stack levels above the table (= 3 cubes high)
    "stack_z_tol_m": 0.010,         # stack success: A centre height vs (B centre + edge)
    "upright_cos": math.cos(math.radians(10)),
    "settle_steps": 240,
    "n_frames": 3,
}


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def T_of(R, t):
    X = np.eye(4)
    X[:3, :3], X[:3, 3] = R, t
    return X


def wrap90(a):
    return fold_yaw(a)


class TaskError(Exception):
    """Pre-condition problem (cube not in scene, nothing to stack on, zone full ...): the task fails with this text."""


class TaskRunner:
    def __init__(self, P, cube_set=None):
        self.P = P
        self.w = P.world
        self.ws = P.ws
        self.edge = P.edge
        self.table_z = P.table_z
        self.cube_set = cube_set or load_cube_set()
        self.names = {n: m.name for n, m in self.cube_set.items()}
        self.zones = zn.place_zones(self.ws)
        self.zone_numbers = tuple(z["number"] for z in self.zones)
        self.zone_boxes = [{"x_m": z["x_m"], "y_m": z["y_m"]} for z in self.zones]
        self.rec: Dict[int, dict] = {}                 # cube -> recorded / commanded pose (NOT ground truth)
        self.support: Dict[int, Optional[int]] = {}    # cube -> cube it rests on (absent / None = table)
        self.spacing_place = float(self.ws.get("place_slot_spacing_m", 0.07))
        self.gap = float(self.ws.get("stack_release_gap_m", 0.002))
        self.fp = P.cfg["gripper"].get("finger_footprint") or {}
        self.records: List[dict] = []
        self._belt = None

    @property
    def belt(self):
        """Conveyor operations (sim/belt_task.py), created on first use."""
        if self._belt is None:
            from sim.belt_task import BeltOps
            self._belt = BeltOps(self)
        return self._belt

    # ------------------------------------------------------------------ scene / layout
    def in_scene(self) -> List[int]:
        return sorted(self.w.cubes)

    def parse(self, text):
        return cmd.parse(text, cubes=self.names, zones=self.zone_numbers, in_scene=self.in_scene())

    def put_cube(self, n, x, y, yaw, settle=M["settle_steps"]):
        """User / layout placement of cube n on the table (spawns it when absent). Records the pose as 'layout'."""
        if n in self.w.cubes:
            self.w.set_cube_pose(x, y, yaw, settle=settle, number=n)
        else:
            self.w.spawn_cube(x, y, yaw, number=n)
        self.rec[n] = {"x": x, "y": y, "z": self.table_z + self.edge / 2, "yaw": yaw,
                       "src": "layout (set by user / sliders)", "zone": None}
        self.release_supports(n)

    def remove_cube(self, n):
        if n in self.w.cubes:
            if self.w.cube == self.w.cubes[n]:
                self.w.detach_cube()
            self.w.remove(self.w.cubes[n])
        self.rec.pop(n, None)
        self.release_supports(n)

    def release_supports(self, n):
        self.support.pop(n, None)
        for k in [k for k, v in self.support.items() if v == n]:
            self.support[k] = None

    def random_layout(self, numbers, rng, spacing=None):
        """Non-overlapping random {n: (x, y, yaw)} inside the pick zone (centre spacing >= spacing). None if it
        does not fit."""
        spacing = M["cube_min_spacing_m"] if spacing is None else spacing
        pts = zn.min_cube_spacing(self.ws["cube_workspace_box"], len(numbers), rng, spacing, M["cube_margin_m"])
        if pts is None:
            return None
        return {n: (x, y, float(rng.uniform(-math.pi, math.pi))) for n, (x, y) in zip(numbers, pts)}

    def apply_layout(self, layout):
        for n, (x, y, yaw) in layout.items():
            self.put_cube(n, x, y, yaw)

    # ------------------------------------------------------------------ perception
    def perceive_all(self, n_frames=None):
        """n frames -> detect all tags -> group by cube ID set -> PnP per cube (K_est) -> median -> base frame
        (z snapped to the stack levels). -> (ests {cube: dict}, info)."""
        P = self.P
        n_frames = M["n_frames"] if n_frames is None else n_frames
        per = {n: {"Ts": [], "ids": set(), "reasons": []} for n in self.cube_set}
        img0, dets0 = None, {}
        for f in range(n_frames):
            img = P.grab()
            d = P.detect(img)
            if f == 0:
                img0, dets0 = img, d
            for n, dets in identify_cubes(list(d.items()), self.cube_set).items():
                r = estimate_cube_pose(dets, P.K, P.dist, self.cube_set[n], P.perc_cfg(M["max_level"]))
                if r.valid:
                    per[n]["Ts"].append(r.T_cam_cube)
                    per[n]["ids"].update(r.tag_ids_used)
                else:
                    per[n]["reasons"].append(r.invalid_reason)
        ests = {}
        for n, v in per.items():
            if len(v["Ts"]) >= 1:
                Tcc = median_pose(v["Ts"])
                Tb = to_base(P.T_base_cam, Tcc, self.table_z, self.edge, max_level=M["max_level"])
                ests[n] = {"x": float(Tb[0, 3]), "y": float(Tb[1, 3]), "z": float(Tb[2, 3]), "yaw": cube_yaw(Tb),
                           "T_cc": Tcc, "valid": len(v["Ts"]), "ids": sorted(v["ids"])}
        owned = {i for m in self.cube_set.values() for i in m.ids}
        info = {"img": img0, "dets": {i: c for i, c in dets0.items() if i in owned},
                "reasons": {n: v["reasons"] for n, v in per.items() if v["reasons"]}}
        return ests, info

    def locate(self, n, ests):
        """Pose of cube n: PERCEIVED if visible, else the last commanded / recorded pose. -> (pose dict, source str)."""
        if n in ests:
            e = ests[n]
            return ({"x": e["x"], "y": e["y"], "z": e["z"], "yaw": e["yaw"]},
                    "perception (camera, tags %s, %d/%d frames)" % (",".join(map(str, e["ids"])), e["valid"], M["n_frames"]))
        if n in self.rec:
            r = self.rec[n]
            return ({k: r[k] for k in ("x", "y", "z", "yaw")}, "recorded: %s (cube not in camera view)" % r["src"])
        return None, "none"

    def obstacles(self, ests, exclude, z_tcp):
        """Other cubes (perceived or recorded pose) whose top is above the open-finger bottom at TCP height z_tcp."""
        fb = z_tcp + float(self.fp.get("finger_bottom_above_tcp_m", 0.0))
        out = []
        for n in self.in_scene():
            if n in exclude:
                continue
            pose, _ = self.locate(n, ests)
            if pose is None or pose["z"] + self.edge / 2 <= fb:
                continue
            out.append(Obstacle(pose["x"], pose["y"], pose["yaw"], self.edge, self.names.get(n, str(n))))
        return out

    # ------------------------------------------------------------------ targets
    def pick_slot(self, zone_no, exclude, ests):
        """First free slot of the zone (target first). Free = no other cube centre closer than the slot spacing."""
        z = zn.get_zone(self.ws, zone_no)
        others = []
        for n in self.in_scene():
            if n == exclude:
                continue
            pose, _ = self.locate(n, ests)
            if pose is not None:
                others.append((pose["x"], pose["y"]))
        for (x, y) in zn.zone_slots(z, self.spacing_place, self.edge):
            if all(math.hypot(x - ox, y - oy) >= self.spacing_place - 1e-6 for ox, oy in others):
                return x, y, z
        raise TaskError("zone %d is full (no free slot at %.0f mm spacing)" % (zone_no, 1000 * self.spacing_place))

    def level_of(self, zc):
        return int(round((zc - (self.table_z + self.edge / 2)) / self.edge))

    # ------------------------------------------------------------------ place planning
    def plan_place(self, c, T_est, eyaw, tgt, obstacles):
        """Commanded place poses from the ESTIMATED grasp geometry; IK check; finger footprint vs neighbours."""
        P = self.P
        T_grasp = pose_to_matrix(c.grasp_pose)
        T_cube_tcp = np.linalg.inv(T_est) @ T_grasp
        n = round((eyaw - tgt["yaw"]) / (math.pi / 2))          # cube is 90-deg symmetric: least tool rotation
        T_cube_t = T_of(rz(tgt["yaw"] + n * math.pi / 2), [tgt["x"], tgt["y"], tgt["z"]])
        T_place = T_cube_t @ T_cube_tcp
        if obstacles and self.fp and neighbor_hit(T_place[0, 3], T_place[1, 3], tool_yaw_of(T_place), obstacles, self.fp):
            return None, "NEIGHBOR_COLLISION"
        T_pre = T_place.copy()
        T_pre[2, 3] += c.approach_distance_m
        lo, hi = P.ik.joint_limits()
        qs, cur = [], list(c.q_lift)
        for T in (T_pre, T_place):
            ok, q, _ = P.ik.solve(T, cur)
            if not ok or joint_margin(q, lo, hi) < 0:
                return None, "NO_IK"
            qs.append(q)
            cur = q
        return {"q_pre": qs[0], "q_place": qs[1], "T_pre": T_pre, "T_place": T_place}, ""

    # ------------------------------------------------------------------ one task
    def run_task(self, task, idx=1, text=""):
        """Run one parsed task (cmd.Task) with its own kin-log pair. Returns the task record (dict)."""
        P = self.P
        meta = {"task_idx": idx, "task": task, "command": text or task.text, "cube_name": self.names.get(task.cube, "-"),
                "cube_ids": self.cube_set[task.cube].ids if task.cube in self.cube_set else [],
                "target": task.describe(self.names), "src_pick": "-", "src_target": "-", "stem": "task%02d_%s" % (idx, task.short(self.names))}
        P.task_meta = meta
        try:
            rec = P.run_logged(idx, lambda: self._run_task(task, idx, meta), task_meta=meta)
        finally:
            P.task_meta = None
        rec["task_idx"], rec["task_text"] = idx, task.describe(self.names)
        self.records.append(rec)
        return rec

    def execute(self, tasks, text="", on_status: Optional[Callable] = None, should_stop: Optional[Callable] = None):
        """Run tasks in order. on_status(i, state, rec): state in running/done/failed/skipped. should_stop() is polled
        between tasks ('stop after current task'). A task whose cube / target cube failed earlier is skipped."""
        recs, failed = [], set()
        for i, t in enumerate(tasks, 1):
            if should_stop is not None and should_stop():
                if on_status:
                    for j in range(i, len(tasks) + 1):
                        on_status(j, "skipped", None)
                break
            if t.cube in failed or (t.onto is not None and t.onto in failed):
                if on_status:
                    on_status(i, "skipped", {"detail": "an earlier task for this cube failed"})
                recs.append({"outcome": "SKIPPED", "detail": "an earlier task for this cube failed", "task_idx": i})
                continue
            if on_status:
                on_status(i, "running", None)
            rec = self.run_task(t, i, text)
            recs.append(rec)
            ok = rec.get("outcome") == "SUCCESS"
            if not ok and t.cube is not None:
                failed.add(t.cube)
            if on_status:
                on_status(i, "done" if ok else "failed", rec)
        return recs

    def _blank_rec(self, task, idx):
        w = self.w
        rec = {"run": idx, "true_x": "", "true_y": "", "true_yaw_deg": 0.0, "est_x": "", "est_y": "", "est_yaw_deg": "",
               "pos_err_mm": "", "yaw_err_deg": "", "tags": "", "feasible": "", "total": "", "chosen_yaw_deg": "",
               "chosen_rank": "", "grasp_offset_mm": "", "place_err_mm": "", "place_yaw_err_deg": "", "upright": "",
               "outcome": "", "detail": "", "action": task.action, "cube": task.cube, "zone": task.zone, "onto": task.onto,
               "cube_name": self.names.get(task.cube, "-"), "nb_obstacles": 0, "nb_rejected": 0,
               "src_pick": "-", "src_target": "-", "stack_z_err_mm": ""}
        if task.cube in w.cubes:
            Tt = w.cube_pose_true(task.cube)
            rec.update(true_x=Tt[0, 3], true_y=Tt[1, 3], true_yaw_deg=math.degrees(cube_yaw(Tt)))
        else:
            rec.update(true_x=0.0, true_y=0.0)
        return rec

    def _run_home(self, task, idx, rec):
        P, w = self.P, self.w
        P.phase = "home"
        if w.cube_attached is not None:
            w.detach_cube()
        if P.args.gui:
            w.move_gripper(0.0, name="home_open")
            P.move(P.q_park, "home")
        else:
            w.set_gripper(0.0)
            w.set_arm(P.q_park)
        rec["outcome"] = "SUCCESS"
        rec["detail"] = "arm at park pose, gripper open"
        return rec

    def _run_task(self, task, idx, meta):
        P, w, a = self.P, self.w, self.P.args
        t0 = time.time()
        rec = self._blank_rec(task, idx)
        P._cur_rec, P._ik_targets, P._cands = rec, [], []
        try:
            return self._run_task_inner(task, idx, meta, rec, t0)
        except TaskError as e:
            rec["outcome"] = "PRECONDITION_FAIL"
            rec["detail"] = str(e)
            P.log("  task %d: %s" % (idx, e))
            return rec

    def _run_task_inner(self, task, idx, meta, rec, t0):
        P, w, a = self.P, self.w, self.P.args
        log = P.log
        if task.action == "home":
            return self._run_home(task, idx, rec)
        if task.action in cmd.BELT_ACTIONS:
            if task.action == "pick_belt":
                return self.belt.run_pick(task, idx, meta, rec, t0)
            if task.action == "feed":
                return self.belt.feed(task, rec)
            return self.belt.run_simple(task, rec)
        A = task.cube
        if A not in w.cubes:
            raise TaskError("%s cube is not in the scene" % self.names.get(A, A))
        on_a = [k for k, v in self.support.items() if v == A and k in w.cubes]
        if on_a:
            raise TaskError("%s cube has %s on top: it cannot be picked" % (self.names[A], self.names[on_a[0]]))
        edge = self.edge

        # ---- perceive everything in view
        P.phase = "perceive"
        if a.gui:
            P.move(P.q_park, "to_park")
        else:
            w.set_arm(P.q_park)
        w.set_gripper(0.0)
        ests, info = self.perceive_all()
        pose_a, src_a = self.locate(A, ests)
        rec["src_pick"], meta["src_pick"] = src_a, src_a
        head = ["task %02d  %s" % (idx, task.describe(self.names))]
        if pose_a is None:
            rec["outcome"] = "PERCEPTION_FAIL"
            rec["detail"] = "%s cube not seen and no recorded pose" % self.names[A]
            P.show(info["img"], head + ["PERCEPTION_FAIL"], info["dets"])
            self._save(idx)
            return rec
        est_a = ests.get(A)
        rec["tags"] = ",".join(map(str, est_a["ids"])) if est_a else ""
        ex, ey, ez, eyaw = pose_a["x"], pose_a["y"], pose_a["z"], pose_a["yaw"]
        Tt = w.cube_pose_true(A)
        xt, yt, yawt = Tt[0, 3], Tt[1, 3], cube_yaw(Tt)
        rec.update(est_x=ex, est_y=ey, est_yaw_deg=math.degrees(eyaw), pos_err_mm=1000 * math.hypot(ex - xt, ey - yt),
                   yaw_err_deg=math.degrees(wrap90(eyaw - yawt)))
        lines = head + ["%s est x=%.3f y=%.3f z=%.3f  err %.1f mm   src: %s" % (
            self.names[A], ex, ey, ez, rec["pos_err_mm"], "camera" if est_a else "RECORDED")]
        T_cc = est_a["T_cc"] if est_a else None
        P.show(info["img"], lines, info["dets"], T_cc)

        # ---- target
        if task.action == "place":
            sx, sy, zone = self.pick_slot(task.zone, A, ests)
            tgt = {"x": sx, "y": sy, "z": self.table_z + edge / 2 + 0.002, "yaw": zone["target"]["yaw_rad"]}
            src_t = "zone %d slot (%.3f, %.3f) from workspace.yaml place_zones" % (task.zone, sx, sy)
            ref = None
        else:
            B = task.onto
            if B not in w.cubes:
                raise TaskError("%s cube (target of the stack) is not in the scene" % self.names.get(B, B))
            over_b = [k for k, v in self.support.items() if v == B and k != A and k in w.cubes]
            if over_b:
                raise TaskError("%s cube already has %s on top" % (self.names[B], self.names[over_b[0]]))
            pose_b, src_b = self.locate(B, ests)
            if pose_b is None:
                raise TaskError("%s cube not seen and no recorded pose" % self.names[B])
            lvl = self.level_of(pose_b["z"])
            if lvl + 1 > M["max_level"]:
                raise TaskError("stack would be %d cubes high (maximum %d)" % (lvl + 2, M["max_level"] + 1))
            zb = self.table_z + edge / 2 + lvl * edge
            tgt = {"x": pose_b["x"], "y": pose_b["y"], "z": zb + edge + self.gap, "yaw": pose_b["yaw"]}
            src_t = "stack on %s: B pose from %s" % (self.names[B], src_b)
            ref = B
        rec["src_target"], meta["src_target"] = src_t, src_t
        log("  pick pose source : %s" % src_a)
        log("  target           : %s -> (%.3f, %.3f, %.3f)" % (src_t, tgt["x"], tgt["y"], tgt["z"]))

        # ---- candidates (neighbour filter)
        obs_pick = self.obstacles(ests, {A}, ez)
        rec["nb_obstacles"] = len(obs_pick)
        cands = generate((ex, ey, ez, eyaw), P.cfg["gripper"], P.cfg["planning"], P.ws, "task%02d" % idx, edge,
                         obstacles=obs_pick, extra_boxes=self.zone_boxes)
        evaluate(cands, P.ik, P.q_ready)
        rec["nb_rejected"] = sum(c.rejection_reason == "NEIGHBOR_COLLISION" for c in cands)
        P._cands = [(c.candidate_id, c.rejection_reason or "ok", "", math.degrees(c.grasp_yaw_rad), c.score.total)
                    for c in cands]
        res = rank(cands, P.cfg["ranking"])
        log(format_table(cands))
        if obs_pick:
            log("  neighbours considered: %s -> %d of %d candidates rejected NEIGHBOR_COLLISION" % (
                ", ".join(o.name for o in obs_pick), rec["nb_rejected"], len(cands)))
        nfeas = len(res.ranked)
        rec["feasible"] = "%d/%d" % (nfeas, len(cands))
        if nfeas == 0 or res.selected_id is None:
            rec["outcome"] = "NO_FEASIBLE"
            why = sorted({c.rejection_reason for c in cands if c.rejection_reason})
            rec["detail"] = "no feasible grasp (%s)" % ", ".join(why)
            P.show(info["img"], lines + ["NO_FEASIBLE " + ",".join(why)], info["dets"], T_cc)
            self._save(idx)
            return rec

        # ---- place feasibility (walk down the ranking)
        T_est = T_of(rz(eyaw), [ex, ey, ez])
        obs_place = self.obstacles(ests, {A}, tgt["z"])
        order = [c for c in res.ranked if c.candidate_id == res.selected_id] + \
                [c for c in res.ranked if c.candidate_id != res.selected_id]
        chosen, plan_fail = None, {}
        for rank_i, c in enumerate(order):
            plan, why = self.plan_place(c, T_est, eyaw, tgt, obs_place)
            if plan is not None:
                chosen, chosen_plan, chosen_rank = c, plan, rank_i
                break
            plan_fail[why] = plan_fail.get(why, 0) + 1
        if chosen is None:
            rec["outcome"] = "NO_FEASIBLE"
            rec["feasible"] += " (place: %s)" % ", ".join("%s x%d" % kv for kv in plan_fail.items())
            rec["detail"] = "no candidate can place: " + ", ".join("%s x%d" % kv for kv in plan_fail.items())
            P.show(info["img"], lines + ["NO_FEASIBLE (place)"], info["dets"], T_cc)
            self._save(idx)
            return rec
        c = chosen
        rec["chosen_id"] = c.candidate_id
        rec.update(chosen_yaw_deg=math.degrees(c.grasp_yaw_rad), chosen_rank=chosen_rank + 1, total=c.score.total)
        P._ik_targets = [
            ("pregrasp", pose_to_matrix(c.pregrasp_pose), c.q_start, c.q_pregrasp),
            ("grasp", pose_to_matrix(c.grasp_pose), c.q_pregrasp, c.q_grasp),
            ("lift", pose_to_matrix(c.lift_pose), c.q_grasp, c.q_lift),
            ("place_pre", chosen_plan["T_pre"], c.q_lift, chosen_plan["q_pre"]),
            ("place", chosen_plan["T_place"], chosen_plan["q_pre"], chosen_plan["q_place"])]
        log("  chosen %s (rank %d of %d feasible), total %.3f" % (c.candidate_id, chosen_rank + 1, nfeas, c.score.total))
        P.show(info["img"], lines + ["chosen %s yaw %.0f deg (%d feasible)" % (
            c.candidate_id, math.degrees(c.grasp_yaw_rad), nfeas)], info["dets"], T_cc)

        # ---- execute (same motion sequence as Pipeline._run_one)
        P.phase = "plan"
        g_open, g_closed = 0.0, P.close_ang
        line_down = P.plan_line("descend", c.q_pregrasp, c.q_grasp, g_open)
        line_up = P.plan_line("lift", c.q_grasp, c.q_lift, g_closed)
        line_pdown = P.plan_line("place_descend", chosen_plan["q_pre"], chosen_plan["q_place"], g_closed)
        line_pup = P.plan_line("place_retreat", chosen_plan["q_place"], chosen_plan["q_pre"], g_open)
        for sg in (line_down, line_up, line_pdown, line_pup):
            if sg.info.get("fallback"):
                log("  [cartesian fallback] %s -> joint quintic: %s" % (sg.name, sg.info["fallback"]))
        w.activate_cube(A)
        P.phase = "pregrasp"
        P.move(P.q_ready, "to_ready")
        cube0 = w.cube_pose_true(A)[:3, 3].copy()
        others0 = {n: w.cube_pose_true(n)[:2, 3].copy() for n in w.cubes if n != A}
        P.move(c.q_pregrasp, "to_pregrasp")
        P.phase = "grasp"
        P.play(line_down)
        moved_pre = float(np.linalg.norm(w.cube_pose_true(A)[:2, 3] - cube0[:2])) * 1000
        P.gripper_to(P.close_ang, "close_gripper")
        T_tcp_cube = Tinv(w.tcp_T()) @ w.cube_pose_true(A)
        rec["grasp_offset_mm"] = float(np.linalg.norm(T_tcp_cube[:3, 3]) * 1000)
        w.attach_cube()
        P.phase = "lift"
        P.play(line_up)
        P.phase = "place"
        P.move(chosen_plan["q_pre"], "transfer")
        P.play(line_pdown)
        w.detach_cube()
        P.gripper_to(0.0, "open_gripper")
        P.phase = "retreat"
        P.play(line_pup)
        P.move(P.q_ready, "to_ready_2")
        P.move(P.q_park, "to_park_end")
        for _ in range(M["stack_settle_steps"] if task.action == "stack" else M["settle_steps"]):
            w.p.stepSimulation()

        # ---- score (TRUE poses: scoring only)
        Tf = w.cube_pose_true(A)
        up = float(Tf[2, 2])
        rec["upright"] = bool(up >= M["upright_cos"])
        tol = 1000 * float(P.ws["place_tol_m"])
        if task.action == "place":
            perr = 1000 * math.hypot(Tf[0, 3] - tgt["x"], Tf[1, 3] - tgt["y"])
            yerr = math.degrees(wrap90(cube_yaw(Tf) - tgt["yaw"]))
            ok = perr < tol and rec["upright"]
            rec["outcome"] = "SUCCESS" if ok else "PLACE_FAIL"
            self.rec[A] = {"x": tgt["x"], "y": tgt["y"], "z": self.table_z + edge / 2, "yaw": tgt["yaw"],
                           "src": "last commanded place pose (zone %d)" % task.zone, "zone": task.zone}
            self.support[A] = None
        else:
            Tb = w.cube_pose_true(ref)                       # TRUE support pose, scoring only
            perr = 1000 * math.hypot(Tf[0, 3] - Tb[0, 3], Tf[1, 3] - Tb[1, 3])
            zerr = 1000 * (Tf[2, 3] - (Tb[2, 3] + edge))
            rec["stack_z_err_mm"] = zerr
            yerr = math.degrees(wrap90(cube_yaw(Tf) - cube_yaw(Tb)))
            ok = perr < tol and rec["upright"] and abs(zerr) < 1000 * M["stack_z_tol_m"]
            rec["outcome"] = "SUCCESS" if ok else "PLACE_FAIL"
            self.rec[A] = {"x": tgt["x"], "y": tgt["y"], "z": tgt["z"] - self.gap, "yaw": tgt["yaw"],
                           "src": "last commanded stack pose (on %s)" % self.names[ref],
                           "zone": self.rec.get(ref, {}).get("zone")}
            self.support[A] = ref
        rec.update(place_err_mm=perr, place_yaw_err_deg=yerr)
        # a cube that moved other cubes (finger contact) is reported, not hidden
        for n, p0 in others0.items():
            d = float(np.linalg.norm(w.cube_pose_true(n)[:2, 3] - p0)) * 1000
            if d > 2.0:
                log("  note: %s cube moved %.1f mm during this task" % (self.names[n], d))
                rec["detail"] += "%s moved %.1f mm; " % (self.names[n], d)
        if moved_pre > 2.0:
            log("  note: cube moved %.1f mm before grasp (finger contact)" % moved_pre)
        rec["moved_before_grasp_mm"] = moved_pre
        log("  %s err %.1f mm (tol %.0f)%s, upright=%s -> %s  [%.1fs]" % (
            "place" if task.action == "place" else "stack xy", perr, tol,
            "" if task.action == "place" else ", dz %.1f mm" % rec["stack_z_err_mm"], rec["upright"], rec["outcome"],
            time.time() - t0))
        P.show(info["img"], lines + ["chosen %s | grasp offset %.1f mm | err %.1f mm | %s" % (
            c.candidate_id, rec["grasp_offset_mm"], perr, rec["outcome"])], info["dets"], T_cc)
        self._save(idx)
        if a.gui:
            P.show(P.grab(), [rec["outcome"] + " - %s, err %.1f mm" % (task.describe(self.names), perr)])
        return rec

    def _save(self, idx):
        P = self.P
        if P.last_frame is not None:
            os.makedirs(os.path.join(P.args.out, "frames"), exist_ok=True)
            cv2.imwrite(os.path.join(P.args.out, "frames", "task_%02d.png" % idx), P.last_frame)
