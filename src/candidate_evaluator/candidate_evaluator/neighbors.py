"""Neighbour-cube collision check for grasp / place poses: 2D footprint of the OPEN fingers vs the footprints of
the other cubes (pure Python, no simulator imports).

Tool frame (generate.py): tool z down, tool y = closing axis, tool x = along the finger width. For a grasp
yaw psi the tool axes in the world xy plane are  x_t = (cos psi, sin psi),  y_t = (sin psi, -cos psi).
The two open fingers are boxes at tool-y = +/-(inner + outer)/2, half size (finger_half_width, (outer - inner)/2).
A candidate is rejected (reason NEIGHBOR_COLLISION) if either finger box, grown by `safety_margin_m`, overlaps the
footprint of another cube. The check is 2D on purpose (conservative: ignores finger / cube heights).
Footprint numbers: config/gripper/gripper.yaml finger_footprint (xArm Gripper stand-in, PLACEHOLDER).
"""
import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from .types import STATUS_REJECTED

REASON = "NEIGHBOR_COLLISION"


@dataclass
class Obstacle:
    """Another cube seen from above: centre (x, y), yaw [rad], edge [m] (square footprint)."""
    x: float
    y: float
    yaw: float = 0.0
    edge: float = 0.05
    name: str = ""


Rect = Tuple[float, float, float, float, float]       # cx, cy, half_u, half_v, angle of u axis


def cube_rect(o: Obstacle) -> Rect:
    return (o.x, o.y, o.edge / 2, o.edge / 2, o.yaw)


def finger_rects(x: float, y: float, psi: float, fp: Dict, margin: Optional[float] = None) -> List[Rect]:
    """The two open-finger boxes (grown by the safety margin) for a TCP at (x, y) with grasp yaw psi."""
    inner, outer = float(fp["open_inner_half_gap_m"]), float(fp["finger_outer_half_m"])
    hw = float(fp["finger_half_width_m"])
    m = float(fp.get("safety_margin_m", 0.0)) if margin is None else margin
    mid, hy = (inner + outer) / 2, (outer - inner) / 2
    yt = np.array([math.sin(psi), -math.cos(psi)])
    out = []
    for sgn in (+1, -1):
        c = np.array([x, y]) + sgn * mid * yt
        out.append((float(c[0]), float(c[1]), hw + m, hy + m, psi))      # u axis = tool x (angle psi)
    return out


def _axes(r: Rect):
    c, s = math.cos(r[4]), math.sin(r[4])
    return np.array([c, s]), np.array([-s, c])


def rects_overlap(a: Rect, b: Rect) -> bool:
    """Separating-axis test of two oriented rectangles (touching counts as no overlap)."""
    ua, va = _axes(a)
    ub, vb = _axes(b)
    d = np.array([b[0] - a[0], b[1] - a[1]])
    for ax in (ua, va, ub, vb):
        ra = a[2] * abs(ax @ ua) + a[3] * abs(ax @ va)
        rb = b[2] * abs(ax @ ub) + b[3] * abs(ax @ vb)
        if abs(d @ ax) >= ra + rb - 1e-12:
            return False
    return True


def tool_yaw_of(T: np.ndarray) -> float:
    """Grasp yaw psi of a tool-down TCP pose (tool x axis heading)."""
    return float(math.atan2(T[1, 0], T[0, 0]))


def neighbor_hit(x: float, y: float, psi: float, obstacles: Iterable[Obstacle], fp: Dict,
                 margin: Optional[float] = None) -> Optional[Obstacle]:
    """First obstacle whose footprint overlaps an open finger box (None = clear)."""
    fr = finger_rects(x, y, psi, fp, margin)
    for o in obstacles:
        cr = cube_rect(o)
        if any(rects_overlap(f, cr) for f in fr):
            return o
    return None


def filter_neighbors(candidates, obstacles: Sequence[Obstacle], gripper_cfg: Dict) -> List:
    """Mark candidates whose open fingers would hit another cube: status REJECTED, reason NEIGHBOR_COLLISION.
    Already rejected candidates keep their first reason. Returns the same list. No `finger_footprint` in the
    gripper config -> no-op (legacy behaviour)."""
    fp = gripper_cfg.get("finger_footprint")
    if not obstacles or not fp:
        return candidates
    for c in candidates:
        if c.status == STATUS_REJECTED:
            continue
        gx, gy = c.grasp_pose.position[0], c.grasp_pose.position[1]
        hit = neighbor_hit(gx, gy, c.grasp_yaw_rad, obstacles, fp)
        if hit is not None:
            c.status = STATUS_REJECTED
            c.rejection_reason = REASON
    return candidates
