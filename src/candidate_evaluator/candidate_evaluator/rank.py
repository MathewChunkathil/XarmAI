"""Candidate ranking per IMPLEMENTATION_PLAN 5.7. total = sum(w * norm); LOWER wins.

Missing-data rule (documented): a term whose raw value is None is normalised to 0 (neutral,
no penalty) and a note is added to ScoreBreakdown.notes. Ties are broken by candidate_id.
Profile A bypasses ranking and selects the candidate with yaw_index_k == profile_a_k
(and yaw offset 0) if it is feasible.
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np

from .types import (GraspCandidate, SCORE_NAMES, STATUS_NAMES, STATUS_REJECTED,
                    STATUS_SELECTED, ScoreBreakdown)


@dataclass
class RankResult:
    ranked: List[GraspCandidate] = field(default_factory=list)    # feasible, best first
    rejected: List[GraspCandidate] = field(default_factory=list)
    selected_id: Optional[str] = None
    profile: str = "B"
    notes: List[str] = field(default_factory=list)


def _clip01(v: float) -> float:
    return float(min(max(v, 0.0), 1.0))


def motion_raw(c: GraspCandidate, joint_weights: Sequence[float]) -> Optional[float]:
    """sum_j w_j * sum_t |dq_j| over start->pregrasp->grasp->lift (IK waypoints; arm segments only)."""
    qs = [c.q_start, c.q_pregrasp, c.q_grasp, c.q_lift]
    if any(q is None for q in qs):
        return None
    a = np.asarray(qs, float)
    d = np.abs(np.diff(a, axis=0)).sum(axis=0)
    return float(np.dot(np.asarray(joint_weights, float)[: len(d)], d))


def _raw_terms(c: GraspCandidate, cfg: dict) -> List[Optional[float]]:
    grasp = c.grasp_axis_error_rad + float(cfg.get("grasp_cov_trace_gain", 0.0)) * c.pose_cov_trace
    return [motion_raw(c, cfg["motion_joint_weights"]), c.min_clearance_m,
            c.joint_margin_min, grasp, c.predicted_duration_s]


def rank(candidates: List[GraspCandidate], ranking_cfg: dict, profile: Optional[str] = None) -> RankResult:
    profile = profile or ranking_cfg.get("profile", "B")
    w = [float(ranking_cfg["weights"][n]) for n in SCORE_NAMES]
    feas = [c for c in candidates if c.status != STATUS_REJECTED]
    rej = [c for c in candidates if c.status == STATUS_REJECTED]
    raws = {c.candidate_id: _raw_terms(c, ranking_cfg) for c in feas}
    max_motion = max([r[0] for r in raws.values() if r[0] is not None], default=0.0)
    max_time = max([r[4] for r in raws.values() if r[4] is not None], default=0.0)
    d_ref = float(ranking_cfg["clearance_d_ref_m"])
    m_max = float(ranking_cfg["kinematic_margin_max"])
    g_ref = float(ranking_cfg["grasp_angle_ref_rad"])

    for c in feas:
        raw = raws[c.candidate_id]
        notes, norm = [], [0.0] * 5
        for i, name in enumerate(SCORE_NAMES):
            if raw[i] is None:
                notes.append(f"{name}: raw missing -> normalised 0")
        if raw[0] is not None:
            norm[0] = _clip01(raw[0] / max_motion) if max_motion > 0 else 0.0
        if raw[1] is not None:
            norm[1] = _clip01(1.0 - min(raw[1] / d_ref, 1.0))
        if raw[2] is not None:
            norm[2] = _clip01(1.0 - raw[2] / m_max)
        norm[3] = _clip01(raw[3] / g_ref)
        if raw[4] is not None:
            norm[4] = _clip01(raw[4] / max_time) if max_time > 0 else 0.0
        c.score = ScoreBreakdown(names=list(SCORE_NAMES), raw=list(raw), normalised=norm,
                                 weights=list(w), total=float(sum(a * b for a, b in zip(w, norm))),
                                 notes=notes)
    ranked = sorted(feas, key=lambda c: (c.score.total, c.candidate_id))
    res = RankResult(ranked=ranked, rejected=rej, profile=profile)
    if not ranked:
        res.notes.append("no feasible candidate")
        return res
    if profile == "A":
        k = int(ranking_cfg.get("profile_a_k", 0))
        pick = [c for c in ranked if c.yaw_index_k == k and c.yaw_offset_deg == 0.0]
        if not pick:
            res.notes.append(f"profile A: fixed candidate k={k} infeasible, none selected")
            return res
        sel = pick[0]
    else:
        sel = ranked[0]
    sel.status = STATUS_SELECTED
    res.selected_id = sel.candidate_id
    return res


def format_table(candidates: List[GraspCandidate]) -> str:
    hdr = (f"{'id':<14}{'yaw_deg':>8} {'status':<10}{'reason':<17}"
           f"{'motion':>7}{'clear':>7}{'kinem':>7}{'grasp':>7}{'time':>7}{'total':>8}")
    lines = [hdr, "-" * len(hdr)]
    for c in sorted(candidates, key=lambda c: c.candidate_id):
        s = c.score
        feasible = c.status != STATUS_REJECTED
        terms = "".join(f"{v:7.3f}" for v in s.normalised) if feasible else " " * 35
        tot = f"{s.total:8.3f}" if feasible else " " * 8
        lines.append(f"{c.candidate_id:<14}{math.degrees(c.grasp_yaw_rad):8.1f} "
                     f"{STATUS_NAMES.get(c.status, str(c.status)):<10}{c.rejection_reason:<17}{terms}{tot}")
    return "\n".join(lines)
