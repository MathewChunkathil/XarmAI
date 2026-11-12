"""Tests for candidate_evaluator.rank. Run: python tests/unit/test_rank.py"""
import copy
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "candidate_evaluator"))

from candidate_evaluator import FakeIK, evaluate, format_table, generate, load_yaml, rank  # noqa: E402
from candidate_evaluator.types import STATUS_REJECTED, STATUS_SELECTED  # noqa: E402

GRIP = load_yaml(ROOT / "config/gripper/gripper.yaml")
PLAN = load_yaml(ROOT / "config/planning/planning.yaml")
WS = load_yaml(ROOT / "config/workspace/workspace.yaml")
RK = load_yaml(ROOT / "config/planning/ranking.yaml")
SEED = [0.0, -0.5, -0.5, 0.0, 0.0]


def make(yaw=0.3, plan=None, backend=None):
    c = generate((0.36, 0.05, 0.025, yaw), GRIP, plan or PLAN, WS, "r01")
    evaluate(c, backend or FakeIK(lower=[-3.0] * 5, upper=[3.0] * 5), SEED)
    return c


def test_norm_in_unit_interval_and_total():
    c = make(plan={**PLAN, "yaw_offsets_deg": [0.0, 20.0]})
    for i, x in enumerate(c):
        x.min_clearance_m = 0.01 * i
        x.predicted_duration_s = 5.0 + i
    r = rank(c, RK)
    assert r.ranked
    for x in r.ranked:
        assert all(0.0 <= v <= 1.0 for v in x.score.normalised)
        tot = sum(w * n for w, n in zip(x.score.weights, x.score.normalised))
        assert abs(tot - x.score.total) < 1e-12


def test_missing_terms_zero_with_note():
    r = rank(make(), RK)
    s = r.ranked[0].score
    assert s.normalised[1] == 0.0 and s.normalised[4] == 0.0
    assert any("clearance" in n for n in s.notes)


def test_deterministic_and_tiebreak():
    a = rank(make(), RK)
    b = rank(copy.deepcopy(make()), RK)
    assert [x.candidate_id for x in a.ranked] == [x.candidate_id for x in b.ranked]
    assert a.selected_id == b.selected_id
    be = FakeIK(q_fn=lambda T, s: [0.1, 0.2, 0.3, 0.4, 0.5])   # identical scores -> exact ties
    r = rank(make(backend=be), RK)
    assert r.selected_id == "r01-c00"
    assert [x.candidate_id for x in r.ranked] == sorted(x.candidate_id for x in r.ranked)


def test_selected_status_and_rejected_not_ranked():
    c = make(backend=FakeIK(reach_fn=lambda T: T[1, 0] < 0.5))
    r = rank(c, RK)
    assert all(x.status != STATUS_REJECTED for x in r.ranked) and len(r.rejected) >= 1
    sel = [x for x in c if x.status == STATUS_SELECTED]
    assert len(sel) == 1 and sel[0].candidate_id == r.selected_id


def _q_fn(T, seed):
    return [0.0, 0.0, 0.0, 0.0, math.atan2(T[1, 0], T[0, 0])]


def test_profile_a_picks_k0():
    # yaw_c=2.0: k=0 has |J5|~2.0 (large motion/margin penalty), another k is better -> B differs from A
    mk = lambda: make(yaw=2.0, backend=FakeIK(q_fn=_q_fn, lower=[-4] * 5, upper=[4] * 5))
    assert rank(mk(), RK, profile="A").selected_id == "r01-c00"
    assert rank(mk(), RK, profile="B").selected_id != "r01-c00"


def test_profile_a_infeasible_k0_selects_none():
    c = make(backend=FakeIK(reach_fn=lambda T: abs(T[0, 0] - math.cos(0.3)) > 1e-6))
    r = rank(c, RK, profile="A")
    assert r.selected_id is None and r.ranked


def test_format_table():
    c = make(backend=FakeIK(reach_fn=lambda T: T[1, 0] < 0.5))
    rank(c, RK)
    t = format_table(c)
    assert "r01-c00" in t and "NO_IK" in t and "SELECTED" in t and "total" in t
    print(t)


if __name__ == "__main__":
    n = 0
    for k, v in list(globals().items()):
        if k.startswith("test_") and callable(v):
            v()
            n += 1
            print("PASS", k)
    print(f"{n} passed")

