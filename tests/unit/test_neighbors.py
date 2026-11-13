"""Neighbour-cube finger collision filter (2D footprint check). Run: python -m pytest tests/unit/test_neighbors.py"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "candidate_evaluator"))

from candidate_evaluator import (FakeIK, Obstacle, evaluate, filter_neighbors, finger_rects,  # noqa: E402
                                 generate, load_yaml, neighbor_hit, rects_overlap)
from candidate_evaluator.types import STATUS_REJECTED  # noqa: E402

GRIP = load_yaml(ROOT / "config/gripper/gripper.yaml")
PLAN = load_yaml(ROOT / "config/planning/planning.yaml")
WS = load_yaml(ROOT / "config/workspace/workspace.yaml")
FP = GRIP["finger_footprint"]
Z = 0.025


def reasons(c):
    return {x.candidate_id[-3:]: x.rejection_reason for x in c}


def test_rects_overlap_basic():
    a = (0, 0, 0.5, 0.5, 0.0)
    assert rects_overlap(a, (0.9, 0, 0.5, 0.5, 0.0))
    assert not rects_overlap(a, (1.1, 0, 0.5, 0.5, 0.0))
    # rotated square (diamond) corner reaches 0.707: overlaps at 1.1 centre distance? 1.1 - 0.707 = 0.39 < 0.5 yes
    assert rects_overlap(a, (1.1, 0, 0.5, 0.5, math.pi / 4))
    assert not rects_overlap(a, (1.3, 0, 0.5, 0.5, math.pi / 4))


def test_finger_boxes_follow_closing_axis():
    # psi = 0: tool y = (0, -1): fingers sit at +/- y of the TCP, at (inner+outer)/2
    fr = finger_rects(0.3, 0.1, 0.0, FP)
    ys = sorted(r[1] for r in fr)
    mid = (FP["open_inner_half_gap_m"] + FP["finger_outer_half_m"]) / 2
    assert abs(ys[0] - (0.1 - mid)) < 1e-9 and abs(ys[1] - (0.1 + mid)) < 1e-9
    assert all(abs(r[0] - 0.3) < 1e-9 for r in fr)
    # psi = 90 deg: closing axis along x
    fr = finger_rects(0.3, 0.1, math.pi / 2, FP)
    xs = sorted(r[0] for r in fr)
    assert abs(xs[0] - (0.3 - mid)) < 1e-9 and abs(xs[1] - (0.3 + mid)) < 1e-9


def test_neighbor_beside_closing_axis_is_hit_but_not_along_finger_width():
    # neighbour 8 cm away along y: in the finger zone for psi = 0 (closing axis = y), clear for psi = 90 deg
    o = [Obstacle(0.35, 0.08, 0.0, 0.05)]
    assert neighbor_hit(0.35, 0.0, 0.0, o, FP) is o[0]
    assert neighbor_hit(0.35, 0.0, math.pi / 2, o, FP) is None


def test_far_neighbor_never_hit():
    o = [Obstacle(0.35, 0.30, 0.3, 0.05)]
    for k in range(4):
        assert neighbor_hit(0.35, 0.0, k * math.pi / 2, o, FP) is None


def test_generate_marks_neighbor_collision():
    # cube yaw 0, neighbour 8 cm in +y: candidates with closing axis along y (k=0 and k=2) must be rejected
    c = generate((0.35, 0.0, Z, 0.0), GRIP, PLAN, WS, "n01", obstacles=[Obstacle(0.35, 0.08, 0.0, 0.05)])
    r = reasons(c)
    assert sum(v == "NEIGHBOR_COLLISION" for v in r.values()) == 2, r
    assert sum(x.status != STATUS_REJECTED for x in c) == 2
    # the survivors are the ones whose closing axis is x
    for x in c:
        if x.status != STATUS_REJECTED:
            assert abs(math.sin(x.grasp_yaw_rad)) < 1e-9 or abs(math.cos(x.grasp_yaw_rad)) < 1e-9


def test_no_obstacles_is_legacy_behaviour():
    a = generate((0.36, 0.08, Z, 0.4), GRIP, PLAN, WS, "t01")
    b = generate((0.36, 0.08, Z, 0.4), GRIP, PLAN, WS, "t01", obstacles=[])
    assert [x.status for x in a] == [x.status for x in b] and all(x.rejection_reason == "" for x in b)


def test_neighbors_all_around_reject_everything():
    obs = [Obstacle(0.35 + dx, 0.0 + dy, 0.0, 0.05) for dx, dy in ((0.08, 0), (-0.08, 0), (0, 0.08), (0, -0.08))]
    c = generate((0.35, 0.0, Z, 0.0), GRIP, PLAN, WS, "n02", obstacles=obs)
    assert all(x.status == STATUS_REJECTED and x.rejection_reason == "NEIGHBOR_COLLISION" for x in c)


def test_keeps_first_rejection_reason():
    c = generate((0.9, 0.0, Z, 0.0), GRIP, PLAN, WS, "n03", obstacles=[Obstacle(0.9, 0.05, 0.0, 0.05)])
    assert all(x.rejection_reason == "OUT_OF_WORKSPACE" for x in c)


def test_evaluate_skips_neighbor_rejected():
    ik = FakeIK()
    c = generate((0.35, 0.0, Z, 0.0), GRIP, PLAN, WS, "n04", obstacles=[Obstacle(0.35, 0.08, 0.0, 0.05)])
    evaluate(c, ik, [0, -0.5, -0.5, 0, 0])
    assert ik.calls == 2 * 3        # only the 2 surviving candidates are IK-checked (3 poses each)


def test_extra_boxes_allow_zone_pick():
    c = generate((0.32, -0.29, Z, 0.0), GRIP, PLAN, WS, "z01")
    assert all(x.rejection_reason == "OUT_OF_WORKSPACE" for x in c)
    c = generate((0.32, -0.29, Z, 0.0), GRIP, PLAN, WS, "z02", extra_boxes=[WS["place_zone"]])
    assert all(x.rejection_reason == "" for x in c)


def test_no_footprint_config_is_noop():
    g = {k: v for k, v in GRIP.items() if k != "finger_footprint"}
    c = generate((0.35, 0.0, Z, 0.0), g, PLAN, WS, "n05", obstacles=[Obstacle(0.35, 0.08, 0.0, 0.05)])
    assert all(x.rejection_reason == "" for x in c)
    assert filter_neighbors(c, [], GRIP) is c
