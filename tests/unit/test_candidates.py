"""Tests for candidate_evaluator.generate / ik_backend.
Run: python tests/unit/test_candidates.py   (or python -m pytest)."""
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "candidate_evaluator"))

from candidate_evaluator import FakeIK, evaluate, generate, load_yaml  # noqa: E402
from candidate_evaluator.generate import pose_to_matrix, wrap_pi  # noqa: E402
from candidate_evaluator.types import STATUS_GENERATED, STATUS_REJECTED  # noqa: E402

GRIP = load_yaml(ROOT / "config/gripper/gripper.yaml")
PLAN = load_yaml(ROOT / "config/planning/planning.yaml")
WS = load_yaml(ROOT / "config/workspace/workspace.yaml")
CUBE_Z = 0.025


def gen(pose=(0.36, 0.08, CUBE_Z, 0.4), plan=None, grip=None):
    return generate(pose, grip or GRIP, plan or PLAN, WS, "t01")


def test_ids_and_count():
    c = gen()
    assert [x.candidate_id for x in c] == [f"t01-c{i:02d}" for i in range(4)]
    c8 = gen(plan={**PLAN, "yaw_offsets_deg": [0.0, 10.0]})
    assert len(c8) == 8 and c8[7].candidate_id == "t01-c07"


def test_tool_z_down():
    for c in gen(plan={**PLAN, "yaw_offsets_deg": [-7.0, 0.0, 12.0]}):
        for p in (c.grasp_pose, c.pregrasp_pose, c.lift_pose):
            R = pose_to_matrix(p)[:3, :3]
            assert np.max(np.abs(R[:, 2] - [0, 0, -1])) < 1e-9
            assert abs(np.linalg.det(R) - 1) < 1e-9


def test_yaws_spaced_90():
    c = gen()
    yaws = [x.grasp_yaw_rad for x in c]
    for a, b in zip(yaws, yaws[1:] + yaws[:1]):
        assert abs(wrap_pi(b - a) - math.pi / 2) < 1e-9
    assert abs(wrap_pi(yaws[0] - 0.4)) < 1e-9
    assert all(x.grasp_axis_error_rad < 1e-9 for x in c)


def test_pregrasp_above_grasp():
    for c in gen():
        assert abs(c.pregrasp_pose.position[2] - c.grasp_pose.position[2] - c.approach_distance_m) < 1e-12
        assert c.lift_pose.position[2] > c.grasp_pose.position[2]
        assert c.pregrasp_pose.position[:2] == c.grasp_pose.position[:2]


def test_out_of_workspace_rejected():
    for pose in [(0.10, 0.0, CUBE_Z, 0.0), (0.35, 0.40, CUBE_Z, 0.0)]:
        c = gen(pose)
        assert all(x.status == STATUS_REJECTED and x.rejection_reason == "OUT_OF_WORKSPACE" for x in c)


def test_gripper_width_rejected():
    c = gen(grip={**GRIP, "max_open_m": 0.060})
    assert all(x.rejection_reason == "GRIPPER_WIDTH" for x in c)


def test_pregrasp_below_table_rejected():
    c = gen(plan={**PLAN, "approach_distance_m": 0.0}, pose=(0.36, 0.0, -0.1, 0.0))
    assert all(x.rejection_reason == "COLLISION" for x in c)


def test_ok_cube_generated_with_widths():
    c = gen()
    assert all(x.status == STATUS_GENERATED and x.rejection_reason == "" for x in c)
    assert abs(c[0].gripper_open_width_m - 0.065) < 1e-12
    assert {x.contact_faces for x in c} == {"+x/-x", "+y/-y"}


def test_evaluate_no_ik_and_joint_limit():
    c = gen((0.36, 0.0, CUBE_Z, 0.0))
    be = FakeIK(reach_fn=lambda T: not (T[1, 0] > 0.5))   # unreachable for psi=+90deg only
    evaluate(c, be, [0.0] * 5)
    reasons = [x.rejection_reason for x in c]
    assert reasons.count("NO_IK") == 1 and sum(x.status == STATUS_GENERATED for x in c) == 3
    ok = [x for x in c if x.status == STATUS_GENERATED][0]
    assert ok.q_pregrasp and ok.q_grasp and ok.q_lift and 0 <= ok.joint_margin_min <= 0.5
    c = gen((0.36, 0.0, CUBE_Z, 0.0))      # tight J5 limits -> JOINT_LIMIT
    evaluate(c, FakeIK(lower=[-6] * 4 + [-0.1], upper=[6] * 4 + [0.1]), [0.0] * 5)
    assert [x.rejection_reason for x in c].count("JOINT_LIMIT") == 3


def test_rejected_not_sent_to_ik():
    be = FakeIK()
    evaluate(gen((0.1, 0, CUBE_Z, 0)), be, [0.0] * 5)
    assert be.calls == 0


if __name__ == "__main__":
    n = 0
    for k, v in list(globals().items()):
        if k.startswith("test_") and callable(v):
            v()
            n += 1
            print("PASS", k)
    print(f"{n} passed")

