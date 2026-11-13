"""PyBullet IK backend implementing candidate_evaluator.ik_backend.IKBackend.  SIMULATION ONLY.

Frame check (done 2026-11-12, see sim/urdf/xarm5_gripper.urdf + probe of finger links in the
link_tcp frame): link_tcp has the same orientation as xarm_gripper_base_link (rpy 0), tool z points
out along the fingers and the fingers open/close symmetrically along link_tcp +/-y. This matches the
candidate_evaluator convention (tool z down, tool y = closing axis), so T_TOOL_TCP is the identity.
If a different gripper is used later, put the fixed rotation tool->link_tcp in T_TOOL_TCP (candidate
poses are right-multiplied by it before IK); do not change candidate_evaluator.

Poses are in link_base == world frame (the arm base sits at the world origin; see sim/README.md).
IK is PyBullet damped least squares + FK verification (SimWorld.ik_tcp); MoveIt/pick_ik replace it later.
"""
import numpy as np

T_TOOL_TCP = np.eye(4)  # candidate "tool" frame -> URDF link_tcp. Identity (verified, see module doc).


REFINE_PASSES = 3               # polish passes when the first solve is close but not within tolerance
REFINE_MAX_POS_ERR_M = 0.02     # only polish solves already within 2 cm / 5 deg (else genuinely unreachable)
REFINE_MAX_ROT_ERR_RAD = 0.087


class PyBulletIK:
    """Implements IKBackend.solve / joint_limits on top of SimWorld.ik_tcp."""

    def __init__(self, world, T_tool_tcp=None):
        self.world = world
        self.T_tool_tcp = T_TOOL_TCP if T_tool_tcp is None else np.asarray(T_tool_tcp, float)
        self.calls = 0

    def solve_info(self, pose_4x4, seed):
        """Like solve() but returns details: dict(ok, q, pe_m, re_rad, tries, reason)."""
        T = np.asarray(pose_4x4, float) @ self.T_tool_tcp
        ok, q, pe, re = self.world.ik_tcp(T, seed=list(seed))
        tries = self.world.last_tries
        # PyBullet DLS often stops 1-3 mm short; re-seeding from its own answer converges (0.02 mm).
        for _ in range(REFINE_PASSES):
            if ok or pe > REFINE_MAX_POS_ERR_M or re > REFINE_MAX_ROT_ERR_RAD:
                break
            ok, q, pe, re = self.world.ik_tcp(T, seed=list(q), tries=1)
            tries += 1
        return {"ok": ok, "q": [float(v) for v in q] if ok else None, "pe_m": pe, "re_rad": re, "tries": tries,
                "reason": "" if ok else "NO_IK"}

    def solve(self, pose_4x4, seed):
        self.calls += 1
        r = self.solve_info(pose_4x4, seed)
        if not r["ok"]:
            return False, None, "NO_IK"
        return True, r["q"], ""

    def joint_limits(self):
        return list(self.world.arm_lo), list(self.world.arm_hi)
