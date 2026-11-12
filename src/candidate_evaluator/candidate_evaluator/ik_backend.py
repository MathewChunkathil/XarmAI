"""IK backend interface + evaluation of candidates (IMPLEMENTATION_PLAN WP5.2).
No simulator imports here; PyBullet / MoveIt backends live elsewhere and implement IKBackend."""
from typing import List, Optional, Protocol, Sequence, Tuple, runtime_checkable

import numpy as np

from .generate import pose_to_matrix
from .types import GraspCandidate, STATUS_REJECTED


@runtime_checkable
class IKBackend(Protocol):
    def solve(self, pose_4x4: np.ndarray, seed: Sequence[float]) -> Tuple[bool, Optional[list], str]:
        """TCP pose (4x4, link_base) -> (ok, q, reason). reason in {"", "NO_IK", "JOINT_LIMIT"}."""
        ...

    def joint_limits(self) -> Tuple[Sequence[float], Sequence[float]]:
        """(lower, upper) per arm joint, radians."""
        ...


class FakeIK:
    """Test backend. reach_fn(pose_4x4) -> bool decides reachability (default: always).
    Default q = seed with q[0]=azimuth of the target and q[4]=tool yaw, so margins are testable."""

    def __init__(self, lower=None, upper=None, reach_fn=None, q_fn=None):
        self.lower = list(lower if lower is not None else [-6.28] * 5)
        self.upper = list(upper if upper is not None else [6.28] * 5)
        self.reach_fn = reach_fn or (lambda T: True)
        self.q_fn = q_fn
        self.calls = 0

    def solve(self, pose_4x4, seed):
        self.calls += 1
        if not self.reach_fn(pose_4x4):
            return False, None, "NO_IK"
        if self.q_fn is not None:
            return True, list(self.q_fn(pose_4x4, seed)), ""
        q = list(seed)
        q[0] = float(np.arctan2(pose_4x4[1, 3], pose_4x4[0, 3]))
        q[4] = float(np.arctan2(pose_4x4[1, 0], pose_4x4[0, 0]))
        return True, q, ""

    def joint_limits(self):
        return self.lower, self.upper


def joint_margin(q: Sequence[float], lower: Sequence[float], upper: Sequence[float]) -> float:
    """Min over joints of normalised margin min(q-lo, hi-q)/(hi-lo), in [0, 0.5] (negative if outside)."""
    q, lo, hi = (np.asarray(v, float) for v in (q, lower, upper))
    return float(np.min(np.minimum(q - lo, hi - q) / (hi - lo)))


def evaluate(candidates: List[GraspCandidate], backend: IKBackend, seed: Sequence[float]) -> List[GraspCandidate]:
    """For every non-rejected candidate solve pregrasp -> grasp -> lift (each seeded by the previous).
    Failure sets status REJECTED with NO_IK or JOINT_LIMIT. On success stores q_start, q_pregrasp,
    q_grasp, q_lift and joint_margin_min. Mutates and returns the list."""
    lo, hi = backend.joint_limits()
    for c in candidates:
        if c.status == STATUS_REJECTED:
            continue
        c.q_start = list(seed)
        cur = list(seed)
        qs, margins = [], []
        fail = ""
        for p in (c.pregrasp_pose, c.grasp_pose, c.lift_pose):
            ok, q, reason = backend.solve(pose_to_matrix(p), cur)
            if not ok or q is None:
                fail = reason or "NO_IK"
                break
            m = joint_margin(q, lo, hi)
            if m < 0.0:
                fail = "JOINT_LIMIT"
                break
            qs.append(list(q))
            margins.append(m)
            cur = list(q)
        if fail:
            c.status = STATUS_REJECTED
            c.rejection_reason = fail
            continue
        c.q_pregrasp, c.q_grasp, c.q_lift = qs
        c.joint_margin_min = float(min(margins))
    return candidates
