"""Candidate generation per IMPLEMENTATION_PLAN 5.6. Pure function, no simulator imports.

Tool frame convention: tool z points DOWN (-Z of link_base); tool y is the gripper closing axis.
R(psi) = Rz(psi) @ Rx(pi) = [[c, s, 0], [s, -c, 0], [0, 0, -1]].
Poses are TCP poses (the IK backend applies the flange/TCP offset).
"""
import math
from typing import Dict, List, Sequence, Tuple

import numpy as np

from .types import GraspCandidate, Pose, STATUS_GENERATED, STATUS_REJECTED


def load_yaml(path) -> dict:
    import yaml
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def wrap_pi(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    a = math.fmod(a + math.pi, 2 * math.pi)
    if a <= 0:
        a += 2 * math.pi
    return a - math.pi


def tool_down_rotation(psi: float) -> np.ndarray:
    c, s = math.cos(psi), math.sin(psi)
    return np.array([[c, s, 0.0], [s, -c, 0.0], [0.0, 0.0, -1.0]])


def rot_to_quat_xyzw(R: np.ndarray) -> Tuple[float, float, float, float]:
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s)
    n = math.sqrt(sum(v * v for v in q))
    return tuple(v / n for v in q)


def quat_xyzw_to_rot(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pose_to_matrix(p: Pose) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = quat_xyzw_to_rot(p.orientation)
    T[:3, 3] = p.position
    return T


def _in_zone(x, y, z, zone: Dict) -> bool:
    for key, v in (("x_m", x), ("y_m", y), ("z_m", z)):
        if key in zone and not (zone[key][0] <= v <= zone[key][1]):
            return False
    return True


def generate(cube_pose: Sequence[float], gripper_cfg: dict, planning_cfg: dict,
             workspace_cfg: dict, task_id: str, cube_edge_m: float = 0.050,
             cube_cov_trace: float = 0.0) -> List[GraspCandidate]:
    """cube_pose = (x, y, z, yaw) of the cube centre in link_base (metres, radians).

    Returns one GraspCandidate per (k, offset). Candidates that fail the geometric
    pre-filter are returned with status REJECTED and a rejection_reason.
    """
    x, y, z, yaw_c = (float(v) for v in cube_pose)
    offsets = planning_cfg.get("yaw_offsets_deg", [0.0]) or [0.0]
    h_off = float(planning_cfg.get("grasp_height_offset_m", 0.0))
    appr = float(planning_cfg.get("approach_distance_m", 0.10))
    lift = float(planning_cfg.get("lift_distance_m", 0.08))
    clr = float(planning_cfg.get("table_clearance_m", 0.02))
    open_w = cube_edge_m + float(gripper_cfg["open_margin_m"])
    close_w = float(gripper_cfg["grasp_close_width_m"])
    table_z = float(workspace_cfg["table_top_z_m"])
    box = workspace_cfg["cube_workspace_box"]

    # cube-level pre-filter reason (same for all candidates)
    cube_reason = ""
    if not (box["x_m"][0] <= x <= box["x_m"][1] and box["y_m"][0] <= y <= box["y_m"][1]):
        cube_reason = "OUT_OF_WORKSPACE"
    elif open_w > float(gripper_cfg["max_open_m"]) or close_w < float(gripper_cfg.get("min_close_m", 0.0)):
        cube_reason = "GRIPPER_WIDTH"
    elif z + h_off + appr < table_z + clr:
        cube_reason = "COLLISION"          # pregrasp would be below table + clearance
    else:
        for zone in workspace_cfg.get("forbidden_zones") or []:
            if _in_zone(x, y, z + h_off, zone) or _in_zone(x, y, z + h_off + appr, zone):
                cube_reason = "COLLISION"
                break

    cands: List[GraspCandidate] = []
    n = 0
    for k in range(4):
        for d in offsets:
            psi = wrap_pi(yaw_c + k * math.pi / 2 + math.radians(d))
            quat = rot_to_quat_xyzw(tool_down_rotation(psi))
            gz = z + h_off
            grasp = Pose((x, y, gz), quat)
            pre = Pose((x, y, gz + appr), quat)
            lf = Pose((x, y, gz + lift), quat)
            # closing axis (tool y) angle = psi - 90deg; relative to cube frame:
            rel = psi - math.pi / 2 - yaw_c
            m = round(rel / (math.pi / 2))
            err = abs(wrap_pi(rel - m * math.pi / 2))
            faces = "+x/-x" if m % 2 == 0 else "+y/-y"
            c = GraspCandidate(
                task_id=task_id, candidate_id=f"{task_id}-c{n:02d}",
                grasp_pose=grasp, pregrasp_pose=pre, grasp_yaw_rad=psi,
                approach_distance_m=appr, lift_distance_m=lift,
                gripper_open_width_m=open_w, gripper_close_width_m=close_w,
                contact_faces=faces, lift_pose=lf, yaw_index_k=k,
                yaw_offset_deg=float(d), grasp_axis_error_rad=err,
                pose_cov_trace=float(cube_cov_trace))
            if cube_reason:
                c.status = STATUS_REJECTED
                c.rejection_reason = cube_reason
            else:
                c.status = STATUS_GENERATED
            cands.append(c)
            n += 1
    return cands
