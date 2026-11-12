"""Dataclasses mirroring GraspCandidate.msg / ScoreBreakdown.msg (IMPLEMENTATION_PLAN 4.1).
Field names are identical; extra fields are appended at the end (allowed by section 4)."""
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

STATUS_GENERATED = 0
STATUS_REJECTED = 1
STATUS_PLANNED = 2
STATUS_SIM_PASSED = 3
STATUS_SIM_FAILED = 4
STATUS_SELECTED = 5
STATUS_NAMES = {0: "GENERATED", 1: "REJECTED", 2: "PLANNED", 3: "SIM_PASSED",
                4: "SIM_FAILED", 5: "SELECTED"}

REJECTION_REASONS = ("NO_IK", "JOINT_LIMIT", "OUT_OF_WORKSPACE", "COLLISION",
                     "GRIPPER_WIDTH", "PLAN_FAIL", "STALE", "SIM_COLLISION", "SIM_DROP")

SCORE_NAMES = ["motion", "clearance", "kinematic", "grasp", "time"]


@dataclass
class Pose:
    """geometry_msgs/PoseStamped equivalent. orientation = quaternion (x, y, z, w)."""
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]
    frame_id: str = "link_base"


@dataclass
class ScoreBreakdown:
    names: List[str] = field(default_factory=lambda: list(SCORE_NAMES))
    raw: List[Optional[float]] = field(default_factory=lambda: [None] * 5)
    normalised: List[float] = field(default_factory=lambda: [0.0] * 5)
    weights: List[float] = field(default_factory=lambda: [0.0] * 5)
    total: float = 0.0                       # sum(w*norm); LOWER is better
    notes: List[str] = field(default_factory=list)   # extra (not in msg)


@dataclass
class GraspCandidate:
    task_id: str
    candidate_id: str                        # "<task_id>-c<NN>"
    grasp_pose: Pose                         # TCP pose in link_base
    pregrasp_pose: Pose
    grasp_yaw_rad: float
    approach_distance_m: float
    lift_distance_m: float
    gripper_open_width_m: float
    gripper_close_width_m: float
    contact_faces: str                       # e.g. "+x/-x" (cube frame)
    status: int = STATUS_GENERATED
    rejection_reason: str = ""
    score: ScoreBreakdown = field(default_factory=ScoreBreakdown)
    # ---- extra fields (appended) ----
    lift_pose: Optional[Pose] = None
    yaw_index_k: int = 0
    yaw_offset_deg: float = 0.0
    grasp_axis_error_rad: float = 0.0        # |closing axis - nearest face normal|
    pose_cov_trace: float = 0.0
    q_start: Optional[list] = None
    q_pregrasp: Optional[list] = None
    q_grasp: Optional[list] = None
    q_lift: Optional[list] = None
    joint_margin_min: Optional[float] = None  # min normalised margin in [0, 0.5]
    min_clearance_m: Optional[float] = None   # from planner (None until planned)
    predicted_duration_s: Optional[float] = None
