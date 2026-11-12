"""Load config/cube/cube_model.yaml into a dataclass (frames: see the yaml header)."""
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict

import numpy as np
import yaml


@dataclass
class CubeModel:
    edge_m: float
    tag_size_m: float
    T_cube_tag: Dict[int, np.ndarray] = field(default_factory=dict)   # id -> 4x4
    corners_cube: Dict[int, np.ndarray] = field(default_factory=dict)  # id -> 4x3, TL,TR,BR,BL
    face_names: Dict[int, str] = field(default_factory=dict)
    yaw_symmetry_deg: float = 90.0
    calib_tag_id: int = -1
    calib_tag_size_m: float = 0.0

    @property
    def ids(self):
        return sorted(self.T_cube_tag)


def tag_corners_in_tag_frame(s: float) -> np.ndarray:
    """OpenCV ArUco order TL,TR,BR,BL, z=0 plane."""
    h = s / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)


def load_cube_model(path=None) -> CubeModel:
    if path is None:
        path = Path(__file__).resolve().parents[3] / "config" / "cube" / "cube_model.yaml"
    c = yaml.safe_load(Path(path).read_text())
    s = float(c["tag_size_m"])
    m = CubeModel(edge_m=float(c["edge_m"]), tag_size_m=s,
                  yaw_symmetry_deg=float(c.get("yaw_symmetry_deg", 90.0)))
    pts = tag_corners_in_tag_frame(s)
    for f in c["faces"]:
        T = np.eye(4)
        T[:3, :3] = np.array(f["R_cube_tag"], dtype=np.float64)
        T[:3, 3] = np.array(f["t_cube_tag"], dtype=np.float64)
        i = int(f["id"])
        m.T_cube_tag[i] = T
        m.corners_cube[i] = pts @ T[:3, :3].T + T[:3, 3]
        m.face_names[i] = f["face"]
    ct = c.get("calib_tag", {})
    m.calib_tag_id = int(ct.get("id", -1))
    m.calib_tag_size_m = float(ct.get("tag_size_m", 0.0))
    return m
