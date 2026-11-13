"""Load config/cube/cube_model.yaml into a dataclass (frames: see the yaml header)."""
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Optional, Sequence, Tuple

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
    number: int = 1                    # cube identity (1 = red, 2 = green, 3 = blue); 1 for the legacy single model
    name: str = "cube"
    rgb: Tuple[int, int, int] = (255, 255, 255)

    @property
    def ids(self):
        return sorted(self.T_cube_tag)


def tag_corners_in_tag_frame(s: float) -> np.ndarray:
    """OpenCV ArUco order TL,TR,BR,BL, z=0 plane."""
    h = s / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)


CONFIG_CUBE_DIR = Path(__file__).resolve().parents[3] / "config" / "cube"


def load_cube_model(path=None, face_ids: Optional[Sequence[int]] = None, number: int = 1, name: str = "cube",
                    rgb=(255, 255, 255)) -> CubeModel:
    """face_ids (optional): replace the tag IDs of the yaml faces, in yaml face order (+z, +x, -x, +y, -y).
    The geometry (edge, tag size, face frames) always comes from cube_model.yaml - never duplicated per cube."""
    if path is None:
        path = CONFIG_CUBE_DIR / "cube_model.yaml"
    c = yaml.safe_load(Path(path).read_text())
    if face_ids is not None:
        if len(face_ids) != len(c["faces"]):
            raise ValueError("face_ids needs %d ids" % len(c["faces"]))
        c["faces"] = [dict(f, id=int(i)) for f, i in zip(c["faces"], face_ids)]
    s = float(c["tag_size_m"])
    m = CubeModel(edge_m=float(c["edge_m"]), tag_size_m=s,
                  yaw_symmetry_deg=float(c.get("yaw_symmetry_deg", 90.0)),
                  number=int(number), name=str(name), rgb=tuple(int(v) for v in rgb))
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


def load_cubes_cfg(path=None) -> dict:
    """config/cube/cubes.yaml as a dict (cubes list + tint settings)."""
    if path is None:
        path = CONFIG_CUBE_DIR / "cubes.yaml"
    return yaml.safe_load(Path(path).read_text())


def load_cube_set(model_path=None, cubes_path=None) -> Dict[int, CubeModel]:
    """{cube number: CubeModel} for the identified cubes. Tag IDs must be unique across cubes and must not
    contain the calibration tag ID (ValueError otherwise)."""
    cfg = load_cubes_cfg(cubes_path)
    out: Dict[int, CubeModel] = {}
    seen = {}
    for e in cfg["cubes"]:
        m = load_cube_model(model_path, face_ids=e["face_ids"], number=e["number"], name=e["name"],
                            rgb=e["color_rgb"])
        for i in m.ids:
            if i in seen:
                raise ValueError("tag id %d used by cube %d and cube %d" % (i, seen[i], m.number))
            if i == m.calib_tag_id:
                raise ValueError("tag id %d of cube %d clashes with the calibration tag" % (i, m.number))
            seen[i] = m.number
        out[m.number] = m
    return out


def identify_cubes(detections, cube_set: Dict[int, CubeModel]):
    """Group detections [(tag_id, corners)] by the cube whose ID set contains the tag.
    -> {cube number: [(tag_id, corners)]}. Tags that belong to no cube (e.g. the calibration tag) are dropped."""
    owner = {i: n for n, m in cube_set.items() for i in m.ids}
    out: Dict[int, list] = {}
    for tid, corners in detections:
        n = owner.get(int(tid))
        if n is not None:
            out.setdefault(n, []).append((int(tid), corners))
    return out
