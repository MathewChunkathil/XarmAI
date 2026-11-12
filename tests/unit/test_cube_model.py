"""Check cube_model.yaml frames are sane. Run: python -m pytest tests/unit/test_cube_model.py"""
from pathlib import Path

import numpy as np
import yaml

CFG = Path(__file__).resolve().parents[2] / "config" / "cube" / "cube_model.yaml"
NORMALS = {"+x": [1, 0, 0], "-x": [-1, 0, 0], "+y": [0, 1, 0], "-y": [0, -1, 0], "+z": [0, 0, 1]}


def load():
    return yaml.safe_load(CFG.read_text())


def test_ids_unique_and_five_faces():
    c = load()
    ids = [f["id"] for f in c["faces"]]
    assert sorted(ids) == [0, 1, 2, 3, 4]
    assert c["calib_tag"]["id"] not in ids


def test_tag_fits_face():
    c = load()
    cell = c["tag_size_m"] / c["tag_cells_black"]
    assert cell * c["tag_cells_total"] <= c["edge_m"] + 1e-9


def test_rotations_and_offsets():
    c = load()
    h = c["edge_m"] / 2
    for f in c["faces"]:
        R = np.array(f["R_cube_tag"], float)
        n = np.array(NORMALS[f["face"]], float)
        assert np.allclose(R @ R.T, np.eye(3)), f
        assert np.isclose(np.linalg.det(R), 1.0), f          # right-handed
        assert np.allclose(R[:, 2], n), f                      # tag z = out of face
        assert np.allclose(np.array(f["t_cube_tag"]), n * h), f  # on face centre
        if f["face"] != "+z":
            assert np.allclose(R[:, 1], [0, 0, 1]), f          # side tag up = cube top


def test_corners_lie_on_face():
    c = load()
    s, h = c["tag_size_m"] / 2, c["edge_m"] / 2
    corners = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]])
    for f in c["faces"]:
        R, t = np.array(f["R_cube_tag"], float), np.array(f["t_cube_tag"], float)
        n = np.array(NORMALS[f["face"]], float)
        p = corners @ R.T + t
        assert np.allclose(p @ n, h), f                        # all on face plane
        assert np.all(np.abs(p) <= h + 1e-9), f                # inside cube face
