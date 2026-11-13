"""Three identified cubes: config, ID sets, grouping, per-cube PnP, stack-level snapping, zones, layout.
Run: python -m pytest tests/unit/test_cube_set.py"""
import math
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src" / "cube_perception"))
from cube_perception import (estimate_cubes, identify_cubes, load_cube_model, load_cube_set, to_base)  # noqa: E402
from sim import zones as zn  # noqa: E402

K = np.array([[900.0, 0, 640], [0, 900.0, 360], [0, 0, 1]])
WS = yaml.safe_load((ROOT / "config/workspace/workspace.yaml").read_text())
CUBES = load_cube_set()


def make_T(R, t):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def rot(axis, deg):
    return cv2.Rodrigues(np.array(axis, float) * np.radians(deg))[0]


def project(model, T, ids):
    out = []
    for i in ids:
        P = model.corners_cube[i] @ T[:3, :3].T + T[:3, 3]
        uv = P @ K.T
        out.append((i, uv[:, :2] / uv[:, 2:3]))
    return out


def test_three_cubes_red_green_blue():
    assert sorted(CUBES) == [1, 2, 3]
    assert [CUBES[n].name for n in (1, 2, 3)] == ["red", "green", "blue"]
    assert CUBES[1].ids == [0, 1, 2, 3, 4]           # cube 1 keeps the original ids


def test_ids_unique_and_avoid_calib_tag():
    allids = [i for m in CUBES.values() for i in m.ids]
    assert len(allids) == len(set(allids)) == 15
    assert CUBES[1].calib_tag_id not in allids


def test_geometry_shared_across_cubes():
    base = load_cube_model()
    for m in CUBES.values():
        assert m.edge_m == base.edge_m and m.tag_size_m == base.tag_size_m
        # same face frames, only the id differs (yaml face order)
        for k, i in enumerate(m.ids):
            assert np.allclose(m.corners_cube[m.ids[k]], base.corners_cube[base.ids[k]])


def test_identify_groups_by_id_set():
    dets = [(0, 1), (6, 2), (12, 3), (10, 4), (3, 5), (99, 6)]       # 10 = calibration tag, 99 = unknown
    g = identify_cubes(dets, CUBES)
    assert {n: [d[0] for d in v] for n, v in g.items()} == {1: [0, 3], 2: [6], 3: [12]}


def test_estimate_cubes_per_cube_pose():
    Ts = {1: make_T(rot([1, 0, 0], 200) @ rot([0, 0, 1], 30), [-0.08, 0.0, 0.5]),
          2: make_T(rot([1, 0, 0], 205) @ rot([0, 0, 1], 70), [0.0, 0.0, 0.5]),
          3: make_T(rot([1, 0, 0], 195) @ rot([0, 0, 1], -20), [0.08, 0.01, 0.5])}
    dets = []
    for n, T in Ts.items():
        m = CUBES[n]
        vis = [i for i, Tf in m.T_cube_tag.items()
               if (T[:3, :3] @ Tf[:3, 2]) @ (T[:3, :3] @ Tf[:3, 3] + T[:3, 3]) < -0.2 * np.linalg.norm(T[:3, 3])]
        assert len(vis) >= 1
        dets += project(m, T, vis)
    res = estimate_cubes(dets, K, None, CUBES)
    assert sorted(res) == [1, 2, 3]
    for n, r in res.items():
        assert r.valid, (n, r.invalid_reason)
        assert np.linalg.norm(r.T_cam_cube[:3, 3] - Ts[n][:3, 3]) < 1e-6
        assert set(r.tag_ids_used) <= set(CUBES[n].ids)


def test_missing_cube_not_reported():
    T = make_T(rot([1, 0, 0], 200), [0, 0, 0.5])
    m = CUBES[2]
    res = estimate_cubes(project(m, T, [m.ids[0]]), K, None, CUBES)
    assert list(res) == [2]


def test_to_base_stack_level_snapping():
    Tb = np.eye(4)
    edge = 0.05
    for level in (0, 1, 2):
        z = edge / 2 + level * edge
        Tcc = make_T(np.eye(3), [0.3, 0.1, z + 0.008])             # 8 mm perception z noise
        out = to_base(Tb, Tcc, 0.0, edge, max_level=2)
        assert abs(out[2, 3] - z) < 1e-12
    # legacy table-only behaviour unchanged
    out = to_base(Tb, make_T(np.eye(3), [0.3, 0.1, 0.075]), 0.0, edge)
    assert abs(out[2, 3] - 0.025) < 1e-12
    # never above max_level
    out = to_base(Tb, make_T(np.eye(3), [0.3, 0.1, 0.5]), 0.0, edge, max_level=2)
    assert abs(out[2, 3] - (0.025 + 2 * edge)) < 1e-12


def test_zones_config_valid():
    zs = zn.place_zones(WS)
    assert [z["number"] for z in zs] == [1, 2, 3]
    # zone 1 == legacy keys
    assert zs[0]["x_m"] == WS["place_zone"]["x_m"] and zs[0]["target"]["x_m"] == WS["place_target"]["x_m"]
    pick = WS["cube_workspace_box"]
    for i, z in enumerate(zs):
        assert not zn.boxes_overlap(z, pick)
        t = z["target"]
        assert z["x_m"][0] <= t["x_m"] <= z["x_m"][1] and z["y_m"][0] <= t["y_m"] <= z["y_m"][1]
        for z2 in zs[i + 1:]:
            assert not zn.boxes_overlap(z, z2)


def test_zones_legacy_fallback():
    ws = {k: v for k, v in WS.items() if k != "place_zones"}
    zs = zn.place_zones(ws)
    assert len(zs) == 1 and zs[0]["number"] == 1 and zs[0]["target"] == WS["place_target"]


def test_zone_slots():
    zs = zn.place_zones(WS)
    edge, sp = 0.05, WS["place_slot_spacing_m"]
    assert len(zn.zone_slots(zs[0], sp, edge)) == 1               # zone 1 holds one cube
    for z in zs[1:]:
        sl = zn.zone_slots(z, sp, edge)
        assert len(sl) >= 3 and sl[0] == (z["target"]["x_m"], z["target"]["y_m"])
        for i, a in enumerate(sl):
            for b in sl[i + 1:]:
                assert math.hypot(a[0] - b[0], a[1] - b[1]) >= sp - 1e-9
            assert z["x_m"][0] + edge / 2 <= a[0] <= z["x_m"][1] - edge / 2


def test_random_layout_spacing():
    rng = np.random.default_rng(0)
    box = WS["cube_workspace_box"]
    for _ in range(50):
        pts = zn.min_cube_spacing(box, 3, rng, 0.10, 0.03)
        assert pts is not None
        for i, a in enumerate(pts):
            assert box["x_m"][0] + 0.03 <= a[0] <= box["x_m"][1] - 0.03
            for b in pts[i + 1:]:
                assert math.hypot(a[0] - b[0], a[1] - b[1]) >= 0.10
    assert zn.min_cube_spacing(box, 30, rng, 0.10, 0.03) is None       # cannot fit -> None, no endless loop
