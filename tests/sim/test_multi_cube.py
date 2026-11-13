"""Multi-cube sim tests: 3 identified cubes, tinted textures, stacking physics, headless command run. SIMULATION ONLY."""
import math
import os
import subprocess
import sys

import cv2
import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "src", "cube_perception"))
from pupil_apriltags import Detector  # noqa: E402
from cube_perception import load_cube_set  # noqa: E402
from sim.c270_camera import C270SimCamera, load_cfg  # noqa: E402
from sim.pyb_world import SimWorld  # noqa: E402


@pytest.fixture(scope="module")
def world():
    w = SimWorld(gui=False)
    yield w
    w.close()


def test_three_cubes_coexist_and_rest_on_table(world):
    for n, (x, y) in {1: (0.36, -0.05), 2: (0.36, 0.06), 3: (0.46, 0.0)}.items():
        world.spawn_cube(x, y, 0.3 * n, number=n)
    assert sorted(world.cubes) == [1, 2, 3]
    for n in (1, 2, 3):
        assert abs(world.cube_pose_true(n)[2, 3] - world.edge / 2) < 1.5e-3
    world.activate_cube(2)
    assert world.cube == world.cubes[2]


def test_each_cube_shows_only_its_own_ids(world):
    cams = C270SimCamera(load_cfg(world.cfg_paths["camera"]), np.random.default_rng(0))
    det = Detector(families="tag36h11")
    cset = load_cube_set()
    for n in list(world.cubes):
        world.remove(world.cubes[n])
    for n, m in cset.items():
        world.spawn_cube(0.38, 0.03, math.radians(25), number=n)
        img = cams.render(world.p)
        ids = {d.tag_id for d in det.detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))}
        assert ids and ids <= set(m.ids), (n, ids)
        world.remove(world.cubes[n])


def test_tint_is_pastel_quiet_zone_full_colour_bottom(world):
    fobj, fimg = world._build_cube_assets(2)
    atlas = cv2.imread(fimg)
    rgb = world.cube_by_num[2]["color_rgb"]
    bottom = atlas[-10, -10]                                   # slot 5 (bottom face) is full colour
    assert np.all(np.abs(bottom[::-1].astype(int) - rgb) <= 1)
    corner = atlas[20, 20].astype(int)                         # quiet zone near the edge band: coloured, lighter than the bottom
    assert corner.sum() > 0
    mid_margin = atlas[40, 256].astype(int)                    # inside the quiet zone, outside the edge band: pastel (not white)
    assert mid_margin[1] > bottom[1] and not np.all(mid_margin == 255)


def test_legacy_single_cube_unchanged(world):
    for n in list(world.cubes):
        world.remove(world.cubes[n])
    world.spawn_cube(0.4, 0.0, 0.0)                            # legacy: no number -> plain white cube, ids 0..4
    assert world.cube is not None and not world.cubes
    world.remove(world.cube)


def test_stack_rests_on_lower_cube(world):
    for n in list(world.cubes):
        world.remove(world.cubes[n])
    e = world.edge
    world.spawn_cube(0.40, 0.0, 0.2, number=1)
    world.spawn_cube(0.40, 0.0, 0.2, number=2, z=world.table_top_z + 1.5 * e + 0.002)
    world.spawn_cube(0.40, 0.0, 0.2, number=3, z=world.table_top_z + 2.5 * e + 0.004)
    for _ in range(480):
        world.p.stepSimulation()
    zs = [world.cube_pose_true(n)[2, 3] for n in (1, 2, 3)]
    assert np.allclose(zs, [0.5 * e, 1.5 * e, 2.5 * e], atol=3e-3), zs
    assert all(world.cube_pose_true(n)[2, 2] > 0.99 for n in (1, 2, 3))
    for n in list(world.cubes):
        world.remove(world.cubes[n])


def test_headless_command_run(tmp_path):
    out, kin = tmp_path / "multi", tmp_path / "kin"
    r = subprocess.run(
        [sys.executable, "-I", os.path.join(ROOT, "scripts", "sim_demo.py"), "--cubes", "red,green",
         "--layout", "0.36,0.0,20;0.36,0.12,-10", "--command", "put green in zone 2 and stack red on green",
         "--seed", "0", "--calib-poses", "6", "--board-poses", "6", "--out", str(out), "--kin-dir", str(kin)],
        capture_output=True, text=True, timeout=1200)
    assert r.returncode == 0, r.stdout[-2500:] + r.stderr[-1500:]
    assert "TASKS 2/2 SUCCESS" in r.stdout
    names = sorted(p.name for p in kin.glob("*.txt"))
    assert len(names) == 2 and len(list(kin.glob("*.csv"))) == 2
    assert names[0].endswith("_task01_green_to_zone2_SUCCESS.txt") and names[1].endswith("_task02_red_on_green_SUCCESS.txt")
    body = (kin / names[1]).read_text(encoding="utf-8")
    assert "command          : put green in zone 2 and stack red on green" in body
    assert "target / stacking pose source" in body and "recorded" in body        # zone 2 is outside the camera view
    s = (out / "summary.md").read_text(encoding="utf-8")
    assert "SIMULATION ONLY" in s and "PLACEHOLDER" in s


def test_unknown_cube_in_command_exits_2(tmp_path):
    r = subprocess.run(
        [sys.executable, "-I", os.path.join(ROOT, "scripts", "sim_demo.py"), "--cubes", "red", "--layout", "0.38,0.0,0",
         "--command", "place the green cube in zone 1", "--calib-poses", "6", "--board-poses", "6",
         "--out", str(tmp_path / "o"), "--kin-dir", str(tmp_path / "k")],
        capture_output=True, text=True, timeout=1200)
    assert r.returncode == 2 and "green cube (cube 2) is not in the scene" in r.stdout
