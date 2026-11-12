"""Sim world tests (headless PyBullet DIRECT). SIMULATION ONLY."""
import math
import os
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from pupil_apriltags import Detector  # noqa: E402
from sim.c270_camera import C270SimCamera, load_cfg  # noqa: E402
from sim.pyb_world import SimWorld, rot_err_rad  # noqa: E402

D = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)


@pytest.fixture(scope="module")
def world():
    w = SimWorld(gui=False)
    yield w
    w.close()


@pytest.fixture(scope="module")
def cam(world):
    return C270SimCamera(load_cfg(world.cfg_paths["camera"]), np.random.default_rng(0))


@pytest.fixture(scope="module")
def cube_frame(world, cam):
    world.spawn_cube(0.38, 0.02, math.radians(25))
    return cam.render(world.p)


def test_urdf_five_arm_joints(world):
    assert len(world.arm_j) == 5 and len(world.grip_j) == 6


def test_cube_rests_on_table(world, cube_frame):
    z = world.cube_pose_true()[2, 3]
    assert abs(z - (world.table_top_z + world.edge / 2)) < 1e-3


def test_detect_ids_only_model_ids(world, cam, cube_frame):
    ids = {d.tag_id for d in Detector(families="tag36h11").detect(cv2.cvtColor(cube_frame, cv2.COLOR_BGR2GRAY))}
    model = {f["id"] for f in world.cube_cfg["faces"]}
    assert len(ids) >= 2 and ids <= model


def _tag_R_world(world, cam, img, tag_id):
    """World rotation of detected tag (OpenCV ArUco convention) via cv2.aruco + IPPE, TRUE camera pose."""
    det = cv2.aruco.ArucoDetector(D, cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(img)
    s = world.cube_cfg["tag_size_m"] / 2
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)
    out = {}
    for c, i in zip(corners, ids.ravel()):
        ok, rv, tv = cv2.solvePnP(obj, c.reshape(4, 2), cam.K_true, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        out[int(i)] = cam.T_world_cam_true[:3, :3] @ cv2.Rodrigues(rv)[0]
    return out


def test_tag_frames_match_cube_model(world, cam, cube_frame):
    Rs = _tag_R_world(world, cam, cube_frame, 0)
    Tc = world.cube_pose_true()
    faces = {f["id"]: f for f in world.cube_cfg["faces"]}
    assert 0 in Rs
    for i, R in Rs.items():
        Rexp = Tc[:3, :3] @ np.array(faces[i]["R_cube_tag"])
        assert math.degrees(rot_err_rad(R, Rexp)) < 5.0, i
    # tag 0 on top: z axis up
    assert math.degrees(math.acos(np.clip(Rs[0][2, 2], -1, 1))) < 10.0
    # side tags: y axis (tag up) = world +z
    for i, R in Rs.items():
        if i != 0:
            assert math.degrees(math.acos(np.clip(R[2, 1], -1, 1))) < 10.0, i


def test_ik_top_down(world):
    yaw = math.radians(30)
    R = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]) \
        @ np.diag([1, -1, -1.0])  # tool z down
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [0.35, 0.0, 0.10 + world.table_top_z]
    ok, q, pe, re = world.ik_tcp(T, seed=[0, -0.5, -0.5, 0, 0])
    assert ok and pe < 1e-3, (pe, math.degrees(re))


def test_checkerboard_found(world, cam):
    world.remove(world.cube)
    T = np.eye(4)
    T[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]  # long side along world y (camera FOV is narrow in depth)
    T[:3, 3] = [0.43, 0.0, world.table_top_z + 0.002]
    b, pts = world.spawn_checkerboard(T)
    img = cam.render(world.p)
    ok, c = cv2.findChessboardCorners(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (9, 6))
    world.remove(b)
    assert ok and len(c) == 54 and pts.shape == (54, 3)


def test_calib_tag_visible(world, cam):
    world.set_arm([0.0, -0.3, -0.5, 0.0, 0.0])
    world.spawn_calib_tag()
    img = cam.render(world.p)
    ids = {d.tag_id for d in Detector(families="tag36h11").detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))}
    world.remove(world.calib_tag_body)
    assert ids == {10}
