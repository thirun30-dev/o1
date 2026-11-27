# Copyright 2026 dk
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from optimus_depth.depth_obstacle_node import (
    backproject,
    classify_points,
    depth_to_metres,
    make_cloud,
    quaternion_matrix,
    voxel_thin,
)

ROOT = Path(__file__).resolve().parents[1]


def _optical_to_base(pitch, height):
    """Rotation/translation of a camera optical frame pitched down `pitch`."""
    # base <- camera_link (pitch about y), camera_link <- optical (z fwd, x right, y down)
    s, c = math.sin(pitch), math.cos(pitch)
    r_pitch = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    r_opt = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    return r_pitch @ r_opt, np.array([0.0, 0.0, height])


def test_depth_decoding_16bit_and_float():
    mm = np.array([[1000, 2500], [0, 65535]], dtype='<u2')
    d = depth_to_metres('16UC1', mm.tobytes(), 2, 2, 4)
    assert d[0, 1] == pytest.approx(2.5)
    f = np.array([[1.25, np.nan]], dtype='<f4')
    assert depth_to_metres('32FC1', f.tobytes(), 1, 2, 8)[0, 0] == pytest.approx(1.25)
    with pytest.raises(ValueError):
        depth_to_metres('rgb8', b'\0' * 6, 1, 2, 6)


def test_quaternion_matrix_yaw_90():
    r = quaternion_matrix(0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert r @ np.array([1, 0, 0]) == pytest.approx(np.array([0, 1, 0]))


def test_floor_box_and_drop_off_classified():
    # Camera 0.3 m above the floor, level. Synthesize depth for: floor rows,
    # a box face 1 m ahead, and a hole (floor 0.3 m lower) further down.
    w, h, f = 64, 48, 40.0
    cx, cy = w / 2, h / 2
    rot, trans = _optical_to_base(0.0, 0.3)
    depth = np.full((h, w), np.nan, np.float32)
    v = np.arange(h)[:, None].repeat(w, 1)
    below = v > cy
    depth[below] = 0.3 / ((v[below] - cy) / f)     # floor: y_opt * d = height
    depth[20:30, 28:36] = 1.0                       # box face at 1 m, around eye level
    depth[40:, 0:8] = 0.6 / ((v[40:, 0:8] - cy) / f)  # drop-off: floor 0.6 m below camera

    pts = backproject(depth, f, f, cx, cy, 1, 0.1, 5.0) @ rot.T + trans
    obstacles, clearing = classify_points(pts, 0.0, 0.04, 0.8, 0.08)

    assert len(clearing) == len(pts)
    box = obstacles[np.abs(obstacles[:, 0] - 1.0) < 1e-3]
    assert len(box) == 80                            # every box pixel is an obstacle
    drops = obstacles[obstacles[:, 2] < -0.08]
    assert len(drops) > 0                            # the hole is marked
    floor_like = np.abs(obstacles[:, 2]) < 0.04
    assert not floor_like.any()                      # plain floor never marks


def test_voxel_thin_and_cloud_layout():
    pts = np.array([[0.0, 0.0, 0.0], [0.01, 0.01, 0.0], [1.0, 0.0, 0.0]])
    thin = voxel_thin(pts, 0.05)
    assert len(thin) == 2
    from std_msgs.msg import Header
    cloud = make_cloud(Header(frame_id='base_link'), thin)
    assert cloud.width == 2 and cloud.point_step == 12 and len(cloud.data) == 24


def test_profiles_complete():
    cfg = yaml.safe_load((ROOT / 'config' / 'depth_camera.yaml').read_text())
    for name, prof in cfg['profiles'].items():
        for key in ('package', 'executable', 'depth_topic', 'color_topic', 'params', 'mount'):
            assert key in prof, (name, key)
        assert prof['params']['camera_name'] == 'depth_camera', name
        assert prof['depth_topic'].startswith('/depth_camera/'), name
