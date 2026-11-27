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

import importlib.util
import math
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def det():
    path = ROOT / 'scripts' / 'camera_obstacle_node.py'
    spec = importlib.util.spec_from_file_location('camera_obstacle_node', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_ground_lookup_centre_pixel(det):
    # Optical axis pitched 30 deg down from 0.5 m hits the floor at h / tan(30).
    w, h, f = 101, 81, 80.0
    rng, ang = det.ground_lookup(w, h, f, f, 50, 40, 0.5, math.radians(30))
    assert rng[40, 50] == pytest.approx(0.5 / math.tan(math.radians(30)), rel=1e-6)
    assert ang[40, 50] == pytest.approx(0.0, abs=1e-9)
    # Left of centre in the image = positive (left) bearing; lower = closer.
    assert ang[40, 10] > 0 > ang[40, 90]
    assert rng[70, 50] < rng[40, 50]


def test_ground_lookup_above_horizon_is_inf(det):
    rng, _ = det.ground_lookup(64, 48, 50.0, 50.0, 32, 24, 0.5, 0.0)
    assert np.isinf(rng[:25]).all()       # level camera: top half + centre row
    assert np.isfinite(rng[30:]).all()


def test_lowest_runs_ignores_short_noise(det):
    mask = np.zeros((20, 3), bool)
    mask[15, 0] = True                     # 1 px noise only
    mask[5:12, 1] = True                   # real obstacle, bottom at row 11
    mask[18, 1] = True                     # noise below it
    row, has = det.lowest_runs(mask, 3)
    assert list(has) == [False, True, False]
    assert row[1] == 11


def test_bin_scan_keeps_nearest(det):
    ranges = det.bin_scan(np.array([1.5, 0.8, 1.9]), np.array([0.01, 0.02, -0.35]),
                          -0.5, 0.5, 10)
    assert ranges[5] == pytest.approx(0.8)
    assert ranges[1] == pytest.approx(1.9)
    assert np.isinf(ranges[0])


def test_floor_model_flags_coloured_box_on_grey_floor(det):
    rng = np.random.default_rng(0)
    img = np.clip(rng.normal(120, 6, (60, 80, 3)), 0, 255).astype(np.uint8)
    img[20:35, 30:50] = (30, 30, 200)      # red box (BGR)
    hsv = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2HSV)
    model = det.FloorModel(0.06, 40)
    model.learn(hsv[50:, 20:60].reshape(-1, 3))
    mask = model.classify(hsv)
    assert mask[24:31, 34:46].mean() > 0.95
    assert mask[45:, :].mean() < 0.02


def test_camera_profiles_complete():
    cfg = yaml.safe_load((ROOT / 'config' / 'camera.yaml').read_text())
    assert {'laptop', 'picam'} <= set(cfg['profiles'])
    for name, prof in cfg['profiles'].items():
        assert prof['driver'] in ('opencv', 'libcamera'), name
        for key in ('x', 'y', 'z_above_floor', 'pitch_deg'):
            assert key in prof['mount'], (name, key)
        assert prof['mount']['pitch_deg'] > 0, name


def test_nav2_uses_camera_scan():
    text = (ROOT / 'config' / 'nav2.yaml').read_text()
    assert text.count('topic: /camera/scan') == 2
    assert text.count('"obstacle_layer", "camera_layer", "depth_layer", "inflation_layer"') == 2
    assert text.count('topic: /depth_camera/obstacles') == 2
