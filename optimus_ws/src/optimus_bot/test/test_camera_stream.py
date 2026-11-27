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
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def stream():
    path = ROOT / 'scripts' / 'camera_stream_node.py'
    spec = importlib.util.spec_from_file_location('camera_stream_node', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _img(encoding, array):
    h, w = array.shape[:2]
    return SimpleNamespace(encoding=encoding, height=h, width=w,
                           step=array.nbytes // h, data=array.tobytes())


def test_parse_feeds(stream):
    feeds = stream.parse_feeds(['front=/camera/image_raw', 'bad', '=x', 'd = /depth '])
    assert feeds == {'front': '/camera/image_raw', 'd': '/depth'}


def test_depth_is_colourised_and_invalid_is_black(stream):
    depth = np.array([[0, 500, 3900]], dtype='<u2')
    img = stream.image_to_bgr(_img('16UC1', depth))
    assert img.shape == (1, 3, 3)
    assert (img[0, 0] == 0).all()                 # no reading -> black
    assert img[0, 1, 2] > img[0, 1, 0]            # near -> red
    assert img[0, 2, 0] > img[0, 2, 2]            # far -> blue


def test_rgb_is_converted_to_bgr(stream):
    rgb = np.zeros((2, 2, 3), np.uint8)
    rgb[..., 0] = 255
    assert stream.image_to_bgr(_img('rgb8', rgb))[0, 0].tolist() == [0, 0, 255]
