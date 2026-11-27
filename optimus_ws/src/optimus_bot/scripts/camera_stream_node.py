#!/usr/bin/env python3
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

"""
Camera feeds for the web dashboard, over rosbridge.

Raw images (~1 MB each) are far too heavy for a WebSocket to a phone, so each
feed is re-encoded as a small JPEG sensor_msgs/CompressedImage:

    <source image topic>  ->  /dashboard/camera/<name>/compressed

Feeds cost nothing while nobody watches: a source is only subscribed while
its dashboard topic has a subscriber (the dashboard's CAMERA tab is open).
Depth images (16UC1 / 32FC1) are colourised (near = red, far = blue).

/dashboard/camera/feeds (std_msgs/String, JSON) lists the feeds and whether
their source is currently publishing, so the dashboard only offers live ones.
"""

import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

DEFAULT_FEEDS = [
    'front=/camera/image_raw',
    'obstacles=/camera/obstacle_debug',
    'depth_color=/depth_camera/color/image_raw',
    'depth=/depth_camera/depth/image_rect_raw',
]


def image_to_bgr(msg, max_depth_m=4.0):
    """sensor_msgs/Image -> BGR array; depth images are colourised."""
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    if enc in ('16uc1', 'mono16', '32fc1'):
        if enc == '32fc1':
            depth = buf[:, :msg.width * 4].copy().view('<f4').reshape(msg.height, msg.width)
        else:
            raw = buf[:, :msg.width * 2].copy().view('<u2').reshape(msg.height, msg.width)
            depth = raw.astype(np.float32) / 1000.0
        valid = np.isfinite(depth) & (depth > 0)
        scaled = np.clip(np.where(valid, depth, 0) / max_depth_m, 0, 1)
        img = cv2.applyColorMap((255 - scaled * 255).astype(np.uint8), cv2.COLORMAP_JET)
        img[~valid] = 0
        return img
    channels = {'bgr8': 3, 'rgb8': 3, '8uc3': 3, 'bgra8': 4, 'rgba8': 4,
                'mono8': 1, '8uc1': 1}
    if enc in channels:
        ch = channels[enc]
        img = buf[:, :msg.width * ch].reshape(msg.height, msg.width, ch)
        conv = {'rgb8': cv2.COLOR_RGB2BGR, 'bgra8': cv2.COLOR_BGRA2BGR,
                'rgba8': cv2.COLOR_RGBA2BGR, 'mono8': cv2.COLOR_GRAY2BGR,
                '8uc1': cv2.COLOR_GRAY2BGR}.get(enc)
        return cv2.cvtColor(img, conv) if conv is not None else img
    if enc in ('yuv422_yuy2', 'yuyv', 'yuv422'):
        img = buf[:, :msg.width * 2].reshape(msg.height, msg.width, 2)
        code = cv2.COLOR_YUV2BGR_UYVY if enc == 'yuv422' else cv2.COLOR_YUV2BGR_YUYV
        return cv2.cvtColor(img, code)
    raise ValueError(f'unsupported image encoding {msg.encoding!r}')


def parse_feeds(entries):
    """['name=/topic', ...] -> {name: topic}."""
    feeds = {}
    for entry in entries:
        name, sep, topic = str(entry).partition('=')
        if sep and name.strip() and topic.strip():
            feeds[name.strip()] = topic.strip()
    return feeds


class Feed:

    def __init__(self, name, topic, publisher):
        self.name, self.topic, self.publisher = name, topic, publisher
        self.subscription = None
        self.last_sent = 0.0


class CameraStreamNode(Node):

    def __init__(self):
        super().__init__('camera_stream_node')
        entries = self.declare_parameter('feeds', DEFAULT_FEEDS).value
        self.max_width = int(self.declare_parameter('max_width', 480).value)
        self.period = 1.0 / max(0.5, float(self.declare_parameter('fps', 8.0).value))
        self.quality = int(self.declare_parameter('jpeg_quality', 65).value)
        self.max_depth = float(self.declare_parameter('max_depth_m', 4.0).value)

        out_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.feeds = {
            name: Feed(name, topic, self.create_publisher(
                CompressedImage, f'dashboard/camera/{name}/compressed', out_qos))
            for name, topic in parse_feeds(entries).items()
        }
        self.list_pub = self.create_publisher(String, 'dashboard/camera/feeds', 1)
        self.create_timer(1.0, self._update)
        self.get_logger().info(
            'Dashboard camera feeds: ' + ', '.join(f'{f.name} <- {f.topic}'
                                                   for f in self.feeds.values()))

    def _update(self):
        listing = []
        for feed in self.feeds.values():
            watched = feed.publisher.get_subscription_count() > 0
            if watched and feed.subscription is None:
                feed.subscription = self.create_subscription(
                    Image, feed.topic, lambda m, f=feed: self._on_image(f, m),
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
            elif not watched and feed.subscription is not None:
                self.destroy_subscription(feed.subscription)
                feed.subscription = None
            listing.append({'name': feed.name, 'topic': feed.topic,
                            'live': self.count_publishers(feed.topic) > 0})
        self.list_pub.publish(String(data=json.dumps(listing)))

    def _on_image(self, feed, msg):
        now = time.monotonic()
        if now - feed.last_sent < self.period:
            return
        feed.last_sent = now
        try:
            img = image_to_bgr(msg, self.max_depth)
        except ValueError as exc:
            self.get_logger().error(f'{feed.name}: {exc}', throttle_duration_sec=30.0)
            return
        h, w = img.shape[:2]
        if w > self.max_width:
            img = cv2.resize(img, (self.max_width, round(h * self.max_width / w)),
                             interpolation=cv2.INTER_AREA)
        ok, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = jpeg.tobytes()
        feed.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
