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
Publish a V4L2 camera (laptop webcam / USB camera) with OpenCV.

Topics: camera/image_raw (bgr8) and camera/camera_info. The intrinsics are
approximated from hfov_deg (no calibration needed for obstacle avoidance).
If the device disappears the node keeps retrying instead of exiting.

The Raspberry Pi camera module is not a plain V4L2 device; camera.launch.py
uses camera_ros (libcamera) for it instead of this node.
"""

import math
import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera')
        self.device = str(self.declare_parameter('device', '/dev/video0').value)
        self.width = int(self.declare_parameter('width', 640).value)
        self.height = int(self.declare_parameter('height', 480).value)
        self.fps = float(self.declare_parameter('fps', 15.0).value)
        self.hfov = math.radians(float(self.declare_parameter('hfov_deg', 60.0).value))
        self.frame_id = str(self.declare_parameter('frame_id', 'camera_optical_frame').value)
        self.fourcc = str(self.declare_parameter('fourcc', 'MJPG').value)

        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', qos)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', qos)

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _open(self):
        source = int(self.device) if self.device.isdigit() else self.device
        cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return None
        if self.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc[:4]))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _camera_info(self, width, height, stamp):
        fx = (width / 2.0) / math.tan(self.hfov / 2.0)
        cx, cy = width / 2.0, height / 2.0
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width, info.height = width, height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [fx, 0.0, cx, 0.0, fx, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fx, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _loop(self):
        cap = None
        min_period = 1.0 / self.fps if self.fps > 0 else 0.0
        last = 0.0
        while self._running and rclpy.ok():
            if cap is None:
                cap = self._open()
                if cap is None:
                    self.get_logger().warning(
                        f'Cannot open camera {self.device}; retrying in 2 s',
                        throttle_duration_sec=10.0)
                    time.sleep(2.0)
                    continue
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                self.get_logger().info(f'Camera {self.device} opened at {w}x{h}')

            ok, frame = cap.read()
            if not ok or frame is None:
                self.get_logger().warning(f'Camera {self.device} read failed; reopening')
                cap.release()
                cap = None
                time.sleep(1.0)
                continue

            now = time.monotonic()
            if now - last < min_period * 0.9:
                continue
            last = now

            stamp = self.get_clock().now().to_msg()
            h, w = frame.shape[:2]
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.height, msg.width = h, w
            msg.encoding = 'bgr8'
            msg.step = w * 3
            msg.data = frame.tobytes()
            self.image_pub.publish(msg)
            self.info_pub.publish(self._camera_info(w, h, stamp))

        if cap is not None:
            cap.release()

    def destroy_node(self):
        self._running = False
        self._thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
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
