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
Depth image -> obstacle / clearing point clouds for Nav2.

Subscribes to a depth image (16UC1 millimetres or 32FC1 metres) and its
camera_info, back-projects every `pixel_stride`-th pixel, transforms the
points into base_link and splits them by height above the floor:

  /depth_camera/obstacles  points between min_obstacle_height and
                           max_obstacle_height (anything the robot would hit,
                           including low pallets and table tops), plus points
                           more than cliff_depth BELOW the floor (drop-offs).
                           Nav2 marks these.
  /depth_camera/clearing   every valid point, flattened. Nav2 raytraces to
                           them to clear space the camera sees as free.

Both clouds are in base_link and much lighter than the driver's own point
cloud, so this runs on a Raspberry Pi.
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
import tf2_ros


# ---------------------------------------------------------------------------
# Pure helpers (unit tested)
# ---------------------------------------------------------------------------

def depth_to_metres(msg_encoding, data, height, width, step):
    """Decode a depth sensor_msgs/Image payload to a float32 array in metres."""
    enc = msg_encoding.lower()
    buf = np.frombuffer(data, dtype=np.uint8).reshape(height, step)
    if enc in ('16uc1', 'mono16'):
        raw = buf[:, :width * 2].copy().view('<u2').reshape(height, width)
        return raw.astype(np.float32) / 1000.0
    if enc == '32fc1':
        return buf[:, :width * 4].copy().view('<f4').reshape(height, width)
    raise ValueError(f'unsupported depth encoding {msg_encoding!r}')


def quaternion_matrix(x, y, z, w):
    """3x3 rotation matrix of a unit quaternion."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def backproject(depth, fx, fy, cx, cy, stride, min_range, max_range):
    """Optical-frame points (N, 3) of every stride-th valid depth pixel."""
    sub = depth[::stride, ::stride]
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    valid = np.isfinite(sub) & (sub > min_range) & (sub < max_range)
    d = sub[valid]
    x = (u[valid] - cx) / fx * d
    y = (v[valid] - cy) / fy * d
    return np.stack([x, y, d], axis=1)


def classify_points(points_base, floor_z, min_h, max_h, cliff_depth):
    """Split base_link points into (obstacles, clearing)."""
    h = points_base[:, 2] - floor_z
    obstacle = (h >= min_h) & (h <= max_h)
    if cliff_depth > 0:
        obstacle |= h < -cliff_depth
    clearing = points_base.copy()
    clearing[:, 2] = floor_z + min_h
    return points_base[obstacle], clearing


def voxel_thin(points, size):
    """Keep one point per size^3 voxel."""
    if size <= 0 or len(points) == 0:
        return points
    keys = np.floor(points / size).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(idx)]


def make_cloud(header, points):
    """sensor_msgs/PointCloud2 (x, y, z float32) from an (N, 3) array."""
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i, n in enumerate('xyz')
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    msg.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
    return msg


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class DepthObstacleNode(Node):

    def __init__(self):
        super().__init__('depth_obstacle_node')
        p = self.declare_parameter
        self.target_frame = str(p('target_frame', 'base_link').value)
        self.floor_z = float(p('floor_z', -0.0655).value)
        self.min_period = 1.0 / max(0.1, float(p('max_rate_hz', 10.0).value))
        self.stride = max(1, int(p('pixel_stride', 4).value))
        self.min_range = float(p('min_range', 0.2).value)
        self.max_range = float(p('max_range', 3.0).value)
        self.min_h = float(p('min_obstacle_height', 0.04).value)
        self.max_h = float(p('max_obstacle_height', 0.8).value)
        self.cliff = float(p('cliff_depth', 0.08).value)
        self.voxel = float(p('voxel_size', 0.04).value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.transforms = {}        # source frame -> (R, t); the mount is static
        self.intrinsics = None
        self.last_processed = 0.0
        self.last_image = None

        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, 'depth/image', self._on_depth, latest)
        self.create_subscription(CameraInfo, 'depth/camera_info', self._on_info, latest)
        out = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.obstacle_pub = self.create_publisher(PointCloud2, 'depth_camera/obstacles', out)
        self.clearing_pub = self.create_publisher(PointCloud2, 'depth_camera/clearing', out)
        self.create_timer(3.0, self._watchdog)
        self.get_logger().info(
            f'Depth obstacles: {self.min_h:.2f}-{self.max_h:.2f} m above floor, '
            f'drop-offs > {self.cliff:.2f} m, range {self.max_range:.1f} m')

    def _watchdog(self):
        if self.last_image is None or time.monotonic() - self.last_image > 3.0:
            self.get_logger().warning('No depth images received',
                                      throttle_duration_sec=15.0)

    def _on_info(self, msg):
        if msg.k[0] > 0 and msg.k[4] > 0:
            self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5], msg.width, msg.height)

    def _transform(self, frame):
        if frame in self.transforms:
            return self.transforms[frame]
        try:
            tf = self.tf_buffer.lookup_transform(self.target_frame, frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self.get_logger().warning(
                f'Waiting for TF {self.target_frame} -> {frame}: {exc}',
                throttle_duration_sec=10.0)
            return None
        q, t = tf.transform.rotation, tf.transform.translation
        result = (quaternion_matrix(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z]))
        self.transforms[frame] = result
        return result

    def _on_depth(self, msg):
        self.last_image = time.monotonic()
        if self.last_image - self.last_processed < self.min_period:
            return
        if self.intrinsics is None:
            self.get_logger().warning('Waiting for depth camera_info',
                                      throttle_duration_sec=10.0)
            return
        transform = self._transform(msg.header.frame_id)
        if transform is None:
            return
        self.last_processed = self.last_image
        try:
            depth = depth_to_metres(msg.encoding, msg.data, msg.height, msg.width, msg.step)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=10.0)
            return

        fx, fy, cx, cy, iw, ih = self.intrinsics
        if iw and (iw != msg.width or ih != msg.height):
            sx, sy = msg.width / iw, msg.height / ih
            fx, fy, cx, cy = fx * sx, fy * sy, cx * sx, cy * sy
        pts = backproject(depth, fx, fy, cx, cy, self.stride, self.min_range, self.max_range)
        rot, trans = transform
        pts = pts @ rot.T + trans
        obstacles, clearing = classify_points(pts, self.floor_z, self.min_h, self.max_h,
                                              self.cliff)
        obstacles = voxel_thin(obstacles, self.voxel)
        clearing = voxel_thin(clearing, self.voxel * 2)

        header = msg.header
        header.frame_id = self.target_frame
        self.obstacle_pub.publish(make_cloud(header, obstacles))
        self.clearing_pub.publish(make_cloud(header, clearing))


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacleNode()
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

