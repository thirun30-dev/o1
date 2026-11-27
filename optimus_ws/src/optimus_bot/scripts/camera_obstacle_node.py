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
Monocular camera obstacle detector -> LaserScan for Nav2.

Works with any single RGB camera (laptop webcam now, Raspberry Pi camera
later) and is light enough for a Pi:

1. Floor model. A patch at the bottom-centre of the image is assumed to be
   floor; hue / saturation / brightness histograms of it are learned and
   slowly updated. Pixels whose colour is rare on the floor are obstacles.
   Learning freezes while the patch stops looking like the floor (an obstacle
   right in front of the robot), and relearns if that lasts relearn_after_s
   (the floor type really changed).
2. Ground-plane geometry. With the camera's height and downward pitch known,
   every image row below the horizon maps to a distance on the floor. For
   each image column the lowest obstacle pixel is where the obstacle touches
   the floor, so its distance and bearing are known.
3. The hits are binned into a LaserScan on /camera/scan, frame
   camera_scan_link (level, at the camera). Rays with no obstacle are +inf so
   Nav2 clears them up to max_range.

Limits: objects the same colour as the floor are not seen, and the floor
reference patch itself is assumed free. The LiDAR stays the primary sensor.
Debug view: /camera/obstacle_debug (only rendered while something subscribes).
"""

import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image, LaserScan


# ---------------------------------------------------------------------------
# Pure helpers (unit tested in test/test_camera_obstacle.py)
# ---------------------------------------------------------------------------

def ground_lookup(width, height, fx, fy, cx, cy, cam_height, pitch):
    """
    Per-pixel floor distance and bearing for a camera pitched down by `pitch`.

    Returns (range, angle) arrays of shape (height, width) in the level frame
    at the camera (x forward, y left). Pixels at or above the horizon get
    range = inf.
    """
    u = np.arange(width, dtype=np.float64)
    v = np.arange(height, dtype=np.float64)
    xn = (u[None, :] - cx) / fx          # optical x: right
    yn = (v[:, None] - cy) / fy          # optical y: down
    s, c = math.sin(pitch), math.cos(pitch)
    forward = c - yn * s                 # (h, 1)
    up = -s - yn * c                     # (h, 1)
    left = -xn                           # (1, w)
    below = up < -1e-6
    t = np.where(below, cam_height / np.where(below, -up, 1.0), np.inf)
    with np.errstate(invalid='ignore'):
        x = np.broadcast_to(t * forward, (height, width))
        y = t * left
        rng = np.hypot(x, y)
        ang = np.arctan2(y, x)
    rng = np.where(np.broadcast_to(below, (height, width)), rng, np.inf)
    ang = np.where(np.isfinite(ang), ang, 0.0)
    return rng, ang


def lowest_runs(mask, min_run):
    """
    Lowest row of a vertical run of >= min_run True pixels, per column.

    Returns (row, has_hit); row is the bottom pixel of that run.
    """
    m = mask.astype(np.uint8)
    if min_run > 1:
        kernel = np.ones((min_run, 1), np.uint8)
        # Anchor at the kernel's last row: a pixel survives only if it and the
        # min_run - 1 pixels above it are set, so the survivor is the run's
        # bottom pixel.
        m = cv2.erode(m, kernel, anchor=(0, min_run - 1), borderType=cv2.BORDER_CONSTANT,
                      borderValue=0)
    has_hit = m.any(axis=0)
    row = m.shape[0] - 1 - np.argmax(m[::-1, :], axis=0)
    return row, has_hit


def bin_scan(hit_ranges, hit_angles, angle_min, angle_max, num_bins):
    """Minimum range per angular bin; bins without a hit are +inf."""
    ranges = np.full(num_bins, np.inf, dtype=np.float32)
    if len(hit_ranges):
        inc = (angle_max - angle_min) / num_bins
        idx = np.clip(((hit_angles - angle_min) / inc).astype(int), 0, num_bins - 1)
        np.minimum.at(ranges, idx, hit_ranges.astype(np.float32))
    return ranges


def _smooth(hist, circular):
    kernel = np.array([1.0, 2.0, 3.0, 2.0, 1.0]) / 9.0
    if circular:
        padded = np.concatenate([hist[-2:], hist, hist[:2]])
        return np.convolve(padded, kernel, mode='valid')
    return np.convolve(hist, kernel, mode='same')


class FloorModel:
    """Hue / saturation / value histograms of the floor."""

    H_BINS, S_BINS, V_BINS = 30, 32, 32

    def __init__(self, threshold, saturation_min):
        self.threshold = threshold
        self.saturation_min = saturation_min
        self.h = self.s = self.v = None

    @property
    def ready(self):
        return self.v is not None

    def _hists(self, hsv_pixels):
        h, s, v = hsv_pixels[:, 0], hsv_pixels[:, 1], hsv_pixels[:, 2]
        coloured = s >= self.saturation_min
        hh = np.bincount((h[coloured].astype(int) * self.H_BINS) // 180,
                         minlength=self.H_BINS).astype(np.float64)
        sh = np.bincount((s.astype(int) * self.S_BINS) // 256,
                         minlength=self.S_BINS).astype(np.float64)
        vh = np.bincount((v.astype(int) * self.V_BINS) // 256,
                         minlength=self.V_BINS).astype(np.float64)
        n = max(1, len(h))
        # Normalise by the total pixel count, so a mostly grey floor has a
        # weak hue histogram and coloured objects on it stand out.
        return _smooth(hh / n, True), _smooth(sh / n, False), _smooth(vh / n, False)

    def learn(self, hsv_pixels, rate=1.0):
        h, s, v = self._hists(hsv_pixels)
        if not self.ready or rate >= 1.0:
            self.h, self.s, self.v = h, s, v
        else:
            self.h = (1 - rate) * self.h + rate * h
            self.s = (1 - rate) * self.s + rate * s
            self.v = (1 - rate) * self.v + rate * v

    def classify(self, hsv):
        """Boolean mask of pixels that do not look like the floor."""
        h = (hsv[..., 0].astype(int) * self.H_BINS) // 180
        s = (hsv[..., 1].astype(int) * self.S_BINS) // 256
        v = (hsv[..., 2].astype(int) * self.V_BINS) // 256
        # Bin frequency relative to the most common bin of each channel.
        hp = self.h / max(self.h.max(), 1e-9) * min(1.0, self.h.sum() * 4)
        sp = self.s / max(self.s.max(), 1e-9)
        vp = self.v / max(self.v.max(), 1e-9)
        thr = self.threshold
        coloured = hsv[..., 1] >= self.saturation_min
        return (vp[v] < thr) | (sp[s] < thr) | (coloured & (hp[h] < thr))


def image_to_bgr(msg):
    """Convert a sensor_msgs/Image to a BGR numpy array (common encodings)."""
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ('bgr8', 'rgb8', 'bgra8', 'rgba8', 'mono8', '8uc1', '8uc3'):
        ch = {'bgr8': 3, 'rgb8': 3, '8uc3': 3, 'bgra8': 4, 'rgba8': 4,
              'mono8': 1, '8uc1': 1}[enc]
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * ch]
        img = img.reshape(msg.height, msg.width, ch)
        if enc == 'rgb8':
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if enc == 'bgra8':
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        if enc == 'rgba8':
            return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        if ch == 1:
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        return img
    if enc in ('yuv422_yuy2', 'yuyv', 'yuv422'):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 2]
        code = cv2.COLOR_YUV2BGR_YUYV if enc != 'yuv422' else cv2.COLOR_YUV2BGR_UYVY
        return cv2.cvtColor(img.reshape(msg.height, msg.width, 2), code)
    raise ValueError(f'unsupported image encoding {msg.encoding!r}')


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class CameraObstacleNode(Node):

    def __init__(self):
        super().__init__('camera_obstacle_node')
        p = self.declare_parameter
        self.cam_height = float(p('camera_height', 0.6).value)
        self.pitch = math.radians(float(p('pitch_deg', 20.0).value))
        self.hfov = math.radians(float(p('hfov_deg', 60.0).value))
        self.rotate_180 = bool(p('rotate_180', False).value)
        self.scan_frame = str(p('scan_frame', 'camera_scan_link').value)
        self.process_width = int(p('process_width', 160).value)
        self.min_period = 1.0 / max(0.1, float(p('max_rate_hz', 10.0).value))
        self.max_range = float(p('max_range', 2.0).value)
        self.num_bins = int(p('num_bins', 64).value)
        self.min_run = int(p('min_run_px', 3).value)
        self.bottom_crop = float(p('bottom_crop_ratio', 0.0).value)
        self.ref_h = float(p('ref_height_ratio', 0.15).value)
        self.ref_w = float(p('ref_width_ratio', 0.5).value)
        self.learn_rate = float(p('learn_rate', 0.05).value)
        self.ref_mismatch = float(p('ref_mismatch_ratio', 0.35).value)
        self.relearn_after = float(p('relearn_after_s', 3.0).value)
        self.model = FloorModel(float(p('obstacle_threshold', 0.06).value),
                                int(p('saturation_min', 40).value))

        self.scan_pub = self.create_publisher(
            LaserScan, 'camera/scan',
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))
        self.debug_pub = self.create_publisher(Image, 'camera/obstacle_debug', 2)
        # Reliable, depth 1: with best-effort QoS the default DDS drops ~1 MB
        # image messages completely (fragment loss), and both camera drivers
        # publish reliable. Depth 1 keeps only the newest frame.
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, 'camera/image_raw', self._on_image, latest)
        self.create_subscription(CameraInfo, 'camera/camera_info', self._on_info, latest)
        self.create_timer(2.0, self._watchdog)

        self.intrinsics = None       # (fx, fy, cx, cy, width, height) at full res
        self.lut_key = None
        self.last_processed = 0.0
        self.last_image = None
        self.mismatch_since = None
        self.get_logger().info(
            f'Camera obstacle detector: height {self.cam_height:.2f} m, '
            f'pitch {math.degrees(self.pitch):.0f} deg, range {self.max_range:.1f} m '
            f'-> /camera/scan ({self.scan_frame})')

    # -- inputs ------------------------------------------------------------

    def _on_info(self, msg):
        k = msg.k
        if k[0] > 0 and k[4] > 0 and msg.width > 0:
            self.intrinsics = (k[0], k[4], k[2], k[5], msg.width, msg.height)

    def _watchdog(self):
        if self.last_image is None or time.monotonic() - self.last_image > 2.0:
            self.get_logger().warning('No images on /camera/image_raw',
                                      throttle_duration_sec=15.0)

    # -- geometry ----------------------------------------------------------

    def _ensure_lut(self, width, height, full_w, full_h):
        intr = self.intrinsics
        if intr is None or intr[4] != full_w or intr[5] != full_h:
            fx = (full_w / 2.0) / math.tan(self.hfov / 2.0)
            intr = (fx, fx, full_w / 2.0, full_h / 2.0, full_w, full_h)
        key = (width, height, intr)
        if key == self.lut_key:
            return
        sx, sy = width / full_w, height / full_h
        fx, fy, cx, cy = intr[0] * sx, intr[1] * sy, intr[2] * sx, intr[3] * sy
        if self.rotate_180:
            # The image is rotated before processing, so the principal point
            # mirrors too.
            cx, cy = width - cx, height - cy
        rng, ang = ground_lookup(width, height, fx, fy, cx, cy, self.cam_height, self.pitch)
        self.rng, self.ang = rng, ang
        in_range = rng <= self.max_range
        if self.bottom_crop > 0:
            in_range[int(height * (1 - self.bottom_crop)):, :] = False
        self.in_range = in_range
        if not in_range.any():
            self.get_logger().error(
                'No floor within max_range is visible - check camera_height / pitch_deg '
                '(the camera may be looking above the horizon).')
        # Bearing of each column at its farthest in-range row (used for
        # free columns) and the scan's angular extent.
        far_row = np.argmax(in_range, axis=0)
        self.col_has_floor = in_range.any(axis=0)
        self.free_angle = ang[far_row, np.arange(width)]
        valid_angles = ang[in_range] if in_range.any() else np.array([-0.5, 0.5])
        self.angle_min = float(valid_angles.min())
        self.angle_max = float(valid_angles.max())
        if self.angle_max - self.angle_min < 1e-3:
            self.angle_max = self.angle_min + 1e-3
        self.lut_key = key

    # -- main loop ---------------------------------------------------------

    def _on_image(self, msg):
        self.last_image = time.monotonic()
        if self.last_image - self.last_processed < self.min_period:
            return
        self.last_processed = self.last_image
        try:
            bgr = image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=10.0)
            return

        full_h, full_w = bgr.shape[:2]
        width = min(self.process_width, full_w)
        height = max(1, round(full_h * width / full_w))
        small = cv2.resize(bgr, (width, height), interpolation=cv2.INTER_AREA)
        if self.rotate_180:
            small = cv2.rotate(small, cv2.ROTATE_180)
        small = cv2.GaussianBlur(small, (5, 5), 0)
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        self._ensure_lut(width, height, full_w, full_h)

        # Floor reference patch.
        bottom = int(height * (1 - self.bottom_crop))
        top = max(0, bottom - max(2, int(height * self.ref_h)))
        half = max(1, int(width * self.ref_w / 2))
        left, right = width // 2 - half, width // 2 + half
        ref = hsv[top:bottom, left:right].reshape(-1, 3)
        self._update_floor_model(ref)

        mask = self.model.classify(hsv)
        mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN,
                                np.ones((3, 3), np.uint8)).astype(bool)
        mask &= self.in_range

        row, has_hit = lowest_runs(mask, self.min_run)
        cols = np.nonzero(has_hit)[0]
        hit_r = self.rng[row[cols], cols]
        hit_a = self.ang[row[cols], cols]
        ranges = bin_scan(hit_r, hit_a, self.angle_min, self.angle_max, self.num_bins)

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        if scan.header.stamp.sec == 0 and scan.header.stamp.nanosec == 0:
            scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.scan_frame
        scan.angle_min = self.angle_min
        inc = (self.angle_max - self.angle_min) / self.num_bins
        scan.angle_increment = inc
        scan.angle_max = self.angle_min + inc * (self.num_bins - 1)
        scan.scan_time = self.min_period
        scan.range_min = 0.05
        scan.range_max = self.max_range
        scan.ranges = ranges.tolist()
        self.scan_pub.publish(scan)

        if self.debug_pub.get_subscription_count() > 0:
            self._publish_debug(msg, small, mask, row, has_hit, (top, bottom, left, right))

    def _update_floor_model(self, ref):
        if not self.model.ready:
            self.model.learn(ref)
            self.get_logger().info('Floor model learned from the reference patch')
            return
        mismatch = float(self.model.classify(ref[None, :, :]).mean())
        now = time.monotonic()
        if mismatch < self.ref_mismatch:
            self.mismatch_since = None
            self.model.learn(ref, self.learn_rate)
            return
        if self.mismatch_since is None:
            self.mismatch_since = now
            self.get_logger().info(
                f'Floor patch looks different ({mismatch:.0%}); holding the floor model')
        elif now - self.mismatch_since > self.relearn_after:
            self.model.learn(ref)
            self.mismatch_since = None
            self.get_logger().warning('Floor patch changed for a while; relearned the floor')

    def _publish_debug(self, msg, small, mask, row, has_hit, ref_box):
        dbg = small.copy()
        dbg[mask] = (0.5 * dbg[mask] + (0, 0, 127)).astype(np.uint8)
        top, bottom, left, right = ref_box
        cv2.rectangle(dbg, (left, top), (right - 1, bottom - 1), (255, 200, 0), 1)
        # Distance guide lines in the centre column (verify the mount by
        # putting an object at a known distance).
        centre = self.rng[:, dbg.shape[1] // 2]
        for dist in (0.5, 1.0, 1.5, 2.0):
            rows = np.nonzero(np.isfinite(centre) & (centre <= dist))[0]
            if len(rows) and dist <= self.max_range:
                r = int(rows[0])
                cv2.line(dbg, (0, r), (dbg.shape[1] - 1, r), (200, 200, 200), 1)
                cv2.putText(dbg, f'{dist:.1f}m', (2, max(8, r - 2)),
                            cv2.FONT_HERSHEY_PLAIN, 0.7, (255, 255, 255), 1)
        for c in np.nonzero(has_hit)[0]:
            cv2.circle(dbg, (int(c), int(row[c])), 1, (0, 255, 0), -1)
        dbg = cv2.resize(dbg, (dbg.shape[1] * 3, dbg.shape[0] * 3),
                         interpolation=cv2.INTER_NEAREST)
        out = Image()
        out.header = msg.header
        out.height, out.width = dbg.shape[:2]
        out.encoding = 'bgr8'
        out.step = out.width * 3
        out.data = dbg.tobytes()
        self.debug_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CameraObstacleNode()
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
