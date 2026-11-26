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
Crop (optionally) and downsample the raw RPLidar scan into /scan.

Optimus carries the RPLidar A1M8 on top of the body at the front, above the
storage container, so by default the full 360-degree scan is kept
(field_of_view_deg: 360). If something on the top deck starts blocking the
rear, lower field_of_view_deg (e.g. 270 or 180) to crop to a sector centred
on the robot's +X axis.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan


class FrontScanFilter(Node):
    """Keep rays within +/- field_of_view/2 of +X and cap the ray count."""

    def __init__(self):
        super().__init__('front_scan_filter')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan_raw',
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.publisher = self.create_publisher(
            LaserScan,
            'scan',
            # The raw driver is consumed with sensor-data QoS, but the
            # filtered topic is also consumed by the system manager and some
            # Nav2 components using the default reliable QoS. A reliable
            # publisher is compatible with both subscriber types.
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        # Some A1 firmware modes produce several thousand samples per scan.
        # That is useful for the driver, but unnecessarily large for Nav2 and
        # especially for rosbridge/WebSocket clients.  Keep the ROS scan
        # bounded while retaining enough angular resolution for SLAM/Nav2.
        self.max_output_rays = int(self.declare_parameter('max_output_rays', 720).value)
        fov_deg = float(self.declare_parameter('field_of_view_deg', 360.0).value)
        fov_deg = min(360.0, max(10.0, fov_deg))
        self.half_fov = math.radians(fov_deg) / 2.0
        self.full_circle = fov_deg >= 359.9
        self.get_logger().info(f'Publishing {fov_deg:.0f} deg of the LiDAR scan on /scan')

    @staticmethod
    def _normalize(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _scan_callback(self, scan):
        # Normalizing and sorting handles drivers that report angles as either
        # [-pi, pi] or [0, 2*pi], including the forward-sector wraparound.
        selected = []
        for index, range_value in enumerate(scan.ranges):
            angle = self._normalize(scan.angle_min + index * scan.angle_increment)
            if self.full_circle or -self.half_fov <= angle <= self.half_fov:
                selected.append((angle, range_value, index))

        if not selected:
            self.get_logger().warning('Received a scan with no rays in the field of view')
            return

        selected.sort(key=lambda item: item[0])
        sample_step = max(1, math.ceil(len(selected) / self.max_output_rays))
        selected = selected[::sample_step]
        output = LaserScan()
        output.header = scan.header
        output.angle_min = selected[0][0]
        output.angle_max = selected[-1][0]
        output.angle_increment = scan.angle_increment * sample_step
        output.time_increment = scan.time_increment
        output.scan_time = scan.scan_time
        output.range_min = scan.range_min
        output.range_max = scan.range_max
        output.ranges = [item[1] for item in selected]
        if scan.intensities:
            output.intensities = [scan.intensities[item[2]] for item in selected]
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = FrontScanFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
