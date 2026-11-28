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
Launch the RPLIDAR A1 node and the scan filter.

config/lidar.yaml is the single source of truth for the serial port, baud
rate and frame_id (see description/sensors/lidar.xacro for the matching TF
frame). The LiDAR sits on top of the body, so the full 360-degree scan is
kept by default; front_scan_filter only caps the ray count (set
lidar_fov_deg:=180 to crop to the front again).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

# Used only when the stable by-id link from lidar.yaml does not exist (e.g. a
# different adapter); this was the previous hard-coded port.
FALLBACK_SERIAL_PORT = '/dev/ttyUSB0'


def _lidar(context):
    share = get_package_share_directory('optimus_bot')
    with open(os.path.join(share, 'config', 'lidar.yaml')) as f:
        params = yaml.safe_load(f)['rplidar_node']['ros__parameters']

    actions = []
    serial_port = LaunchConfiguration('serial_port').perform(context)
    if not serial_port:
        serial_port = params['serial_port']
        if not os.path.exists(serial_port):
            actions.append(LogInfo(msg=(
                f'LiDAR port {serial_port} from lidar.yaml not found; '
                f'falling back to {FALLBACK_SERIAL_PORT}')))
            serial_port = FALLBACK_SERIAL_PORT

    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'serial_baudrate': int(params['serial_baudrate']),
            'frame_id': LaunchConfiguration('lidar_frame'),
            'inverted': bool(params.get('inverted', False)),
            'angle_compensate': True,
        }],
        remappings=[('scan', '/scan_raw')],
    )

    actions += [
        LogInfo(msg=f'Starting RPLIDAR on {serial_port}'),
        # Give the USB serial adapter and the A1 motor time to settle before
        # the first scan-mode handshake. A rapid restart after Ctrl-C can
        # otherwise return 80008002 even though the health check succeeds.
        TimerAction(period=10.0, actions=[lidar_node]),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lidar_frame', default_value='lidar_link'),
        DeclareLaunchArgument(
            'serial_port', default_value='',
            description='Override the LiDAR serial port (default: config/lidar.yaml)'),
        DeclareLaunchArgument(
            'lidar_yaw_deg',
            default_value='180',
            description='LiDAR mount yaw (deg); /scan is rotated by this so it is robot-forward'),
        DeclareLaunchArgument(
            'lidar_fov_deg',
            default_value='360',
            description='Degrees of the scan published on /scan, centred on +X'),
        OpaqueFunction(function=_lidar),
        Node(
            package='optimus_bot',
            executable='front_scan_filter.py',
            name='front_scan_filter',
            output='screen',
            parameters=[{
                "field_of_view_deg": ParameterValue(LaunchConfiguration("lidar_fov_deg"), value_type=float),
                "angle_offset_deg": ParameterValue(LaunchConfiguration("lidar_yaw_deg"), value_type=float),
            }],
        ),
    ])
