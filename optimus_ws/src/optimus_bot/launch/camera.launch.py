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
Camera driver + camera obstacle detector.

    ros2 launch optimus_bot camera.launch.py                 # laptop webcam
    ros2 launch optimus_bot camera.launch.py camera:=picam   # Raspberry Pi camera
    ros2 launch optimus_bot camera.launch.py camera:=none    # no camera

config/camera.yaml is the single source of truth for the profile (driver,
resolution, field of view) and the camera mount. The mount is published here
as static TF (base_link -> camera_link -> camera_optical_frame, and the level
base_link -> camera_scan_link used by /camera/scan) and passed to the
detector, so swapping laptop <-> Pi camera only needs `camera:=`.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _static_tf(name, parent, child, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        output='log',
        arguments=['--x', str(x), '--y', str(y), '--z', str(z),
                   '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
                   '--frame-id', parent, '--child-frame-id', child],
    )


def _camera(context):
    profile_name = LaunchConfiguration('camera').perform(context).strip()
    if profile_name in ('', 'none', 'false', 'off'):
        return [LogInfo(msg='Camera disabled (camera:=none)')]

    share = get_package_share_directory('optimus_bot')
    with open(os.path.join(share, 'config', 'camera.yaml')) as f:
        cfg = yaml.safe_load(f)
    profiles = cfg['profiles']
    if profile_name not in profiles:
        raise RuntimeError(
            f"Unknown camera profile '{profile_name}'. "
            f"Use one of: {', '.join(profiles)} or none (config/camera.yaml)")
    prof = profiles[profile_name]
    mount = prof['mount']

    # base_link sits at hub-motor axle height (wheel_radius above the floor).
    with open(os.path.join(share, 'config', 'drivetrain.yaml')) as f:
        wheel_radius = float(yaml.safe_load(f)['optimus_drivetrain_node']
                             ['ros__parameters']['wheel_radius'])
    x, y = float(mount['x']), float(mount['y'])
    z_floor = float(mount['z_above_floor'])
    z = z_floor - wheel_radius
    pitch = math.radians(float(mount['pitch_deg']))
    yaw = math.radians(float(mount.get('yaw_deg', 0.0)))
    rotate_180 = bool(prof.get('rotate_180', False))

    actions = [
        LogInfo(msg=f"Camera profile '{profile_name}' ({prof['driver']}): "
                    f'{z_floor:.2f} m above floor, pitched {mount["pitch_deg"]} deg down'),
        _static_tf('camera_link_tf', 'base_link', 'camera_link', x, y, z, 0.0, pitch, yaw),
        _static_tf('camera_optical_tf', 'camera_link', 'camera_optical_frame',
                   roll=-math.pi / 2, yaw=-math.pi / 2),
        _static_tf('camera_scan_tf', 'base_link', 'camera_scan_link', x, y, z, yaw=yaw),
    ]

    driver = prof['driver']
    if driver == 'opencv':
        actions.append(Node(
            package='optimus_bot',
            executable='camera_publisher_node.py',
            name='camera',
            output='screen',
            parameters=[{
                'device': str(prof.get('device', '/dev/video0')),
                'width': int(prof['width']),
                'height': int(prof['height']),
                'fps': float(prof.get('fps', 15)),
                'hfov_deg': float(prof['hfov_deg']),
                'frame_id': 'camera_optical_frame',
            }],
        ))
    elif driver == 'libcamera':
        # camera_ros (ros-$ROS_DISTRO-camera-ros) publishes ~/image_raw and
        # ~/camera_info; with name 'camera' that is /camera/image_raw.
        actions.append(Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'camera': int(prof.get('camera_index', 0)),
                'width': int(prof['width']),
                'height': int(prof['height']),
                'format': str(prof.get('format', 'RGB888')),
                'frame_id': 'camera_optical_frame',
            }],
        ))
    else:
        raise RuntimeError(f"Unknown camera driver '{driver}' (use opencv or libcamera)")

    actions.append(Node(
        package='optimus_bot',
        executable='camera_obstacle_node.py',
        name='camera_obstacle_node',
        output='screen',
        parameters=[
            cfg['camera_obstacle_node']['ros__parameters'],
            {
                'camera_height': z_floor,
                'pitch_deg': float(mount['pitch_deg']),
                'hfov_deg': float(prof['hfov_deg']),
                'rotate_180': rotate_180,
                'scan_frame': 'camera_scan_link',
            },
        ],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera', default_value='laptop',
            description='Camera profile from config/camera.yaml: laptop, picam or none'),
        OpaqueFunction(function=_camera),
    ])
