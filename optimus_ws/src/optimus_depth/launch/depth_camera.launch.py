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
Depth camera driver + mount TF + depth_obstacle_node.

    ros2 launch optimus_depth depth_camera.launch.py depth_camera:=realsense
    ros2 launch optimus_depth depth_camera.launch.py depth_camera:=orbbec

Profiles, mount and detector settings: config/depth_camera.yaml.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _wheel_radius():
    """Floor height below base_link (base_link is at hub-motor axle height)."""
    try:
        share = get_package_share_directory('optimus_bot')
        with open(os.path.join(share, 'config', 'drivetrain.yaml')) as f:
            return float(yaml.safe_load(f)['optimus_drivetrain_node']
                         ['ros__parameters']['wheel_radius'])
    except Exception:  # optimus_bot not built: fall back to the 131 mm wheels
        return 0.0655


def _depth_camera(context):
    name = LaunchConfiguration('depth_camera').perform(context).strip()
    if name in ('', 'none', 'false', 'off'):
        return [LogInfo(msg='Depth camera disabled (depth_camera:=none)')]

    share = get_package_share_directory('optimus_depth')
    with open(os.path.join(share, 'config', 'depth_camera.yaml')) as f:
        cfg = yaml.safe_load(f)
    if name not in cfg['profiles']:
        raise RuntimeError(
            f"Unknown depth camera profile '{name}'. Use one of: "
            f"{', '.join(cfg['profiles'])} or none (optimus_depth/config/depth_camera.yaml)")
    prof = cfg['profiles'][name]
    mount = prof['mount']
    wheel_radius = _wheel_radius()
    z = float(mount['z_above_floor']) - wheel_radius
    base_frame = f"{prof['params'].get('camera_name', 'depth_camera')}_link"
    depth_topic = prof['depth_topic']
    info_topic = depth_topic.rsplit('/', 1)[0] + '/camera_info'

    return [
        LogInfo(msg=f"Depth camera '{name}' ({prof['package']}), "
                    f"{mount['z_above_floor']} m above floor, pitched {mount['pitch_deg']} deg"),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='depth_camera_mount_tf',
            output='log',
            arguments=[
                '--x', str(mount['x']), '--y', str(mount['y']), '--z', str(z),
                '--roll', '0', '--pitch', str(math.radians(float(mount['pitch_deg']))),
                '--yaw', str(math.radians(float(mount.get('yaw_deg', 0.0)))),
                '--frame-id', 'base_link', '--child-frame-id', base_frame],
        ),
        Node(
            package=prof['package'],
            executable=prof['executable'],
            name='depth_camera',
            namespace=prof.get('namespace', ''),
            output='screen',
            parameters=[prof['params']],
        ),
        Node(
            package='optimus_depth',
            executable='depth_obstacle_node',
            name='depth_obstacle_node',
            output='screen',
            parameters=[cfg['depth_obstacle_node']['ros__parameters'],
                        {'floor_z': -wheel_radius}],
            remappings=[('depth/image', depth_topic), ('depth/camera_info', info_topic)],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'depth_camera', default_value='realsense',
            description='Depth camera profile from config/depth_camera.yaml: '
                        'realsense, orbbec or none'),
        OpaqueFunction(function=_depth_camera),
    ])
