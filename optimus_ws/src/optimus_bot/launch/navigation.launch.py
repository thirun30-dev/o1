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
Optimus Nav2 navigation stack (without localization).

The nodes are launched explicitly instead of including nav2_bringup's
navigation_launch.py. From Jazzy on, that file also starts route_server,
collision_monitor and docking_server; config/nav2.yaml has no configuration
for them, collision_monitor and docking_server fail to configure, and the
lifecycle manager then aborts the whole stack. This list matches
lifecycle_manager_navigation.node_names in config/nav2.yaml and the nodes
system_manager_node.py waits for, and works unchanged on Humble.
"""

import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LIFECYCLE_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
]


def _params_for_distro(params):
    """
    Adapt config/nav2.yaml's Humble plugin names to the running distro.

    The file uses Humble names such as "nav2_behaviors/Spin".

    From Iron on, Nav2 plugins are registered only by C++ type
    ("nav2_behaviors::Spin"); the "/" spelling makes planner_server and
    behavior_server fail to configure. Translate on newer distros.
    """
    if os.environ.get('ROS_DISTRO', '') == 'humble':
        return params
    with open(params) as f:
        text = f.read()
    text = re.sub(r'(plugin:\s*")(nav2_\w+)/(\w+)"', r'\1\2::\3"', text)
    out = tempfile.NamedTemporaryFile(
        'w', prefix='optimus_nav2_', suffix='.yaml', delete=False)
    with out:
        out.write(text)
    return out.name


def generate_launch_description():
    share = get_package_share_directory('optimus_bot')
    params = _params_for_distro(os.path.join(share, 'config', 'nav2.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    common = [params, {'use_sim_time': use_sim_time}]
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    def nav_node(package, executable, remappings=()):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            output='screen',
            parameters=common,
            remappings=tf_remaps + list(remappings),
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use simulation clock'),
        # controller/behavior output -> velocity_smoother -> /cmd_vel
        nav_node('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        nav_node('nav2_smoother', 'smoother_server'),
        nav_node('nav2_planner', 'planner_server'),
        nav_node('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
        nav_node('nav2_bt_navigator', 'bt_navigator'),
        nav_node('nav2_waypoint_follower', 'waypoint_follower'),
        nav_node('nav2_velocity_smoother', 'velocity_smoother',
                 [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': True,
                         'node_names': LIFECYCLE_NODES}],
        ),
    ])
