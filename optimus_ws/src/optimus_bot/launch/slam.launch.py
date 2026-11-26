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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg


def generate_launch_description():
    share = get_package_share_directory('optimus_bot')
    params = os.path.join(share, 'config', 'slam.yaml')

    node_kwargs = {
        'package': 'slam_toolbox',
        'executable': 'async_slam_toolbox_node',
        'name': 'slam_toolbox',
        'output': 'screen',
        'parameters': [params, {'use_sim_time': LaunchConfiguration('use_sim_time'),
                                'map_frame': LaunchConfiguration('map_frame'),
                                'odom_frame': LaunchConfiguration('odom_frame'),
                                'base_frame': LaunchConfiguration('base_frame'),
                                'scan_topic': LaunchConfiguration('scan_topic')}],
    }

    actions = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use simulation clock'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
    ]

    if os.environ.get('ROS_DISTRO', '') == 'humble':
        # Humble's slam_toolbox (2.6.x) is a plain node that starts mapping
        # immediately.
        actions.append(Node(**node_kwargs))
        return LaunchDescription(actions)

    # From Jazzy on, slam_toolbox is a lifecycle node. Started as a plain
    # Node it stays "unconfigured" forever: no scan subscription, no /map,
    # no serialize/deserialize services. Configure and activate it here,
    # mirroring slam_toolbox's own online_async_launch.py.
    slam_node = LifecycleNode(namespace='', **node_kwargs)
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam_node),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam_node,
        start_state='configuring',
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))]))

    actions += [slam_node, configure, activate]
    return LaunchDescription(actions)
