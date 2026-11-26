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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='9090',
        description='Port for websocket server'
    )
    address_arg = DeclareLaunchArgument(
        'address',
        default_value='',
        description=('Address for websocket server '
                     '(empty string binds to all IPv4 and IPv6 interfaces)')
    )

    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'address': LaunchConfiguration('address'),
            'max_message_size': 20000000,
            # Keep only a small per-client backlog.  Teleop and dashboard
            # control packets are time-sensitive; stale packets must be
            # dropped instead of accumulating until rosbridge reports a
            # full incoming queue.
            'incoming_queue_size': 64,
            'write_queue_size': 64,
            'send_action_goals_in_new_thread': True,
            'call_services_in_new_thread': True,
            'unregister_timeout': 10.0
        }]
    )

    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen'
    )

    return LaunchDescription([
        port_arg,
        address_arg,
        rosbridge_node,
        rosapi_node
    ])
