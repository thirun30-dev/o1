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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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

    # wss:// is required when the dashboard is opened over https:// (for
    # example the Vercel deployment): browsers block plain ws:// from a
    # secure page.  Create the certificate with tools/make_rosbridge_cert.sh.
    use_ssl_arg = DeclareLaunchArgument(
        'use_ssl',
        default_value='false',
        description='Serve wss:// (TLS) instead of ws://'
    )
    certfile_arg = DeclareLaunchArgument(
        'certfile',
        default_value=os.path.expanduser('~/.optimus_certs/rosbridge.crt'),
        description='TLS certificate (PEM) used when use_ssl is true'
    )
    keyfile_arg = DeclareLaunchArgument(
        'keyfile',
        default_value=os.path.expanduser('~/.optimus_certs/rosbridge.key'),
        description='TLS private key (PEM) used when use_ssl is true'
    )

    common_params = {
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
    }

    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[common_params],
        condition=UnlessCondition(LaunchConfiguration('use_ssl'))
    )

    rosbridge_ssl_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            **common_params,
            'certfile': LaunchConfiguration('certfile'),
            'keyfile': LaunchConfiguration('keyfile'),
        }],
        condition=IfCondition(LaunchConfiguration('use_ssl'))
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
        use_ssl_arg,
        certfile_arg,
        keyfile_arg,
        rosbridge_node,
        rosbridge_ssl_node,
        rosapi_node
    ])
