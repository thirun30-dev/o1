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
Optimus in simulation, driven through ros2_control like the real robot.

Backend (``sim_backend`` argument):
  classic - Gazebo Classic + gazebo_ros2_control (ROS 2 Humble)
  gz      - Gazebo Sim (Harmonic) + gz_ros2_control (Jazzy and newer, where
            Gazebo Classic is not available)
  auto    - classic on Humble, gz otherwise (default)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _load_drivetrain_geometry(pkg_share):
    """
    Read wheel geometry from config/drivetrain.yaml.

    That file is the single source of truth, so the URDF passed to
    robot_state_publisher never disagrees with optimus_drivetrain_node's
    kinematics/odometry.
    """
    drivetrain_yaml = os.path.join(pkg_share, 'config', 'drivetrain.yaml')

    with open(drivetrain_yaml, 'r') as f:
        params = yaml.safe_load(f)['optimus_drivetrain_node']['ros__parameters']

    return str(params['wheel_radius']), str(params['wheel_separation'])


def _resolve_backend(requested):
    if requested == 'auto':
        return 'classic' if os.environ.get('ROS_DISTRO', '') == 'humble' else 'gz'
    if requested not in ('classic', 'gz'):
        raise RuntimeError(f"sim_backend must be auto, classic or gz (got '{requested}')")
    return requested


def _simulator(context, pkg_share, xacro_file, drivetrain_file, rviz_file):
    backend = _resolve_backend(LaunchConfiguration('sim_backend').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time')
    wheel_radius, wheel_separation = _load_drivetrain_geometry(pkg_share)

    robot_description_content = Command([
        'bash -c "xacro ',
        xacro_file,
        ' use_sim:=true',
        f' sim_backend:={backend}',
        ' wheel_radius:=',
        wheel_radius,
        ' wheel_separation:=',
        wheel_separation,
        # Humble's gazebo_ros2_control passes this value back through the
        # ROS parameter-argument parser. Xacro's informational comments
        # contain ':' and make that parser reject an otherwise valid URDF.
        # Remove comments from this runtime copy only.
        " | perl -0pe 's/<!--.*?-->//gs'\"",
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str),
            'use_sim_time': use_sim_time,
        }],
    )

    if backend == 'classic':
        simulator = [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
            launch_arguments={
                # obstacle.world contains static geometry so the simulated
                # lidar has finite returns as soon as the robot is spawned.
                'world': os.path.join(pkg_share, 'worlds', 'obstacle.world'),
                'verbose': 'false',
            }.items(),
        )]
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'optimus',
                '-topic', 'robot_description',
                '-x', '0.0', '-y', '0.0', '-z', '0.10',
            ],
            output='screen',
        )
    else:
        headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
        world = os.path.join(pkg_share, 'worlds', 'obstacle_gz.sdf')
        simulator = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
                launch_arguments={
                    'gz_args': ('-r -s ' if headless else '-r ') + world,
                    'on_exit_shutdown': 'true',
                }.items(),
            ),
            # Gazebo Sim topics are not ROS topics: bridge the clock and the
            # lidar. Wheel commands/joint states go through gz_ros2_control.
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='optimus_gz_bridge',
                output='screen',
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                ],
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ]
        spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'optimus',
                '-topic', 'robot_description',
                '-x', '0.0', '-y', '0.0', '-z', '0.10',
            ],
            output='screen',
        )

    def spawner(controller):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                controller,
                '--controller-manager', '/controller_manager',
                '--controller-manager-timeout', '60',
            ],
            output='screen',
        )

    drivetrain_node = Node(
        package='optimus_bot',
        executable='optimus_drivetrain_node',
        name='optimus_drivetrain_node',
        output='screen',
        parameters=[drivetrain_file, {'use_sim_time': use_sim_time}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [
        robot_state_publisher,
        *simulator,
        spawn_entity,
        # The controller manager is created by the simulator only when the
        # model is inserted. Starting spawners after insertion avoids the
        # startup race.
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_entity,
            on_exit=[TimerAction(
                period=2.0,
                actions=[
                    spawner('joint_state_broadcaster'),
                    spawner('optimus_wheel_velocity_controller'),
                ],
            )],
        )),
        drivetrain_node,
        rviz,
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('optimus_bot')
    xacro_file = os.path.join(pkg_share, 'description', 'optimus.urdf.xacro')
    drivetrain_file = os.path.join(pkg_share, 'config', 'drivetrain.yaml')
    rviz_file = os.path.join(pkg_share, 'rviz', 'simulation.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description="Use the simulator's clock"),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz configured for simulation time'),
        DeclareLaunchArgument(
            'sim_backend', default_value='auto',
            description='auto | classic (Gazebo Classic) | gz (Gazebo Sim)'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='gz backend only: run the Gazebo Sim server without the GUI'),
        OpaqueFunction(
            function=_simulator,
            args=[pkg_share, xacro_file, drivetrain_file, rviz_file]),
    ])
