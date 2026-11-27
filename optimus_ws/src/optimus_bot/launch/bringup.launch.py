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
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
import yaml
from launch_ros.substitutions import FindPackageShare


def _camera_stream(context):
    """
    Dashboard camera feeds (camera_stream_node), started with rosbridge.

    The depth topics depend on the depth camera driver, so they are read from
    the selected optimus_depth profile.
    """
    if LaunchConfiguration('use_rosbridge').perform(context).lower() != 'true':
        return []
    feeds = ['front=/camera/image_raw', 'obstacles=/camera/obstacle_debug']
    depth_profile = LaunchConfiguration('depth_camera').perform(context).strip()
    if depth_profile not in ('', 'none', 'false', 'off'):
        try:
            share = get_package_share_directory('optimus_depth')
            with open(os.path.join(share, 'config', 'depth_camera.yaml')) as f:
                prof = yaml.safe_load(f)['profiles'][depth_profile]
            feeds += [f"depth_color={prof['color_topic']}", f"depth={prof['depth_topic']}"]
        except Exception:  # unknown profile: depth_camera.launch.py reports it
            pass
    return [Node(
        package='optimus_bot',
        executable='camera_stream_node.py',
        name='camera_stream_node',
        output='screen',
        parameters=[{'feeds': feeds}],
    )]


def generate_launch_description():
    pkg_optimus = get_package_share_directory('optimus_bot')

    # Launch Arguments
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Launch Gazebo simulation stack if true, else real robot hardware'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock; false for the physical robot'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Start RViz2 alongside the system (dashboard is primary UI)'
    )
    use_rosbridge_arg = DeclareLaunchArgument(
        'use_rosbridge',
        default_value='true',
        description='Start rosbridge WebSocket server on port 9090'
    )
    rosbridge_ssl_arg = DeclareLaunchArgument(
        'rosbridge_ssl',
        default_value='false',
        description=('Serve rosbridge as wss:// (needed when the dashboard is opened '
                     'over https://, e.g. from Vercel). Certificate: '
                     'tools/make_rosbridge_cert.sh')
    )
    use_system_mgr_arg = DeclareLaunchArgument(
        'use_system_manager',
        default_value='true',
        description='Start AMR System Manager node'
    )
    use_voice_arg = DeclareLaunchArgument(
        'use_voice',
        default_value='false',
        description=('Start the optimus_voice node so the dashboard voice panel (Ctrl + ,) '
                     'can execute commands; speech is recognised in the browser')
    )
    use_location_mgr_arg = DeclareLaunchArgument(
        'use_location_manager',
        default_value='true',
        description='Start AMR Location Manager node'
    )
    maps_dir_arg = DeclareLaunchArgument(
        'maps_dir',
        default_value=str(Path.home() / 'optimus_data' / 'maps'),
        description='Path to maps directory'
    )
    map_frame_arg = DeclareLaunchArgument('map_frame', default_value='map')
    odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value='odom')
    base_frame_arg = DeclareLaunchArgument('base_frame', default_value='base_link')
    lidar_frame_arg = DeclareLaunchArgument('lidar_frame', default_value='lidar_link')
    depth_camera_arg = DeclareLaunchArgument(
        'depth_camera',
        default_value='none',
        description=('Depth camera profile from optimus_depth/config/depth_camera.yaml: '
                     'realsense, orbbec or none')
    )
    camera_arg = DeclareLaunchArgument(
        'camera',
        default_value='laptop',
        description=('Obstacle camera profile from config/camera.yaml: laptop (robot '
                     'laptop webcam), picam (Raspberry Pi camera) or none')
    )

    simulation = LaunchConfiguration('simulation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    use_rosbridge = LaunchConfiguration('use_rosbridge')
    use_system_manager = LaunchConfiguration('use_system_manager')
    use_location_manager = LaunchConfiguration('use_location_manager')
    use_voice = LaunchConfiguration('use_voice')
    maps_dir = LaunchConfiguration('maps_dir')
    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    lidar_frame = LaunchConfiguration('lidar_frame')

    # 1. Simulation Bringup
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_optimus, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': rviz
        }.items(),
        condition=IfCondition(simulation)
    )

    # 2. Real Hardware Bringup
    real_robot_launch = GroupAction(
        condition=UnlessCondition(simulation),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_optimus, 'launch', 'real_robot.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_optimus, 'launch', 'lidar.launch.py')),
                launch_arguments={'lidar_frame': lidar_frame}.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_optimus, 'launch', 'camera.launch.py')),
                launch_arguments={'camera': LaunchConfiguration('camera')}.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [FindPackageShare('optimus_depth'), 'launch', 'depth_camera.launch.py'])),
                launch_arguments={'depth_camera': LaunchConfiguration('depth_camera')}.items(),
                condition=UnlessCondition(PythonExpression(
                    ["'", LaunchConfiguration('depth_camera'), "' in ('', 'none', 'false', 'off')"]))
            )
        ]
    )

    # 3. rosbridge WebSocket Server
    rosbridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_optimus, 'launch', 'rosbridge.launch.py')),
        launch_arguments={'use_ssl': LaunchConfiguration('rosbridge_ssl')}.items(),
        condition=IfCondition(use_rosbridge)
    )

    # 4. AMR System Manager
    system_mgr_node = Node(
        package='optimus_bot',
        executable='system_manager_node.py',
        name='amr_system_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'maps_dir': maps_dir,
            'auto_ready': True,
            'map_frame': map_frame,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'lidar_frame': lidar_frame
        }],
        condition=IfCondition(use_system_manager)
    )

    # 5. AMR Location Manager
    location_mgr_node = Node(
        package='optimus_bot',
        executable='location_manager_node.py',
        name='amr_location_manager',
        output='screen',
        parameters=[{
            'locations_file': str(
                Path.home() / 'optimus_data' / 'locations' / 'warehouse_locations.yaml'),
            'use_sim_time': use_sim_time,
            'map_frame': map_frame,
            'base_frame': base_frame,
            'odom_frame': odom_frame
        }],
        condition=IfCondition(use_location_manager)
    )

    # 6. Voice command node (optional). The package is located lazily so bringup
    #    still works on machines where optimus_voice is not installed.
    voice_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('optimus_voice'), 'launch', 'voice.launch.py'])),
        launch_arguments={'enable_mic': 'false'}.items(),
        condition=IfCondition(use_voice)
    )

    return LaunchDescription([
        simulation_arg,
        use_sim_time_arg,
        rviz_arg,
        use_rosbridge_arg,
        rosbridge_ssl_arg,
        use_system_mgr_arg,
        use_location_mgr_arg,
        use_voice_arg,
        maps_dir_arg,
        map_frame_arg,
        odom_frame_arg,
        base_frame_arg,
        lidar_frame_arg,
        camera_arg,
        depth_camera_arg,
        sim_launch,
        real_robot_launch,
        rosbridge_launch,
        system_mgr_node,
        location_mgr_node,
        voice_launch,
        OpaqueFunction(function=_camera_stream)
    ])
