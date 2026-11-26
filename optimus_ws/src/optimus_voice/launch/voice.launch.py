import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("optimus_voice")
    params = os.path.join(share, "config", "voice_params.yaml")
    default_wps = os.path.join(share, "config", "waypoints.yaml")

    wps = LaunchConfiguration("waypoints_file")
    mic = LaunchConfiguration("enable_mic")
    bridge = LaunchConfiguration("rosbridge")

    return LaunchDescription([
        DeclareLaunchArgument("waypoints_file", default_value=default_wps,
                              description="Use a path outside install/ so 'save location' persists"),
        DeclareLaunchArgument("enable_mic", default_value="true"),
        DeclareLaunchArgument("rosbridge", default_value="false",
                              description="Start rosbridge_websocket for the dashboard"),
        Node(package="optimus_voice", executable="voice_node", name="optimus_voice",
             output="screen",
             parameters=[params, {"waypoints_file": wps, "enable_mic": mic}]),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(
                get_package_share_directory("rosbridge_server"),
                "launch", "rosbridge_websocket_launch.xml")),
            condition=IfCondition(bridge)),
    ])
