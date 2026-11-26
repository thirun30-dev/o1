"""Optimus in the simulated warehouse.

ros2 launch optimus_sim warehouse.launch.py                 # Gazebo + SLAM + Nav2 + RViz + trolley
ros2 launch optimus_sim warehouse.launch.py voice:=true     # + "Optimus, ..." voice control
ros2 launch optimus_sim warehouse.launch.py headless:=true  # no Gazebo GUI (saves RAM)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

PKG = "optimus_sim"


def _voice(context, *args, **kwargs):
    if LaunchConfiguration("voice").perform(context).lower() != "true":
        return []
    share = get_package_share_directory(PKG)
    voice_share = get_package_share_directory("optimus_voice")
    return [Node(
        package="optimus_voice", executable="voice_node", name="optimus_voice", output="screen",
        parameters=[os.path.join(voice_share, "config", "voice_params.yaml"), {
            "use_sim_time": True,
            "base_frame": "base_footprint",
            "waypoints_file": LaunchConfiguration("waypoints_file").perform(context)
                or os.path.join(share, "config", "sim_waypoints.yaml"),
        }])]


def generate_launch_description():
    share = get_package_share_directory(PKG)
    world = os.path.join(share, "worlds", "warehouse.sdf")
    urdf = os.path.join(share, "urdf", "optimus.urdf")
    nav_params = os.path.join(share, "config", "nav2_params.yaml")
    with open(urdf) as f:
        robot_description = f.read()

    use_rviz = LaunchConfiguration("rviz")
    use_nav = LaunchConfiguration("nav")
    headless = LaunchConfiguration("headless")
    sim = {"use_sim_time": True}

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": PythonExpression(
            ["'-r -s " + world + "' if '", headless, "' == 'true' else '-r " + world + "'"]),
            "on_exit_shutdown": "true"}.items())

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               parameters=[{"robot_description": robot_description, **sim}], output="screen")

    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-world", "warehouse", "-name", "optimus", "-topic", "robot_description",
                            "-x", "0", "-y", "0", "-z", "0.03", "-Y", "0"])

    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                  parameters=[{"config_file": os.path.join(share, "config", "bridge.yaml"), **sim}])

    traffic = Node(package=PKG, executable="traffic", output="screen", parameters=[sim])

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")),
        launch_arguments={"slam_params_file": os.path.join(share, "config", "slam.yaml"),
                          "use_sim_time": "true"}.items(),
        condition=IfCondition(use_nav))

    nav_nodes = ["controller_server", "smoother_server", "planner_server", "behavior_server",
                 "velocity_smoother", "collision_monitor", "bt_navigator", "waypoint_follower"]
    nav = [
        Node(package="nav2_controller", executable="controller_server", output="screen",
             parameters=[nav_params], remappings=[("cmd_vel", "cmd_vel_nav")]),
        Node(package="nav2_smoother", executable="smoother_server", output="screen", parameters=[nav_params]),
        Node(package="nav2_planner", executable="planner_server", output="screen", parameters=[nav_params]),
        Node(package="nav2_behaviors", executable="behavior_server", output="screen",
             parameters=[nav_params], remappings=[("cmd_vel", "cmd_vel_nav")]),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother", output="screen",
             parameters=[nav_params], remappings=[("cmd_vel", "cmd_vel_nav")]),
        Node(package="nav2_collision_monitor", executable="collision_monitor", output="screen",
             parameters=[nav_params]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", output="screen", parameters=[nav_params]),
        Node(package="nav2_waypoint_follower", executable="waypoint_follower", output="screen",
             parameters=[nav_params]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             output="screen", parameters=[{"autostart": True, "node_names": nav_nodes, **sim}]),
    ]
    nav_group = GroupAction(nav, condition=IfCondition(use_nav))

    rviz = Node(package="rviz2", executable="rviz2", output="log", condition=IfCondition(use_rviz),
                arguments=["-d", os.path.join(get_package_share_directory("nav2_bringup"),
                                              "rviz", "nav2_default_view.rviz")],
                parameters=[sim])

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("nav", default_value="true", description="SLAM + Nav2"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("voice", default_value="false"),
        DeclareLaunchArgument("waypoints_file", default_value=""),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(share, "worlds")),
        gz, rsp, spawn, bridge, traffic,
        # give Gazebo + the robot a few seconds before SLAM/Nav2 start
        TimerAction(period=5.0, actions=[slam, nav_group, rviz, OpaqueFunction(function=_voice)]),
    ])
