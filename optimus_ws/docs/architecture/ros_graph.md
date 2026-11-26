# ROS 2 Computation Graph

When the physical robot is launched with `ros2 launch optimus_bot bringup.launch.py`, the following graph is established.

## Always-on nodes
- `/controller_manager`: the `ros2_control` loop at 50 Hz, hosting the ZLAC8015D hardware plugin.
- `/optimus_wheel_velocity_controller`: `forward_command_controller` passing per-wheel rad/s to the hardware.
- `/joint_state_broadcaster`: publishes `sensor_msgs/JointState`.
- `/optimus_drivetrain_node`: differential-drive kinematics, 0.5 s command watchdog, odometry and `odom -> base_link`.
- `/robot_state_publisher`: parses the URDF and broadcasts static transforms.
- `/rplidar_node` and `/front_scan_filter`: LiDAR driver (`/scan_raw`) and filtered `/scan`.
- `/rosbridge_websocket` and `/rosapi`: dashboard WebSocket bridge on port 9090.
- `/amr_system_manager` and `/amr_location_manager`.
- `/optimus_voice` (with `use_voice:=true`).

## Mode-dependent nodes (started by the system manager)
- Mapping: `/slam_toolbox`.
- Localization: `/map_server`, `/amcl`, `/lifecycle_manager_localization`.
- Navigation: `/planner_server`, `/controller_server`, `/smoother_server`, `/behavior_server`,
  `/bt_navigator`, `/waypoint_follower`, `/velocity_smoother`, `/lifecycle_manager_navigation`.

The dashboard's **Diagnostics** page shows which of these are running (via `rosapi`).

## Core topics
- `/cmd_vel` (`geometry_msgs/Twist`): target velocity from teleop, the dashboard or Nav2. Consumed by `/optimus_drivetrain_node`.
- `/optimus_wheel_velocity_controller/commands` (`std_msgs/Float64MultiArray`): `[left, right]` wheel rad/s.
- `/joint_states` (`sensor_msgs/JointState`): wheel positions and velocities.
- `/odom` (`nav_msgs/Odometry`): published by `/optimus_drivetrain_node`.
- `/tf` and `/tf_static` (`tf2_msgs/TFMessage`): coordinate frame transformations.
- `/scan` (`sensor_msgs/LaserScan`): filtered 2D laser scan.
