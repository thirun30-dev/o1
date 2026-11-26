# Optimus Bot

ROS 2 (Humble / Jazzy) robot integration package for the Optimus differential-drive AMR (RPLIDAR A1, SLAM Toolbox, AMCL, Nav2). Low-level ZLAC8015D Modbus communication lives in the separate `zlac_drive` package - this package owns robot geometry, TF frames, ros2_control wiring, drivetrain kinematics/odometry, and navigation launch/configuration.

## Architecture

```
RPLIDAR A1 --/scan--> SLAM Toolbox --map->odom--> Nav2 --/cmd_vel-->
                                                              |
                                                              v
                                          optimus_drivetrain_node
                                     (differential-drive kinematics,
                                      cmd_vel timeout watchdog)
                                                              |
                                          per-wheel rad/s, published to
                                   /optimus_wheel_velocity_controller/commands
                                                              |
                                                              v
                              forward_command_controller (ros2_control)
                                                              |
                                                              v
                                   zlac_drive/ZlacSystemHardware -> Modbus RTU -> ZLAC8015D
                                                              |
                                                              v
                                                joint_states (position/velocity)
                                                              |
                                                              v
                                          optimus_drivetrain_node
                                   (velocity-integration odometry, odom->base_link)
```

**`diff_drive_controller` is intentionally not used.** Wheel-command kinematics and odometry are owned directly by Optimus's own `optimus_drivetrain_node` (`src/drivetrain_node.cpp`); `ros2_control` only forwards the resulting per-wheel velocity commands via `forward_command_controller`.

## Single source of truth for wheel geometry

`config/drivetrain.yaml` is the only place `wheel_radius` and `wheel_separation` are defined. Launch files read it and pass both values into the xacro command line (`description/wheels/wheels.xacro` consumes them as `xacro:arg`s with matching defaults), and `optimus_drivetrain_node` loads the same file as its ROS parameters file. Do not hard-code either value anywhere else.

- `wheel_radius = 0.0655 m`: `DERIVED` from the ZLLG50ASM200 V2.0 wheel diameter (131 +-2 mm, `SOURCE_VERIFIED`) / 2.
- `wheel_separation = 0.265 m`: measured wheel-centre separation (`wheel_separation_verified: true`). Re-measure after any chassis change.

## Odometry approach

`optimus_drivetrain_node` integrates **wheel velocity** feedback (`/joint_states`, RPM-derived rad/s, `PHYSICALLY_VERIFIED` scale) rather than position feedback, because the ZLAC8015D position-register scale is `CALIBRATION_REQUIRED` (see `zlac_drive/README.md`). This is a deliberate open-loop-on-position / closed-loop-on-velocity dead-reckoning source; like any dead-reckoning estimate it will drift and is corrected by SLAM Toolbox/AMCL publishing `map -> odom`, never by this node.

## TF tree

```
map -> odom      (SLAM Toolbox while mapping, AMCL while localizing - never both at once)
odom -> base_link (optimus_drivetrain_node)
base_link -> lidar_link (robot_state_publisher, from the URDF)
```

## Safety: command timeout

`optimus_drivetrain_node` runs a timer at `control_frequency` (default matches `controller_manager`'s `update_rate`, 50 Hz) that re-publishes the wheel command **every cycle** - zero velocity if no `/cmd_vel` has arrived within `cmd_vel_timeout` (default 0.5 s). This mirrors how `diff_drive_controller` itself implements `cmd_vel_timeout`; simply reacting to `/cmd_vel` callbacks would leave the last non-zero command latched on the `forward_command_controller` command interface forever if `/cmd_vel` stopped arriving.

Independently, every wheel command is hard-clamped inside the node to the motor's physical peak speed (350 RPM, converted to rad/s) regardless of the configured `max_linear_velocity`/`max_angular_velocity`.

## Build and test

```bash
cd ~/optimus_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

## Bring-up sequence (real hardware)

Normal operation is one command; the dashboard then starts mapping,
localization and navigation through the system manager:

```bash
ros2 launch optimus_bot bringup.launch.py            # add use_voice:=true for voice commands
```

Individual pieces, for debugging:

```bash
ros2 launch optimus_bot real_robot.launch.py
ros2 launch optimus_bot lidar.launch.py              # serial_port:=... overrides config/lidar.yaml
ros2 launch optimus_bot slam.launch.py
# drive the robot around, then:
ros2 run nav2_map_server map_saver_cli -f ~/optimus_data/maps/my_map
```

Serial devices use stable `/dev/serial/by-id/` names: the ZLAC8015D adapter in
`description/control/real_hardware.xacro`, the RPLIDAR in `config/lidar.yaml`.
See `docs/RASPBERRY_PI.md` for a full first power-up procedure.

For saved-map localization + navigation:

```bash
ros2 launch optimus_bot real_robot.launch.py
ros2 launch optimus_bot lidar.launch.py
ros2 launch optimus_bot localization.launch.py map:=/absolute/path/to/my_map.yaml
ros2 launch optimus_bot navigation.launch.py
```

## Bring-up sequence (Gazebo simulation)

```bash
ros2 launch optimus_bot simulation.launch.py      # sim_backend:=auto|classic|gz, headless:=true
```

This starts Gazebo Classic (`gazebo_ros2_control/GazeboSystem`) on Humble or
Gazebo Sim (`gz_ros2_control/GazeboSimSystem`, world `worlds/obstacle_gz.sdf`)
on Jazzy, inserts Optimus, and loads the wheel controllers. In a second terminal, source the workspace and run:

```bash
ros2 launch optimus_bot teleop.launch.py
```

Use the displayed `teleop_twist_keyboard` keys to publish `/cmd_vel`. Gazebo is useful for physics and integration testing, but the wheel separation and physical sensor/motor calibrations still require real-robot validation.

The simulated 2-D LiDAR publishes `sensor_msgs/LaserScan` on `/scan`
with `lidar_link` as its frame at 10 Hz (bridged by `ros_gz_bridge` on Gazebo Sim). It is separate from
`lidar.launch.py`, which is only for the physical RPLIDAR.

The default `obstacle.world` has static objects, so `/scan` includes finite
ranges immediately. For command-line inspection, use sensor-data QoS:

```bash
ros2 topic echo /scan --qos-reliability best_effort
```

Gazebo draws the scan fan when its GUI is enabled. The small red marker on the
lidar body points along `lidar_link` +X, which is both the robot-forward and
the `/scan` zero-angle direction. In RViz, use a `LaserScan` display with
`/scan`, **Best Effort** reliability, and a fixed frame of `odom`,
`base_link`, or `lidar_link`.

## Remaining calibration before autonomous navigation

1. ~~Measure wheel-center separation~~ - done (0.265 m, verified).
2. Verify left/right motor direction against the ROS forward-motion convention: `left_motor_inverted` / `right_motor_inverted` in `description/control/real_hardware.xacro` (currently left `1`, right `0`).
3. Encoder counts per wheel revolution: `encoder_cpr` 16427, `encoder_cpr_verified: true` in `real_hardware.xacro` (see `zlac_drive/README.md`).
4. Measure the LiDAR mounting transform (`description/sensors/lidar.xacro`, currently estimates x 0.24 m, z 0.34 m).
5. Validate `/odom`, `odom -> base_link`, and `/scan` together in RViz before running SLAM.
6. Run SLAM, save a map, then validate AMCL and Nav2 end to end.
