# Optimus Physical-Robot Operation

This is the normal operating procedure for the real robot. Do not use Gazebo values or simulation time on the physical machine.

The robot currently runs Ubuntu 22.04 + ROS 2 Humble; Ubuntu 24.04 + Jazzy is also supported.

## 1. Build and source

```bash
cd ~/optimus_ws
source /opt/ros/$ROS_DISTRO/setup.bash      # humble on the robot today
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

On a Raspberry Pi, follow [Running Optimus on a Raspberry Pi](RASPBERRY_PI.md)
for OS, dependencies and the first power-up.

### Serial devices

Both USB devices are addressed by their stable `/dev/serial/by-id/` names, so
re-plugging does not change them:

| Device | Path | Configured in |
|---|---|---|
| ZLAC8015D (USB-RS485, CH340) | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C65150306-if00` | `description/control/real_hardware.xacro` (`serial_port`, 115200 baud, slave ID 1) |
| RPLIDAR A1M8 (CP2102) | `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` | `config/lidar.yaml` (falls back to `/dev/ttyUSB0` if missing) |

Check them with `ls -l /dev/serial/by-id/` before powering the robot. A
different USB-RS485 adapter has a different `by-id` name - update the xacro and
rebuild. Your user must be in the `dialout` group.

## 2. Start the unified physical bringup

```bash
ros2 launch optimus_bot bringup.launch.py                 # add use_voice:=true for voice commands
```

`bringup.launch.py` defaults to `simulation:=false` and `use_sim_time:=false`. It starts the hardware and always-on services:

| Component | Purpose |
|---|---|
| `real_robot.launch.py` | Robot description, fixed TF, ZLAC ros2_control, wheel controllers, drivetrain odometry |
| `lidar.launch.py` | RPLIDAR `/scan_raw` and filtered `/scan` (360°, capped at 720 rays) |
| `rosbridge.launch.py` | WebSocket ROS bridge on port `9090` (+ `rosapi`) |
| `system_manager_node.py` | Safety FSM, health, process supervision, map operations |
| `location_manager_node.py` | Robot pose, locations, AMCL initial pose, Nav2 goals |
| `voice.launch.py` (`use_voice:=true`) | Voice command node for the dashboard voice panel |

The backend owns the fixed frame contract:

```text
map -> odom -> base_link -> lidar_link
```

Quick hardware check after start-up:

```bash
ros2 control list_controllers                # both controllers active
ros2 topic list | grep commands              # /optimus_wheel_velocity_controller/commands
ros2 topic hz /scan                          # ~10 Hz
ros2 topic echo /amr/system_health --once    # "overall": "OK"
```

Saved data lives outside the workspace so rebuilds never touch it:

- maps: `~/optimus_data/maps/<name>.yaml|.pgm|.posegraph|.data`
- locations: `~/optimus_data/locations/warehouse_locations.yaml`

## 3. Start the dashboard

```bash
cd web_dashboard
npm install          # first time
npm run dev          # port 3000
```

Open `http://<dashboard-computer>:3000`. If the dashboard computer is different from the robot, pass the robot host and port in the URL:

```text
http://<dashboard-computer>:3000/?ros_host=<robot-ip>&ros_port=9090
```

Open **Diagnostics** and press **RUN SELF-TEST** before operating: it checks
the connection, LiDAR, drive, odometry, TF and that all core nodes are running.
The **SOP GUIDE** button in the header walks through the full procedure.

## 4. Mapping procedure

1. Check hardware topics and TF:

   ```bash
   ros2 topic echo /amr/system_health --once
   ros2 topic echo /odom --once
   ros2 topic echo /scan --once
   ros2 run tf2_ros tf2_echo odom base_link
   ```

2. Press **Start mapping**. This calls `/amr/set_mode` with `MAPPING`; the manager launches `mapping.launch.py`, which launches `slam.launch.py` and `async_slam_toolbox_node`.
3. Drive the robot slowly. SLAM publishes `/map` and `map -> odom`.
4. Enter a name and press **Save occupancy map**. This calls `/amr/map_operation` with `save_map` and creates `<name>.yaml` and `<name>.pgm`.
5. Press **Serialize graph**. This calls SLAM Toolbox and creates `<name>.posegraph` and `<name>.data`.
6. Press **Stop** to return to `READY`.

## 5. Localization and autonomous navigation

1. Select the saved map and press **Start AMCL**. The manager stops SLAM, launches `localization.launch.py`, starts map server and AMCL, and prevents simultaneous `map -> odom` publishers.
2. Set the initial pose from the map view or pose fields. The dashboard calls `/amr/set_initial_pose`, publishing `/initialpose` in the configured map frame.
3. Wait for the health panel to report AMCL pose output.
4. Nav2 starts automatically once AMCL publishes a pose (or press **Start Nav2**). The manager launches `navigation.launch.py`: controller, smoother, planner, behavior server, BT navigator, waypoint follower and velocity smoother.
5. Click a free map location or enter coordinates, then press **Send pose goal**. The dashboard sends `{"action": "navigate_to_pose"}` on `/amr/command` (scripts can call the `/amr/navigate_to_pose` service); the location manager sends one `nav2_msgs/action/NavigateToPose` goal and reports progress on `/amr/navigation_status`. RViz "Nav2 Goal" clicks on `/goal_pose` go to Nav2's BT navigator directly.
6. Use **Cancel goal** to call `/amr/cancel_navigation`.

## 6. Direct ROS service requests

```bash
# Mapping (mapping.launch.py -> slam.launch.py -> async SLAM)
ros2 service call /amr/set_mode optimus_bot/srv/SetMode \
  "{mode: MAPPING, map_name: ''}"

# Save map and serialize pose graph
ros2 service call /amr/map_operation optimus_bot/srv/MapOperation \
  "{operation: save_map, map_name: warehouse_main}"
ros2 service call /amr/map_operation optimus_bot/srv/MapOperation \
  "{operation: serialize_pose_graph, map_name: warehouse_main}"

# Stop feature processes
ros2 service call /amr/set_mode optimus_bot/srv/SetMode \
  "{mode: READY, map_name: ''}"

# AMCL/map server, initial pose, Nav2, and a goal
ros2 service call /amr/set_mode optimus_bot/srv/SetMode \
  "{mode: LOCALIZATION, map_name: warehouse_main}"
ros2 service call /amr/set_initial_pose optimus_bot/srv/SetInitialPose \
  "{x: 0.0, y: 0.0, yaw: 0.0}"
ros2 service call /amr/set_mode optimus_bot/srv/SetMode \
  "{mode: NAVIGATION, map_name: warehouse_main}"
ros2 service call /amr/navigate_to_pose optimus_bot/srv/NavigateToPose \
  "{x: 2.0, y: 1.0, yaw: 1.57}"
ros2 service call /amr/cancel_navigation std_srvs/srv/Trigger "{}"
```

The dashboard uses the same typed services through rosbridge:

```text
optimus_bot/srv/SetMode
optimus_bot/srv/MapOperation
optimus_bot/srv/SetInitialPose
optimus_bot/srv/NavigateToPose
std_srvs/srv/Trigger
```

Status topics are `/amr/system_state`, `/amr/system_health`, `/amr/maps`, `/amr/map_operation_result`, `/amr/navigation_status`, `/amr/robot_pose`, and `/map`.

## 7. Safety and shutdown

- **Hardware stop button:** cuts motor power. It is the primary emergency stop -
  keep it within reach whenever the robot moves.
- **Dashboard / service E-STOP:** software only. The system manager latches
  `EMERGENCY_STOP`, cancels Nav2 goals and publishes zero `/cmd_vel` at 20 Hz
  until it is reset.
- **Command timeout:** `optimus_drivetrain_node` commands zero wheel speed if
  `/cmd_vel` stops for 0.5 s.

```bash
ros2 service call /amr/emergency_stop std_srvs/srv/Trigger "{}"
ros2 service call /amr/reset_emergency_stop std_srvs/srv/Trigger "{}"
```

Stop the unified bringup with `Ctrl+C`; do not manually kill SLAM, AMCL, or Nav2 while another feature is active.
