# Simulation Guide

Optimus has two simulations:

| | Robot simulation (`optimus_bot`) | Warehouse simulation (`optimus_sim`) |
|---|---|---|
| Purpose | Run the **same stack as the real robot** (ros2_control, drivetrain node, managers, dashboard) without hardware | Self-contained SLAM + Nav2 demo and obstacle-response tests in a 24 × 16 m warehouse, with the real robot's geometry |
| Simulator | Gazebo Classic (Humble) or Gazebo Sim / Harmonic (Jazzy) - chosen automatically | Gazebo Sim / Harmonic (Jazzy only) |
| Start | `ros2 launch optimus_bot simulation.launch.py` | `ros2 launch optimus_sim warehouse.launch.py` |

---

## 1. Robot simulation architecture

```text
simulation.launch.py   (sim_backend: auto | classic | gz)
  ├── World: worlds/obstacle.world (Classic) or worlds/obstacle_gz.sdf (Gazebo Sim)
  ├── Model: optimus (robot_description from optimus.urdf.xacro use_sim:=true)
  ├── ros2_control inside the simulator:
  │     gazebo_ros2_control/GazeboSystem (Classic) or gz_ros2_control/GazeboSimSystem (Gazebo Sim)
  │     ├── joint_state_broadcaster
  │     └── optimus_wheel_velocity_controller (forward_command_controller)
  ├── optimus_drivetrain_node  (/cmd_vel -> wheel commands, /joint_states -> /odom + TF)
  ├── LiDAR: 360 rays over 360°, 10 Hz, 0.12-12 m -> /scan (frame lidar_link)
  │     (Gazebo Sim: bridged by ros_gz_bridge together with /clock)
  └── RViz (rviz:=true)
```

`worlds/obstacle_gz.sdf` is generated from `obstacle.world` by
`tools/classic_world_to_gz.py` - edit the Classic world and re-run the script.

---

## 2. Drivetrain single source of truth

The wheel geometry is configured in **only one place**:

```yaml
# optimus_bot/config/drivetrain.yaml
optimus_drivetrain_node:
  ros__parameters:
    wheel_radius: 0.0655        # 131 mm ZLLG50ASM200 V2.0 hub wheel
    wheel_separation: 0.265     # wheel-centre to wheel-centre (verified)
    max_linear_velocity: 0.5    # m/s
    max_angular_velocity: 1.0   # rad/s
    cmd_vel_timeout: 0.5        # watchdog timeout (seconds)
    control_frequency: 50.0     # Hz
```

The launch files read this YAML and pass `wheel_radius` and `wheel_separation`
into the URDF/Xacro, so the simulated wheels and the odometry always agree.

---

## 3. Starting the robot simulation

Standalone (Gazebo window + RViz):

```bash
ros2 launch optimus_bot simulation.launch.py
ros2 launch optimus_bot simulation.launch.py headless:=true rviz:=false   # Gazebo Sim without GUI
ros2 launch optimus_bot simulation.launch.py sim_backend:=gz              # force a backend
```

With the complete system (managers, rosbridge, dashboard workflow):

```bash
ros2 launch optimus_bot bringup.launch.py simulation:=true use_sim_time:=true
```

Always pass `use_sim_time:=true` with `simulation:=true`, otherwise the
managers run on wall time while the simulator publishes simulated time.

---

## 4. Verifying the simulation

```bash
ros2 control list_controllers                       # both controllers active
ros2 topic echo --once --full-length /scan --field header.frame_id   # lidar_link
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
ros2 topic echo /odom --field pose.pose.position    # x increases
ros2 run tf2_ros tf2_echo odom base_link
```

`/odom` is integrated from wheel speeds, so it reports motion even if the
simulated robot slips. To see the true pose in Gazebo Sim:

```bash
gz model -m optimus -p
```

`ros2 topic echo` truncates long arrays to 128 items; use `--full-length`
when checking scan ray counts.

---

## 5. Warehouse simulation (optimus_sim)

```bash
ros2 launch optimus_sim warehouse.launch.py                 # Gazebo + SLAM + Nav2 + RViz + moving trolley
ros2 launch optimus_sim warehouse.launch.py headless:=true  # no Gazebo GUI
ros2 launch optimus_sim warehouse.launch.py voice:=true     # + voice control
ros2 run optimus_sim obstacle_test --list                   # obstacle-response scenarios
ros2 run optimus_sim spawn_obstacle --ahead 1.5 --kind person
```

SLAM starts with an almost empty map, and Nav2 cannot plan to points outside
the mapped area. One slow turn in place at the start is enough to seed the map;
avoid repeated fast spins before the map exists, as they can leave the whole
map rotated against the real walls. Drive or send goals in steps toward unexplored areas; the map
grows as the robot moves. See `src/optimus_sim/README.md` for the world layout
and test scenarios.
