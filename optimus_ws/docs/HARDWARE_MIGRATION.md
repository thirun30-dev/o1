# Gazebo to Physical AMR Hardware Migration Guide

## Overview

The Optimus AMR software stack is designed following strict separation of concerns. The navigation, localization, location management, and web dashboard layers consume only standard ROS 2 interfaces and are **completely decoupled from Gazebo**.

---

## 1. Interface Boundary

```text
+--------------------------------------------------------------+
|     Navigation, Localization, System Manager, Dashboard      |
+------------------------------+-------------------------------+
                               | Standard ROS 2 Topics & TF
                               | (/cmd_vel, /odom, /scan, TF)
+------------------------------+-------------------------------+
|      Simulation Layer        |       Physical Robot Layer    |
|   (simulation.launch.py)     |      (real_robot.launch.py)   |
+------------------------------+-------------------------------+
|  Gazebo Classic (Humble) or  |  Physical Chassis             |
|  Gazebo Sim (Jazzy)          |  zlac_drive (ZLAC8015D Modbus)|
|  gazebo/gz_ros2_control      |  Physical RPLIDAR A1M8        |
|  Simulated 2D LiDAR          |  ZLAC8015D wheel feedback     |
+------------------------------+-------------------------------+
```

---

## 2. Transition Steps

To deploy on physical hardware:

1. **Wheel geometry** (done): `wheel_radius: 0.0655` and
   `wheel_separation: 0.265` in `optimus_bot/config/drivetrain.yaml`
   (`wheel_separation_verified: true`). Re-measure after any chassis change.
2. **Motor directions**: positive `/cmd_vel` must roll both wheels forward
   (+X forward, +Y left). The mapping is `left_motor_inverted` /
   `right_motor_inverted` in
   `optimus_bot/description/control/real_hardware.xacro` (currently left `1`,
   right `0`). Check with the wheels off the ground (see
   [Running Optimus on a Raspberry Pi](RASPBERRY_PI.md#4-first-power-up-wheels-off-the-ground)).
3. **LiDAR mounting offset**: `lidar_x` / `lidar_z` in
   `optimus_bot/description/sensors/lidar.xacro` (currently 0.24 m forward,
   0.34 m above the wheel axle - estimates, measure them).
4. **Serial devices**: confirm the `by-id` paths in `real_hardware.xacro` and
   `config/lidar.yaml` (see [RS-485 wiring](hardware/rs485_wiring.md)).
5. **Launch**:
   ```bash
   ros2 launch optimus_bot bringup.launch.py simulation:=false use_sim_time:=false
   ```
6. **Web Dashboard**: the dashboard connects to the physical robot's
   rosbridge without code changes. Open
   `http://<dashboard-host>:3000/?ros_host=<robot-ip>&ros_port=9090`.
