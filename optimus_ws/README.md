# Optimus AMR — Autonomous Warehouse Navigation & Web Control Dashboard

*(formerly BumbleB)*

An industrial Autonomous Mobile Robot (AMR) system for warehouse logistics built on **ROS 2** (Humble on the robot, Jazzy also supported) with a **React web dashboard** and optional **voice control**.

---

## System Overview

```
                        +------------------------------------+
                        |       Operator Web Dashboard       |
                        |      (React 18 + Vite + roslib)    |
                        |  http://<host>:3000 (Ctrl+, voice) |
                        +-----------------+------------------+
                                          |
                               WebSocket (ws://<robot>:9090)  
                                          |
                        +-----------------v------------------+
                        |          rosbridge_server          |
                        |      + rosapi (from apt/rosdep)    |
                        +-----------------+------------------+
                                          |
                                 ROS 2 Humble / Jazzy
                                          |
     +------------------------------------+------------------------------------+
     |                                    |                                    |
+----v--------------------+      +--------v---------------+          +---------v----------+
|   amr_system_manager    |      |  amr_location_manager  |          |     optimus_bot    |
| (FSM, Health, Processes)|      | (Locations & Nav Goals)|          | (Kinematics, TF,   |
+----+--------------------+      +--------+---------------+          |  URDF, Simulation) |
     |                                    |                          +---------+----------+
     |                                    |                                    |
     +-----------------+------------------+                                    |
                       |                                                       |
        +--------------+--------------+                                        |
        |                             |                                        |
+-------v---------+           +-------v---------+                              |
|  SLAM Toolbox   |           |  Map Server &   |                              |
| (Mapping Mode)  |           | AMCL + Nav2     |                              |
+-----------------+           +-----------------+                              |
                                                                               |
               +---------------------------------------------------------------+
               |
        +------v------------------------------+
        |  Simulation (Gazebo Classic / Sim)  |
        |  or Physical Robot (ZLAC8015D)      |
        +-------------------------------------+
```

---

## Robot Hardware

| Item | Detail |
|---|---|
| Body | 600 × 400 × 300 mm box, 4 corner casters, 2 ZLTECH hub motors in the middle, storage container on the top deck |
| Drive | 2 × ZLTECH ZLLG50ASM200 V2.0 hub motors (131 mm wheels, 265 mm apart), ZLAC8015D driver, RS485 (USB adapter, 115200 baud) → robot computer |
| LiDAR | RPLidar A1M8 on top of the body at the front end, full 360° scan (container sits below the scan plane) |
| Odometry | ZLAC8015D wheel speed feedback, integrated by `optimus_drivetrain_node` |
| Battery | 25.6 V nominal, 15 Ah, BMS 20 A |
| Safety | Hardware stop button (cuts motor power); software E-STOP and 0.5 s command timeout |
| Computer | Robot laptop (Ubuntu 22.04 + ROS 2 Humble) or Raspberry Pi 4/5 - see [Raspberry Pi guide](docs/RASPBERRY_PI.md) |
| Limits | 0.5 m/s, 1.0 rad/s (software); 350 RPM motor ceiling |

**LiDAR field of view:** `/scan` carries the full 360° by default. If anything on the top deck ever blocks the rear, crop it with `lidar_fov_deg` (e.g. `ros2 launch optimus_bot lidar.launch.py lidar_fov_deg:=270`) or the `field_of_view_deg` parameter of `front_scan_filter`. Measure and update `lidar_x` / `lidar_z` in `description/sensors/lidar.xacro` and the container size in `description/base/base.xacro` — current values are estimates.

**Planned upgrades:** ESP32-S2 sensor hub (IMU, rear ToF sensors, battery monitor, E-stop sense) and a depth camera. Until the battery monitor is added, the battery shown on the dashboard is simulated.

---

## Workspace Structure

```text
~/optimus_ws/
├── src/
│   ├── optimus_bot/             # Robot package: URDF, launch, config, managers, drivetrain node
│   │   ├── config/              # drivetrain.yaml, controllers.yaml, nav2.yaml, slam.yaml, localization.yaml, lidar.yaml
│   │   ├── description/         # URDF/Xacro: body, wheels, LiDAR, real + simulated ros2_control
│   │   ├── launch/              # bringup, real_robot, lidar, simulation, slam/mapping, localization, navigation, rosbridge
│   │   ├── scripts/             # system_manager_node.py, location_manager_node.py, front_scan_filter.py
│   │   ├── src/                 # drivetrain_node.cpp (kinematics, odometry, TF)
│   │   └── worlds/, tools/      # simulation worlds and the Classic -> Gazebo Sim world converter
│   ├── zlac_drive/              # ZLAC8015D Modbus ros2_control hardware driver
│   ├── optimus_voice/           # Voice commands ("Optimus, go to ...")
│   └── optimus_sim/             # Gazebo Sim warehouse + obstacle tests (Jazzy only)
├── tools/                       # Hardware test scripts, systemd service
└── docs/                        # Documentation

~/optimus_data/                  # Created at runtime, survives rebuilds
├── maps/                        # Saved maps (.yaml, .pgm, .posegraph, .data)
└── locations/                   # warehouse_locations.yaml
```

The web dashboard lives outside the workspace in its own `web_dashboard/` folder.

---

## Quick Start

### 0. Migrating from BumbleB

The package is now `optimus_bot` and the workspace `optimus_ws`. On the robot computer:

```bash
mv ~/bumbleb_ws ~/optimus_ws && cd ~/optimus_ws && rm -rf build install log
mv ~/bumbleb_data ~/optimus_data 2>/dev/null || mkdir -p ~/optimus_data/{maps,locations}
sudo systemctl disable --now bumbleb-bringup 2>/dev/null   # old service, if installed
```

### 1. Install dependencies and build

```bash
cd ~/optimus_ws
source /opt/ros/$ROS_DISTRO/setup.bash
[ "$ROS_DISTRO" = humble ] && touch src/optimus_sim/COLCON_IGNORE   # Gazebo Sim package is Jazzy-only
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

### 2. Launch the Physical Robot System

Start the ZLAC drivetrain, robot-state publisher, RPLIDAR, rosbridge, system manager and location manager with one command:

```bash
ros2 launch optimus_bot bringup.launch.py                 # add use_voice:=true for voice commands
```

The default is `simulation:=false` and `use_sim_time:=false`. Dashboard buttons start and stop SLAM, localization and Nav2 through the backend. For the simulated robot use `simulation:=true use_sim_time:=true`.

### 3. Launch the Web Dashboard

```bash
cd web_dashboard
npm install        # first time
npm run dev
```

Open `http://localhost:3000` (add `?ros_host=<robot-ip>` when the robot is another machine). Check **Diagnostics → RUN SELF-TEST**, then follow the **SOP GUIDE**. Press **Ctrl + ,** for voice commands.

---

## Documentation Index

Detailed guides are available in the [`docs/`](docs/) directory:

- [Architecture Guide](docs/ARCHITECTURE.md) — System components, ROS 2 topics, services, actions, and TF tree.
- [Setup & Installation](docs/SETUP.md) — Supported platforms, dependencies, build and tests.
- [Raspberry Pi Guide](docs/RASPBERRY_PI.md) — Wiring, Pi setup, first power-up, autostart.
- [Simulation Guide](docs/SIMULATION.md) — Robot simulation (Gazebo Classic / Gazebo Sim) and the warehouse simulation.
- [Mapping Guide](docs/MAPPING.md) — SLAM Toolbox mapping, pre-flight checks, and map saving.
- [Localization Guide](docs/LOCALIZATION.md) — Map Server, AMCL, and pose initialization.
- [Navigation Guide](docs/NAVIGATION.md) — Nav2 stack, costmaps, planner, and click-to-nav.
- [Web Dashboard Guide](docs/WEB_DASHBOARD.md) — Pages, voice commands (Ctrl + ,), teleoperation, and code structure.
- [Hardware Migration Guide](docs/HARDWARE_MIGRATION.md) — Transferring from Gazebo simulation to physical AMR.
- [Physical Robot Operation](docs/REAL_ROBOT_OPERATION.md) — Real-robot bringup, dashboard operations, and ROS/rosbridge service requests.
- [RS-485 Wiring](docs/hardware/rs485_wiring.md) — Motor driver connection and serial troubleshooting.
- Package READMEs: `src/optimus_bot`, `src/zlac_drive`, `src/optimus_voice`, `src/optimus_sim`.
