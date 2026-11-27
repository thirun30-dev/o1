# Optimus AMR

**A low-cost ROS 2 autonomous mobile robot for warehouse transport: SLAM mapping, Nav2 navigation, a custom ZLAC8015D motor driver, a React web dashboard, and offline voice control.**

![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-22314E?logo=ros)
![C++](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus)
![Python](https://img.shields.io/badge/Python-3-3776AB?logo=python&logoColor=white)
![React](https://img.shields.io/badge/React-18-61DAFB?logo=react&logoColor=black)
![Vite](https://img.shields.io/badge/Vite-5-646CFF?logo=vite&logoColor=white)
![License](https://img.shields.io/badge/License-Apache%202.0-blue)

> Formerly known as **BumbleB**.

---

## Overview

Optimus is a differential-drive AMR built to carry a storage container between named warehouse locations (shelves, stations). An operator maps the warehouse with SLAM, saves locations by name, and then dispatches the robot from a browser, by clicking on the map, picking a saved location, or saying *"Optimus, go to shelf 1"*.

The project covers the full stack:

- **Hardware driver**: a from-scratch ros2_control plugin that speaks Modbus RTU over RS-485 to the ZLAC8015D dual-motor driver
- **Robot software**: kinematics, odometry, TF, a system state machine, process supervision, map and location management
- **Autonomy**: SLAM Toolbox for mapping, AMCL for localization, Nav2 for path planning and obstacle avoidance
- **Operator interface**: a React dashboard over rosbridge WebSockets, plus an offline voice assistant
- **Simulation**: Gazebo Classic and a Gazebo Sim warehouse world with obstacle and traffic tests

## Features

| Area | What it does |
|---|---|
| Mapping | Start and stop SLAM Toolbox from the dashboard, with pre-flight checks on `/scan`, `/odom` and TF, then save maps without a terminal |
| Localization | Map Server + AMCL, set the initial pose from the dashboard |
| Navigation | Nav2 click-to-navigate, or go to saved named locations |
| State machine | `OFF → BOOTING → READY → MAPPING / LOCALIZATION / NAVIGATION → ERROR / EMERGENCY_STOP`, with validated transitions |
| Safety | Hardware stop button (cuts motor power), software E-stop, 0.5 s `cmd_vel` timeout, velocity limits clamped to the motor's 350 RPM peak |
| Teleoperation | On-screen joystick with an inching mode for precise docking |
| Voice | Offline wake-word commands (Vosk, Indian-English model) and spoken replies (Piper / espeak) |
| Diagnostics | Live sensor and robot health, dashboard self-test, step-by-step SOP guide |
| Remote access | wss:// support, so the dashboard also works from an https deployment (such as Vercel) |

---

## System architecture

```
┌──────────────────────────────────────────────┐
│        Operator Web Dashboard (React)        │
│   map · teleop · modes · locations · voice   │
└──────────────────────┬───────────────────────┘
                       │ WebSocket  ws:// or wss://  :9090
┌──────────────────────▼───────────────────────┐
│           rosbridge_server + rosapi          │
└──────────────────────┬───────────────────────┘
                       │ ROS 2 Humble / Jazzy
      ┌────────────────┼──────────────────┬──────────────────┐
      ▼                ▼                  ▼                  ▼
┌─────────────┐ ┌──────────────┐ ┌────────────────┐ ┌───────────────┐
│   System    │ │   Location   │ │ optimus_voice  │ │ Drivetrain    │
│   Manager   │ │   Manager    │ │ (Vosk + Piper) │ │ node (C++)    │
│ FSM, health,│ │ named poses, │ └────────────────┘ │ kinematics,   │
│ processes   │ │ Nav2 goals   │                    │ odometry, TF  │
└──────┬──────┘ └──────────────┘                    └───────┬───────┘
       │ starts / stops                                     │
       ▼                                                    ▼
┌──────────────────────────────┐            ┌──────────────────────────────┐
│ SLAM Toolbox │ AMCL │ Nav2   │            │ ros2_control + zlac_drive    │
└──────────────────────────────┘            │ Modbus RTU over RS-485       │
                                            └──────────────┬───────────────┘
                                                           ▼
                                      ZLAC8015D driver → 2 × ZLTECH hub motors
                                      (or Gazebo in simulation)
```

---

## Hardware

| Item | Detail |
|---|---|
| Body | 600 × 400 × 300 mm, 4 corner casters, storage container on the top deck |
| Drive | 2 × ZLTECH ZLLG50ASM200 V2.0 hub motors (131 mm wheels, 265 mm apart) |
| Motor driver | ZLAC8015D, RS-485 via a USB adapter at 115200 baud |
| LiDAR | RPLidar A1M8, mounted on top at the front, full 360° scan |
| Odometry | Wheel-speed feedback from the ZLAC8015D |
| Battery | 25.6 V nominal, 15 Ah, 20 A BMS |
| Computer | Ubuntu 22.04 laptop (ROS 2 Humble) or Raspberry Pi 4/5 |
| Speed limits | 0.5 m/s linear, 1.0 rad/s angular (software) |

**Planned:** an ESP32-S2 sensor hub (IMU, rear ToF sensors, battery monitor, E-stop sense) and a depth camera. Until the battery monitor is fitted, the battery level on the dashboard is **simulated**.

---

## Repository structure

```text
.
├── optimus_ws/                     # ROS 2 workspace
│   ├── src/
│   │   ├── optimus_bot/            # URDF, launch, config, drivetrain node, managers
│   │   ├── zlac_drive/             # ZLAC8015D Modbus ros2_control hardware plugin
│   │   ├── optimus_voice/          # Voice commands and text-to-speech
│   │   └── optimus_sim/            # Gazebo Sim warehouse and obstacle tests (Jazzy only)
│   ├── maps/                       # Sample warehouse map
│   ├── locations/                  # Sample saved locations
│   ├── tools/                      # Hardware test scripts, systemd service, TLS cert script
│   └── docs/                       # Full documentation
├── web_dashboard_src/
│   └── web_dashboard/              # React 18 + Vite + Tailwind operator dashboard
└── optimus_layout_patches/         # Patches for the LiDAR-on-top body layout
```

### ROS 2 packages

| Package | Language | Purpose |
|---|---|---|
| `optimus_bot` | C++ / Python | Robot description, drivetrain kinematics and odometry, system manager (state machine), location manager, scan filter, launch files, Nav2/SLAM/AMCL configs, custom services (`SetMode`, `MapOperation`, `NavigateToPose`, `SetInitialPose`) |
| `zlac_drive` | C++ | ros2_control `SystemInterface` for the ZLAC8015D: termios serial, Modbus RTU with CRC-16, register map, diagnostics, gtest suite |
| `optimus_voice` | Python | Vosk speech recognition, intent parser, Piper/espeak TTS, waypoint commands |
| `optimus_sim` | Python | Gazebo Sim warehouse world, Nav2 parameters, obstacle spawning and traffic tests |

---

## Getting started

### Prerequisites

- Ubuntu 22.04 + ROS 2 **Humble** (on the robot), or Ubuntu 24.04 + ROS 2 **Jazzy**
- Node.js 18+ and npm (for the dashboard)
- A USB-to-RS485 adapter and an RPLidar A1M8 (for the physical robot)

### 1. Build the workspace

```bash
git clone https://github.com/thirun30-dev/o1.git
cd o1/optimus_ws
source /opt/ros/$ROS_DISTRO/setup.bash

# optimus_sim needs Gazebo Sim, which is Jazzy-only
[ "$ROS_DISTRO" = humble ] && touch src/optimus_sim/COLCON_IGNORE

rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

Saved maps and locations are kept in `~/optimus_data/` so they survive rebuilds:

```bash
mkdir -p ~/optimus_data/{maps,locations}
```

### 2. Launch the robot

```bash
# Physical robot: drivetrain, robot state publisher, LiDAR, rosbridge, managers
ros2 launch optimus_bot bringup.launch.py

# With voice commands
ros2 launch optimus_bot bringup.launch.py use_voice:=true

# Simulated robot
ros2 launch optimus_bot bringup.launch.py simulation:=true use_sim_time:=true
```

You don't need separate terminals for SLAM, localization or Nav2: the dashboard starts and stops them through the system manager.

### 3. Run the web dashboard

```bash
cd web_dashboard_src/web_dashboard
npm install
npm run dev
```

Open `http://localhost:3000`. If the robot is a different machine, add `?ros_host=<robot-ip>`. You can also set a default in `.env`:

```bash
VITE_ROSBRIDGE_URL=ws://<robot-ip>:9090
```

Then open **Diagnostics → RUN SELF-TEST**, and follow the **SOP GUIDE**.

### 4. Using the dashboard over https (optional)

Browsers block `ws://` from an `https://` page. To use a hosted dashboard (for example on Vercel), serve rosbridge over TLS:

```bash
./optimus_ws/tools/make_rosbridge_cert.sh            # creates a self-signed certificate
ros2 launch optimus_bot bringup.launch.py rosbridge_ssl:=true
```

Visit `https://<robot-ip>:9090` once in the browser and accept the certificate warning. After that, wss:// connections work.

---

## Typical workflow

1. **Map**: switch to *Mapping*, drive the robot around with the joystick, then save the map.
2. **Localize**: switch to *Localization*, load the map, and set the initial pose.
3. **Save locations**: drive to each shelf or station and save it by name.
4. **Navigate**: click on the map, pick a saved location, or say *"Optimus, go to shelf 1"*.
5. **Stop**: use the dashboard E-STOP or the hardware stop button at any time.

---

## Simulation

```bash
# Robot in the Gazebo obstacle world
ros2 launch optimus_bot simulation.launch.py

# Warehouse world (Jazzy + Gazebo Sim)
ros2 launch optimus_sim warehouse.launch.py
```

See [Simulation Guide](optimus_ws/docs/SIMULATION.md) for the obstacle and traffic tests.

---

## Testing

```bash
cd optimus_ws
colcon build --symlink-install            # builds with tests enabled
colcon test && colcon test-result --verbose
```

The tests include:

- **zlac_drive**: gtest suites for the Modbus client, driver, motor model and hardware interface
- **optimus_bot**: URDF and launch-file checks
- **optimus_voice**: intent-parser tests
- **optimus_sim**: package file checks

Manual hardware test plans for motors, LiDAR, SLAM, navigation and odometry calibration are in [`optimus_ws/docs/testing/`](optimus_ws/docs/testing/).

---

## Documentation

| Guide | Topic |
|---|---|
| [Architecture](optimus_ws/docs/ARCHITECTURE.md) | Components, topics, services, actions, TF tree |
| [Setup](optimus_ws/docs/SETUP.md) | Platforms, dependencies, build and tests |
| [Raspberry Pi](optimus_ws/docs/RASPBERRY_PI.md) | Wiring, Pi setup, first power-up, autostart |
| [Simulation](optimus_ws/docs/SIMULATION.md) | Gazebo Classic / Gazebo Sim |
| [Mapping](optimus_ws/docs/MAPPING.md) | SLAM Toolbox, pre-flight checks, saving maps |
| [Localization](optimus_ws/docs/LOCALIZATION.md) | Map Server, AMCL, initial pose |
| [Navigation](optimus_ws/docs/NAVIGATION.md) | Nav2, costmaps, planner, click-to-nav |
| [Web Dashboard](optimus_ws/docs/WEB_DASHBOARD.md) | Pages, voice, teleop, code structure |
| [Hardware Migration](optimus_ws/docs/HARDWARE_MIGRATION.md) | Moving from simulation to the physical robot |
| [Real Robot Operation](optimus_ws/docs/REAL_ROBOT_OPERATION.md) | Bringup, operations, service calls |
| [RS-485 Wiring](optimus_ws/docs/hardware/rs485_wiring.md) | Motor driver connection and troubleshooting |
| [ZLAC8015D Protocol](optimus_ws/docs/hardware/zlac8015d_protocol.md) | Modbus registers and commands |

Each package also has its own README under `optimus_ws/src/`.

---

## Roadmap

- [ ] ESP32-S2 sensor hub: IMU, rear ToF sensors, real battery monitor, E-stop sense
- [ ] Depth camera for 3D obstacle detection
- [ ] IMU + wheel odometry fusion (robot_localization EKF)
- [ ] `twist_mux` so the software E-stop locks out every velocity source
- [ ] Authentication on rosbridge
- [ ] Multi-robot fleet dispatching

---

## Author

**Thirunavukkarasu Veeramani**, B.E. Robotics and Automation, Rajalakshmi Engineering College, Chennai
GitHub: [@thirun30-dev](https://github.com/thirun30-dev)

## License

Licensed under the [Apache License 2.0](optimus_ws/src/optimus_bot/LICENSE).
