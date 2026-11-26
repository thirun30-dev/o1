# Setup and Installation Guide

## 1. Supported platforms

| | Robot (current) | Development / simulation |
|---|---|---|
| OS | Ubuntu 22.04 LTS | Ubuntu 24.04 LTS |
| ROS 2 | Humble (`/opt/ros/humble`) | Jazzy (`/opt/ros/jazzy`) |
| Simulator | Gazebo Classic 11 (`gazebo_ros2_control`) | Gazebo Sim / Harmonic (`gz_ros2_control`) |
| Computer | Robot laptop or Raspberry Pi 4 | Any x86 / arm64 PC; Raspberry Pi 5 |

The launch files detect the ROS distribution (`ROS_DISTRO`) and pick the right
SLAM Toolbox start-up, Nav2 plugin names and simulator automatically.

Also needed:

- **Node.js** 18 or newer (with npm) for the web dashboard.
- **Python** 3.10 or newer (ships with both Ubuntu versions).

For a Raspberry Pi, follow [Running Optimus on a Raspberry Pi](RASPBERRY_PI.md).

---

## 2. Workspace contents

```bash
cd ~/optimus_ws
colcon list
# optimus_bot     robot integration: URDF, launch, config, managers, drivetrain node
# optimus_sim     Gazebo Sim warehouse + obstacle tests (Jazzy only)
# optimus_voice   wake-word voice commands ("Optimus, go to ...")
# zlac_drive      ZLAC8015D Modbus ros2_control hardware driver
```

On **Humble**, skip the Jazzy-only simulator package:

```bash
touch src/optimus_sim/COLCON_IGNORE
```

---

## 3. Dependencies

All ROS dependencies are declared in the `package.xml` files (Gazebo packages
are chosen per distribution), so `rosdep` installs them:

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init        # only once per machine
rosdep update

cd ~/optimus_ws
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -y
```

This installs ros2_control and the controllers, xacro, robot_state_publisher,
`rplidar_ros`, SLAM Toolbox, Nav2, `rosbridge_server` / `rosapi`, the simulator
packages and the teleop tools. rosbridge comes from apt; a separate rosbridge
underlay workspace is no longer required.

Optional extras:

```bash
pip install "pymodbus<3"                          # hardware test scripts in tools/
sudo apt install -y sox espeak-ng pulseaudio-utils # voice replies (optimus_voice)
pip install vosk sounddevice                       # offline microphone recognition (optimus_voice)
```

---

## 4. Build

```bash
cd ~/optimus_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

On a 4 GB Raspberry Pi add `--parallel-workers 2`. To source the workspace in
every new terminal:

```bash
echo "source ~/optimus_ws/install/setup.bash" >> ~/.bashrc
```

### Tests

```bash
colcon build --cmake-args -DBUILD_TESTING=ON
colcon test && colcon test-result --verbose
```

This runs the C++ driver tests, the Python URDF/launch tests, the voice parser
tests, the simulator file checks and the ament lint checks.

---

## 5. Run

| Goal | Command | Guide |
|---|---|---|
| Physical robot | `ros2 launch optimus_bot bringup.launch.py` | [Physical Robot Operation](REAL_ROBOT_OPERATION.md) |
| Robot simulation (same stack as the robot) | `ros2 launch optimus_bot bringup.launch.py simulation:=true use_sim_time:=true` | [Simulation Guide](SIMULATION.md) |
| Warehouse simulation (SLAM + Nav2 demo, Jazzy) | `ros2 launch optimus_sim warehouse.launch.py` | `src/optimus_sim/README.md` |

`bringup.launch.py` defaults to `simulation:=false` and `use_sim_time:=false`.
The dashboard then starts SLAM, AMCL localization and Nav2 through the system
manager. Add `use_voice:=true` to start the voice command node.

---

## 6. Web dashboard

```bash
cd web_dashboard
npm install
npm run dev        # development server on http://<host>:3000
npm run build      # production build in dist/
```

If the dashboard is opened on a different machine from the robot, point it at
the robot's rosbridge:

```text
http://<dashboard-host>:3000/?ros_host=<robot-ip>&ros_port=9090
```

See the [Web Dashboard Guide](WEB_DASHBOARD.md).
