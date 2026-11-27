# Running Optimus on a Raspberry Pi

This guide runs the Optimus robot software on a Raspberry Pi mounted on the
robot, with the web dashboard opened from a laptop or tablet. The same steps
apply to any onboard Linux computer (the robot laptop works the same way).

> **Status:** the software builds and its tests pass on ROS 2 Jazzy (x86).
> Running on a Pi and on Humble has not been tested end to end yet, so treat
> the first run as a bring-up test and go through
> [First run on Humble](#8-first-run-checks-on-humble).

---

## 1. Hardware overview

```text
 25.6 V battery ─┬─► ZLAC8015D motor driver ──► 2 × ZLLG50ASM200 hub motors
 (15 Ah, BMS)    │         │ RS485 (A+/B-)
                 │   USB-RS485 adapter (CH340) ─┐
                 │                              ├── USB ──► Raspberry Pi
                 │   RPLIDAR A1M8 (CP2102 USB) ─┘              │ Wi-Fi
                 └─► DC-DC step-down → 5 V ──► Pi power        ▼
                                                    Laptop / tablet browser
                                                    (dashboard ↔ rosbridge :9090)
```

| Item | Recommendation |
|---|---|
| Computer | Raspberry Pi 4 (4 GB or more) or Raspberry Pi 5 |
| OS + ROS | **Pi 4:** Ubuntu 22.04 64-bit + ROS 2 **Humble** (what the robot service file expects). **Pi 5:** Ubuntu 24.04 + ROS 2 **Jazzy** (Ubuntu 22.04 does not support the Pi 5). The software supports both. |
| Pi power | A step-down converter to 5 V rated **3 A (Pi 4) / 5 A (Pi 5)**, with an input range covering the whole battery voltage (25.6 V nominal; choose one rated to about 30 V or more). Undervoltage causes random USB-serial drop-outs. |
| Motor bus | USB-RS485 adapter to the ZLAC8015D: A+ → A+, B- → B-. Add a 120 Ω termination resistor if you see Modbus CRC errors. |
| LiDAR | RPLIDAR A1M8 on USB (CP2102 adapter). |
| Safety | The robot's **hardware stop button cuts motor power** - keep it within reach for every test. The dashboard E-STOP is software only (zero velocity at 20 Hz). |

---

## 2. Prepare the Pi (once)

Install Ubuntu Server 64-bit and ROS 2 (`ros-<distro>-ros-base` is enough; no
desktop is needed on the robot), then:

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep git
sudo rosdep init; rosdep update
sudo usermod -aG dialout $USER        # serial-port access; log out and back in
```

Copy the source from your development machine. Do **not** copy `build/`,
`install/` or `log/` - they are compiled for that machine:

```bash
rsync -av --exclude build --exclude install --exclude log \
  ~/o1/optimus_ws/ <user>@<pi-ip>:~/optimus_ws/
```

Build on the Pi:

```bash
cd ~/optimus_ws
source /opt/ros/$ROS_DISTRO/setup.bash            # humble or jazzy
# optimus_sim is a Gazebo Sim (Jazzy) package - skip it on Humble:
[ "$ROS_DISTRO" = humble ] && touch src/optimus_sim/COLCON_IGNORE
rosdep install --from-paths src --ignore-src -y   # ros2_control, Nav2, SLAM Toolbox, rplidar_ros, rosbridge, ...
colcon build --symlink-install --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF
echo "source ~/optimus_ws/install/setup.bash" >> ~/.bashrc
```

`--parallel-workers 2` keeps a 4 GB Pi from running out of memory during the
C++ build.

---

## 3. Identify the USB devices

Plug in both USB devices and list them by their stable names:

```bash
ls -l /dev/serial/by-id/
```

| Device | Expected `by-id` name | Configured in |
|---|---|---|
| ZLAC8015D (CH340 adapter) | `usb-1a86_USB_Single_Serial_5C65150306-if00` | `src/optimus_bot/description/control/real_hardware.xacro` → `serial_port` |
| RPLIDAR (CP2102) | `usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` | `src/optimus_bot/config/lidar.yaml` → `serial_port` |

The motor adapter's name contains that adapter's serial number. **If you use a
different adapter, put its `by-id` path in `real_hardware.xacro` and rebuild.**
If the LiDAR path from `lidar.yaml` does not exist, `lidar.launch.py` falls back
to `/dev/ttyUSB0` and logs it; you can also pass `serial_port:=/dev/...`.

---

## 4. First power-up (wheels off the ground)

Go through these steps in order, with the drive wheels lifted.

### a. Motor driver alone

The scripts in `~/optimus_ws/tools` talk to the ZLAC8015D directly:

```bash
pip install "pymodbus<3"          # the scripts use the pymodbus 2.x API
# edit PORT at the top of each script to your by-id path (they default to /dev/ttyACM0)
python3 ~/optimus_ws/tools/zlac_read_test.py   # read-only: status, faults, positions, speeds
python3 ~/optimus_ws/tools/motor.py            # SPINS THE MOTORS
```

### b. ros2_control drive stack

```bash
ros2 launch optimus_bot real_robot.launch.py
```

In a second terminal:

```bash
ros2 control list_controllers      # joint_state_broadcaster + optimus_wheel_velocity_controller: active
ros2 topic list | grep commands    # must be /optimus_wheel_velocity_controller/commands
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"   # Ctrl-C stops
```

Both wheels must roll **forward**. If one turns backward, change
`left_motor_inverted` / `right_motor_inverted` in `real_hardware.xacro`
(currently left `1`, right `0`) and rebuild.

### c. LiDAR

```bash
ros2 launch optimus_bot lidar.launch.py   # logs "Starting RPLIDAR on ..."; the driver starts after 10 s
ros2 topic hz /scan                       # about 10 Hz
```

### d. Camera (Pi camera module)

```bash
ros2 launch optimus_bot camera.launch.py camera:=picam
ros2 topic hz /camera/scan                # about 5-10 Hz
```

Measure the mount into the `picam` profile of `config/camera.yaml` first and
run `bringup` / `optimus` with `camera:=picam` - see [CAMERA.md](CAMERA.md).

---

## 5. Normal operation

```bash
ros2 launch optimus_bot bringup.launch.py                 # add use_voice:=true for voice commands
```

This starts the drive, LiDAR, rosbridge (port 9090), the system manager and the
location manager. Mapping, localization and navigation are started and stopped
by the dashboard - do not launch them by hand at the same time.

Maps are saved to `~/optimus_data/maps/` and locations to
`~/optimus_data/locations/warehouse_locations.yaml` on the Pi.

---

## 6. Dashboard

Run the dashboard on your **laptop** (keeps load off the Pi and lets the
browser microphone work, because browsers only allow it on `localhost` or
`https`):

```bash
cd web_dashboard
npm install            # first time
npm run dev            # serves on port 3000
# open: http://localhost:3000/?ros_host=<pi-ip>&ros_port=9090
```

The laptop and the Pi must be on the same network with port 9090 reachable.
Alternatively run `npm run dev` on the Pi (Node 18+) and open
`http://<pi-ip>:3000`; everything works except the browser microphone.

Then:

1. Open **Diagnostics** → all 8 core nodes should be running; press
   **RUN SELF-TEST** (connection, LiDAR, drive, odometry, TF, nodes).
2. Follow the **SOP GUIDE** in the dashboard header: map → save map →
   localize (set initial pose) → navigate.
3. **Ctrl + ,** opens voice commands (needs `use_voice:=true`).

---

## 7. Start automatically on boot (optional)

`~/optimus_ws/tools/optimus-bringup.service` hard-codes user `dk`,
`/home/dk/...` and `/opt/ros/humble`. Edit those to your user, paths and
distro, then:

```bash
sudo cp ~/optimus_ws/tools/optimus-bringup.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now optimus-bringup
journalctl -u optimus-bringup -f
```

---

## 8. First-run checks on Humble

Several launch fixes were verified on Jazzy only. On the first Humble run confirm:

1. `ros2 topic list | grep commands` shows
   `/optimus_wheel_velocity_controller/commands` (and not
   `/controller_manager/commands`).
2. In mapping mode, `/map` is published (Humble's SLAM Toolbox starts directly;
   on Jazzy the launch file configures and activates it).
3. After starting navigation, `ros2 lifecycle get /bt_navigator` reports
   `active`.

---

## Troubleshooting

| Symptom | Check |
|---|---|
| `Failed to activate ZLAC controller` | Device path in `real_hardware.xacro`, `dialout` group, adapter powered, A+/B- wiring |
| Modbus CRC / timeout errors | Termination resistor, cable length, baud 115200, slave ID 1 |
| LiDAR `80008002` on start | Normal after a fast restart - the launch waits 10 s before starting the driver |
| Dashboard shows OFFLINE | `ros_host` in the URL, Pi IP, port 9090 reachable, `rosbridge_websocket` running |
| Robot stutters or resets | Pi power supply (`vcgencmd get_throttled` should be `0x0`) |
