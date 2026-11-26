# Package Interfaces

The Optimus workspace has four ROS 2 packages with a strict separation of
concerns between the reusable hardware driver and robot-specific configuration.

## 1. `zlac_drive`
A **reusable hardware driver**. It is agnostic to the robot's dimensions and navigation.
- **Inputs**: per-wheel `velocity` commands (rad/s) from `ros2_control`.
- **Outputs**: per-wheel `position` and `velocity` state interfaces.
- **Internal**: Modbus RTU framing and CRC, ZLAC8015D register access, speed
  clamping (`max_velocity_rpm`), direction mapping, gear ratio and encoder
  scaling. Parameters come from the URDF (`real_hardware.xacro`).

## 2. `optimus_bot`
The **robot integration package**. It owns all physical traits and application-level configuration.
- **URDF/Xacro**: `base.xacro` (0.60 × 0.40 × 0.30 m body), `wheels.xacro`,
  `lidar.xacro`, and the real (`real_hardware.xacro`) or simulated
  (`simulation.xacro`) ros2_control blocks.
- **Controllers**: `forward_command_controller` (`optimus_wheel_velocity_controller`)
  and `joint_state_broadcaster`. `diff_drive_controller` is intentionally not used.
- **Drivetrain**: `optimus_drivetrain_node` - `/cmd_vel` → wheel commands,
  `/joint_states` → `/odom` and `odom -> base_link`.
- **Managers**: `system_manager_node.py` (modes, health, e-stop, maps) and
  `location_manager_node.py` (locations, initial pose, Nav2 goals).
- **Navigation/SLAM**: Nav2, SLAM Toolbox and AMCL launch files and parameters.

## 3. `optimus_voice`
Optional voice control: wake word "Optimus", intent parsing, spoken replies.
Accepts text on `/optimus/voice/text_command` (dashboard, Ctrl + ,) or a local
microphone, and drives Nav2 actions.

## 4. `optimus_sim`
Gazebo Sim (Jazzy) warehouse world with SLAM + Nav2 and obstacle-response
tests. Not built on Humble.
