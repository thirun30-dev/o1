# Optimus AMR System Architecture

The software architecture of the Optimus robot follows a bottom-up hardware abstraction pattern.

## Control Flow Architecture

1. **Hardware Layer (ZLAC8015D Controller)**: the dual-channel motor driver connected to the left and right ZLLG50ASM200 hub motors.
2. **Transport Layer (Modbus RTU)**: RS-485 serial (`modbus_client.cpp`) at 115200 baud, handling framing and CRC verification.
3. **Driver Layer (`ZlacDriveDriver`)**: translates motor commands into ZLAC registers (velocity mode 3, enable 0x08, target RPM, feedback block reads), clamps speed and applies the direction mapping.
4. **Hardware Interface (`ZlacSystemHardware`)**: the `ros2_control` plugin. It parses URDF parameters (`serial_port`, `baud_rate`, `slave_id`, `encoder_cpr`, `gear_ratio`, `max_velocity_rpm`, direction flags) and converts wheel rad/s ↔ motor RPM using the `Motor` utility class.
5. **Controller Layer**: `forward_command_controller` forwards per-wheel velocity commands; `joint_state_broadcaster` publishes wheel state.
6. **Drivetrain Layer (`optimus_drivetrain_node`)**: converts `geometry_msgs/Twist` to per-wheel commands with a 0.5 s timeout, and integrates wheel speeds into `/odom`.
7. **Application Layer (Nav2 / SLAM / managers / dashboard)**: plans trajectories from `/scan`, `/odom` and the map, and supervises modes and safety.
