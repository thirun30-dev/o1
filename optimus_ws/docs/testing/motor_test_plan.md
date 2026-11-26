# Motor Test Plan

This verifies basic hardware communication.

## Procedure
Lift the drive wheels off the ground and keep the hardware stop button within reach.

1. Launch the hardware:
   ```bash
   ros2 launch optimus_bot real_robot.launch.py
   ```
2. Open a separate terminal for Teleop:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
3. Press `i` to drive forward.
   - **Validation**: Do both wheels spin in the forward direction? If one spins backward, change `left_motor_inverted` / `right_motor_inverted` in `optimus_bot/description/control/real_hardware.xacro` (currently left `1`, right `0`) and rebuild. Also confirm `ros2 topic list | grep commands` shows `/optimus_wheel_velocity_controller/commands`.
4. Press `k` to stop.
   - **Validation**: Motors must smoothly decelerate to zero velocity.
5. Press `,` to reverse.
   - **Validation**: Motors must reverse cleanly.
