# Odometry Calibration Test Plan

To bring the robot out of open-loop dead-reckoning and activate closed-loop position feedback, you must physically measure the Encoder Counts Per Revolution (CPR).

## Staged Calibration Procedure
1. Prop the robot up so its drive wheels do not touch the ground, OR place a physical mark on the wheel and the floor.
2. Read the raw position of the left wheel (`0x20A7-0x20A8`) via a Modbus scanner or temporary ROS logging.
3. Physically rotate the wheel exactly one full revolution (360 degrees / 2π radians).
4. Read the raw position again.
5. The absolute difference is your **Encoder CPR**.
6. Open `optimus_bot/description/control/real_hardware.xacro`.
7. Set `encoder_cpr` to your measured value and `encoder_cpr_verified` to `true`
   (current values: `16427`, `true`). Position feedback stays at zero unless
   both are set.
8. Rebuild the workspace using `colcon build`.

The drivetrain odometry integrates wheel **velocity**, so it does not depend on
`encoder_cpr`; the position scale only affects the `position` joint states.

## Geometric Calibration
Measure the distance between the centre-lines of the left and right wheel contact patches and set `wheel_separation` in `optimus_bot/config/drivetrain.yaml` (currently `0.265`, `wheel_separation_verified: true`). The launch files pass it to `wheels.xacro`; do not edit it there.

A quick check: command a slow spin in place (`angular.z: 0.5`) for one full turn on the floor. If `/odom` reports noticeably more or less than 360°, the separation or wheel slip needs attention.
