# ZLAC8015D Motor Specifications

## Known Architecture
- Dual-channel hub motor controller.
- Feedback: Internal relative/absolute encoders.
- Native Command Protocol: Signed 16-bit Target Velocity (RPM).
- Safety: Integrated fault registries for overcurrent, overvoltage, and phase loss.

## ZLLG50ASM200 V2.0 hub motor (motor parameter sheet)

| Parameter | Value |
|---|---|
| Rated / peak speed | 270 / 350 RPM (350 RPM is the command ceiling) |
| Rated / peak torque | 3 / 9 Nm |
| Rated / peak current | 5 / 15 A |
| Encoder | 4096 lines |
| Wheel diameter | 131 mm (radius 0.0655 m) |
| Gear ratio | 1.0 (direct-drive hub motor) |

At the 0.5 m/s software speed limit the wheels turn at about 73 RPM, well
below the rated speed. The driver reports speed, position, status and fault
registers; it does not report motor current or temperature to ROS.

## Physical Integration
The exact wheel geometry (radius, separation, mass) is completely isolated in the `optimus_bot` URDF package (`wheels.xacro`). Do not hardcode motor specifications in the driver, as ZLAC drivers are used across multiple chassis form factors.
