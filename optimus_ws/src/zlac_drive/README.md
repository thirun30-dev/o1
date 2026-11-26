# zlac_drive

Reusable ROS 2 (Humble / Jazzy) `ros2_control` hardware package for the ZLAC8015D dual motor controller (used with ZLLG50ASM200 V2.0 hub motors). This package contains no robot geometry, no Nav2/SLAM/AMCL, and no RPLIDAR code - see `optimus_bot` for those.

## Architecture

```
ros2_control (controller_manager)
        |
        v
ZlacSystemHardware        (hardware/, ros2_control SystemInterface)
        |
        v
ZlacDriveDriver            (driver/, controller-specific commands + state machine)
        |
        v
ModbusClient                (modbus/, generic Modbus RTU transport, CRC, framing)
        |
        v
RS-485 / Modbus RTU -> ZLAC8015D -> left/right ZLLG50ASM200 V2.0 motors
```

Each layer only knows about the one below it: `ModbusClient` has no ZLAC-specific register knowledge, `ZlacDriveDriver` has no ROS/ros2_control knowledge, and `ZlacSystemHardware` has no Modbus framing knowledge.

## Register map and verification status

The supplied ZLAC8015D user manual (v1.02) documents electrical ratings, connector pinouts, and LED fault codes, but **does not publish a Modbus register table**. Every register address in `include/zlac_drive/registers.hpp` is therefore bring-up (`PHYSICALLY_VERIFIED`) knowledge, not something checkable against the manual - see the header comment in that file for the full traceability policy (`SOURCE_VERIFIED` / `PHYSICALLY_VERIFIED` / `DERIVED` / `CALIBRATION_REQUIRED` / `UNVERIFIED`).

| Register | Address | Access | Status |
|---|---|---|---|
| Control mode (velocity=3) | `0x200D` | write | PHYSICALLY_VERIFIED |
| Control command (enable=0x0008) | `0x200E` | write | PHYSICALLY_VERIFIED (enable value only - no verified disable value) |
| Left/right target velocity | `0x2088` / `0x2089` | write | PHYSICALLY_VERIFIED, RPM, int16 |
| Left/right position | `0x20A7-0x20A8` / `0x20A9-0x20AA` | read | PHYSICALLY_VERIFIED presence; CALIBRATION_REQUIRED scale |
| Left/right actual velocity | `0x20AB` / `0x20AC` | read | PHYSICALLY_VERIFIED, `RPM = raw / 10.0` |
| Driver status | `0x20A2` | read | PHYSICALLY_VERIFIED presence; UNVERIFIED per-bit meaning |
| Left/right fault | `0x20A5` / `0x20A6` | read | PHYSICALLY_VERIFIED presence; UNVERIFIED per-bit meaning |

`0x20A7-0x20AC` are contiguous, so `ZlacDriveDriver::read_feedback()` reads all six registers in a single block transaction instead of four separate ones.

**No documented fault-clear/disable-torque register exists.** `ZlacDriveDriver::disable()` and `stop()` command zero velocity (a verified safe-stop), which is not the same as a servo/torque disable. A controller fault currently requires a verified external recovery procedure (e.g. power cycle) - do not invent a fault-clear register.

## Safety pipeline (`ZlacDriveDriver::set_velocity`)

1. Reject non-finite input (treated as zero).
2. Clamp to `[-max_velocity_rpm, +max_velocity_rpm]` (configurable, must be `<= MotorLimits::kPeakRpm = 350`).
3. Apply explicit `left_motor_inverted` / `right_motor_inverted` direction mapping.
4. Intentional round + explicit `int16_t` range check (never a blind cast).
5. Send via a single Modbus block write.

## URDF parameters (`optimus_bot/description/control/real_hardware.xacro`)

| Parameter | Current value | Meaning |
|---|---|---|
| `serial_port` | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C65150306-if00` | USB-RS485 adapter (stable by-id name) |
| `baud_rate` / `slave_id` | `115200` / `1` | Modbus RTU link |
| `timeout_ms` | `500` | Modbus response timeout |
| `max_velocity_rpm` | `350` | Motor speed ceiling (must be `<= 350`) |
| `gear_ratio` | `1.0` | Motor revolutions per wheel revolution |
| `encoder_cpr` / `encoder_cpr_verified` | `16427` / `true` | Counts per wheel revolution, and the explicit "measured" flag |
| `left_motor_inverted` / `right_motor_inverted` | `1` / `0` | Direction mapping so positive commands roll both wheels forward |

The hardware plugin uses these parameters as given; nothing is hard-coded in C++.

## Units and gear ratio

ros2_control commands and reports **wheel** rad/s. The ZLAC8015D commands and
reports **motor** RPM. `ZlacSystemHardware` converts between them with
`Motor::wheel_rad_per_sec_to_motor_rpm()` / `motor_rpm_to_wheel_rad_per_sec()`,
applying `gear_ratio`. `max_velocity_rpm` is a motor-side limit, so it stays
correct for any gear ratio. `encoder_cpr` is measured at the wheel and already
includes any gearing.

## Encoder scaling - CRITICAL

The ZLLG50ASM200 V2.0 motor has 4096 encoder lines (`SOURCE_VERIFIED`, motor parameter sheet). **This does not prove 4096 effective counts per wheel revolution** at the ZLAC8015D position registers - quadrature decoding, gearing, and firmware scaling are unknown. `hardware_interface::HardwareInfo` requires **both** `encoder_cpr` **and** a separate `encoder_cpr_verified="true"` parameter before position feedback is used; supplying `encoder_cpr` alone is not sufficient. Until verified, `left_position_`/`right_position_` state interfaces remain at `0.0` (an honest default, not a fabricated value).

To calibrate: command a known number of full wheel revolutions at a fixed low RPM, compare against the raw encoder-count delta from `ZlacDriveDriver::read_feedback()`, compute the effective counts-per-revolution, then set `encoder_cpr_verified: "true"` in `description/control/real_hardware.xacro`.

## Driver state machine

`DISCONNECTED -> CONNECTING -> CONNECTED -> INITIALIZED -> ENABLED`, with `COMMUNICATION_ERROR` reachable from any state on a Modbus transport failure and `FAULT` reachable on a controller-reported fault (see `driver/driver_state.hpp`).

## Build and test

```bash
cd ~/optimus_ws
colcon build --packages-select zlac_drive --symlink-install
source install/setup.bash
colcon test --packages-select zlac_drive
colcon test-result --verbose
```

## Remaining hardware validation (see final engineering report)

1. Effective encoder counts per wheel revolution - set to `16427` and marked verified in `real_hardware.xacro`; re-measure after any wheel or motor change.
2. Left/right sign convention for forward robot motion (`left_motor_inverted`/`right_motor_inverted`) - set from bring-up testing (left `1`, right `0`); re-check with the wheels off the ground after wiring changes.
3. Fault register bit-level decoding (`UNVERIFIED` - manual only documents LED blink codes, not register bits).
4. Communication-loss and fault-recovery behavior under sustained load.
