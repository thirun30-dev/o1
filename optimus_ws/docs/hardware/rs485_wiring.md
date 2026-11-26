# RS-485 / USB Wiring Guidelines

The robot computer (robot laptop or Raspberry Pi) connects to the ZLAC8015D
controller through a USB-to-RS485 converter (CH340, USB ID `1a86`).

## Linux device mapping

The device is addressed by its stable `by-id` name, set in
`optimus_bot/description/control/real_hardware.xacro`:

```text
/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C65150306-if00
```

This name includes the adapter's serial number, so it survives re-plugging and
does not depend on `/dev/ttyACM*` / `/dev/ttyUSB*` numbering. If you replace the
adapter, run `ls -l /dev/serial/by-id/`, put the new name in
`real_hardware.xacro` (`serial_port`) and rebuild.

| Setting | Value |
|---|---|
| Baud rate | 115200, 8N1 |
| Modbus slave ID | 1 (one dual-channel driver for both wheels) |
| Timeout | 500 ms (`timeout_ms` in `real_hardware.xacro`) |

## Troubleshooting permissions

If `ZlacSystemHardware` fails to activate with a connection error during `real_robot.launch.py`:

1. Check the device exists: `ls -l /dev/serial/by-id/`
2. Check user permissions: `groups` must include `dialout`.
3. If not: `sudo usermod -aG dialout $USER`, then log out and back in.

## Testing the link without ROS

`optimus_ws/tools/zlac_read_test.py` (read-only) and `tools/motor.py` (spins
the motors) talk to the driver directly. They need `pip install "pymodbus<3"`
and their `PORT` constant set to the `by-id` path above.

## Modbus RTU signal requirements

- Ensure Data `A+` and `B-` lines are securely fastened to the controller.
- Use terminating resistors (120 Ohm) if experiencing CRC errors on longer communication wire runs.
- Power the robot computer from a stable supply; USB brown-outs show up as
  Modbus timeouts.
