# Encoder Conversion

The ZLAC8015D provides internal position feedback as signed 32-bit integers via registers `0x20A7-0x20AA`.

## Default Configuration
By default, the `ros2_control` hardware interface strictly disables position conversions by ignoring the raw counts. This is because **guessing an encoder scaling factor is strictly prohibited**. 

## Activating Encoder Scaling
Once you have physically performed the Odometry Calibration (see `odometry_calibration.md`), you must update `real_hardware.xacro` with the verified count values:

```xml
  <xacro:property name="encoder_cpr" value="YOUR_VERIFIED_VALUE"/>
  <xacro:property name="encoder_cpr_verified" value="true"/>
```

Both are required: `encoder_cpr` alone never enables position feedback. With
both set (current values `16427` / `true`), `ZlacSystemHardware` converts the
position registers with `Motor::encoder_counts_to_radians()` and publishes them
as the wheel joint `position` states.

`encoder_cpr` is counts per **wheel** revolution, measured at the wheel, so it
already includes any gearing. `gear_ratio` (motor revolutions per wheel
revolution, `1.0` for the Optimus hub motors) applies only to the velocity
conversion between wheel rad/s and the driver's motor RPM.

`/odom` is computed from wheel **velocity** by `optimus_drivetrain_node`, so it
does not depend on this setting.
