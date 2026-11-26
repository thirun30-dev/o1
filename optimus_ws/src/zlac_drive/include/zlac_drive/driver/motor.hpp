#ifndef ZLAC_DRIVE__DRIVER__MOTOR_HPP_
#define ZLAC_DRIVE__DRIVER__MOTOR_HPP_

#include <cstdint>

namespace zlac_drive::driver
{

/**
 * @brief ZLLG50ASM200 V2.0 motor specification constants.
 *
 * Source: ZLLG50ASM200 V2.0 motor parameter sheet (SOURCE_VERIFIED).
 * These are hardware specification values, not driver protocol values -
 * see zlac_drive/registers.hpp for protocol-layer constants.
 */
struct MotorLimits
{
  /// Rated speed. SOURCE_VERIFIED (motor parameter sheet).
  static constexpr double kRatedRpm = 270.0;

  /// Peak (maximum) speed. SOURCE_VERIFIED (motor parameter sheet).
  /// This is the absolute ceiling for any commanded velocity.
  static constexpr double kPeakRpm = 350.0;

  /// Rated torque, Nm. SOURCE_VERIFIED (motor parameter sheet).
  static constexpr double kRatedTorqueNm = 3.0;

  /// Peak torque, Nm. SOURCE_VERIFIED (motor parameter sheet).
  static constexpr double kPeakTorqueNm = 9.0;

  /// Rated current, A. SOURCE_VERIFIED (motor parameter sheet).
  static constexpr double kRatedCurrentA = 5.0;

  /// Peak current, A. SOURCE_VERIFIED (motor parameter sheet).
  static constexpr double kPeakCurrentA = 15.0;

  /// Encoder line count. SOURCE_VERIFIED (motor parameter sheet).
  /// This is the raw encoder specification, NOT a proven
  /// counts-per-wheel-revolution scale - see MotorConfig below.
  static constexpr uint32_t kEncoderLines = 4096;

  /// Nominal wheel radius, m (from 131 +/-2 mm wheel diameter).
  /// DERIVED: 0.131 m / 2.
  static constexpr double kNominalWheelRadiusM = 0.0655;
};

/**
 * @brief Motor configuration required for unit conversion.
 *
 * Values must be populated from verified motor/controller
 * configuration.
 */
struct MotorConfig
{
  /**
   * Motor revolutions per wheel revolution (1.0 for the Optimus hub
   * motors). The ZLAC8015D commands and reports MOTOR RPM; ros2_control
   * joints are WHEEL rad/s - see Motor::motor_rpm_to_wheel_rad_per_sec().
   */
  double gear_ratio{1.0};

  /**
   * Effective encoder counts per WHEEL revolution, as measured by the
   * calibration procedure in real_hardware.xacro (command N wheel
   * revolutions, compare the raw count delta). Because it is measured at
   * the wheel it already includes any gearing.
   *
   * Set to a verified value before encoder conversion is used.
   */
  uint32_t encoder_counts_per_revolution{0};

  bool encoder_counts_verified{false};
};


/**
 * @brief Generic motor state in ROS-compatible SI units.
 */
struct MotorState
{
  /**
   * Motor shaft position in radians.
   */
  double position_rad{0.0};

  /**
   * Motor shaft velocity in radians per second.
   */
  double velocity_rad_per_sec{0.0};
};


/**
 * @brief Conversion utilities for motor feedback and commands.
 *
 * This class performs mathematical conversions only.
 * It does not perform serial or Modbus communication.
 */
class Motor
{
public:
  explicit Motor(const MotorConfig & config);

  /**
   * @brief Convert encoder counts to wheel radians.
   *
   * @throws std::runtime_error if encoder resolution is not verified.
   */
  double encoder_counts_to_radians(
    int64_t encoder_counts) const;

  /**
   * @brief Convert wheel radians to encoder counts.
   *
   * @throws std::runtime_error if encoder resolution is not verified.
   */
  int64_t radians_to_encoder_counts(
    double radians) const;

  /**
   * @brief Convert RPM to radians per second.
   */
  static double rpm_to_rad_per_sec(
    double rpm);

  /**
   * @brief Convert radians per second to RPM.
   */
  static double rad_per_sec_to_rpm(
    double radians_per_second);

  /**
   * @brief Convert controller-reported motor RPM to wheel rad/s.
   */
  double motor_rpm_to_wheel_rad_per_sec(
    double motor_rpm) const;

  /**
   * @brief Convert a wheel rad/s command to motor RPM for the controller.
   */
  double wheel_rad_per_sec_to_motor_rpm(
    double wheel_radians_per_second) const;

  double gear_ratio() const noexcept {return config_.gear_ratio;}

private:
  MotorConfig config_;
};

}  // namespace zlac_drive::driver

#endif  // ZLAC_DRIVE__DRIVER__MOTOR_HPP_