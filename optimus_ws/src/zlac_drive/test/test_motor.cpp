#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "zlac_drive/driver/motor.hpp"

namespace zlac_drive::driver
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1e-9;

TEST(MotorTest, ConvertsRpmToRadiansPerSecond)
{
  const double radians_per_second =
    Motor::rpm_to_rad_per_sec(60.0);

  EXPECT_NEAR(
    radians_per_second,
    2.0 * kPi,
    kTolerance);
}

TEST(MotorTest, ConvertsRadiansPerSecondToRpm)
{
  const double rpm =
    Motor::rad_per_sec_to_rpm(
      2.0 * kPi);

  EXPECT_NEAR(
    rpm,
    60.0,
    kTolerance);
}

TEST(MotorTest, RejectsInvalidGearRatio)
{
  MotorConfig config;

  config.gear_ratio = 0.0;

  EXPECT_THROW(
    Motor motor(config),
    std::invalid_argument);
}

TEST(MotorTest, RejectsNegativeGearRatio)
{
  MotorConfig config;

  config.gear_ratio = -1.0;

  EXPECT_THROW(
    Motor motor(config),
    std::invalid_argument);
}

TEST(MotorTest, RejectsEncoderConversionWhenUnverified)
{
  MotorConfig config;

  config.encoder_counts_per_revolution = 4096;
  config.encoder_counts_verified = false;

  Motor motor(config);

  EXPECT_THROW(
    motor.encoder_counts_to_radians(100),
    std::runtime_error);
}

TEST(MotorTest, RejectsVerifiedZeroEncoderResolution)
{
  MotorConfig config;

  config.encoder_counts_per_revolution = 0;
  config.encoder_counts_verified = true;

  EXPECT_THROW(
    Motor motor(config),
    std::invalid_argument);
}

TEST(MotorTest, ConvertsEncoderCountsToRadians)
{
  MotorConfig config;

  config.encoder_counts_per_revolution = 4096;
  config.encoder_counts_verified = true;

  Motor motor(config);

  const double radians =
    motor.encoder_counts_to_radians(4096);

  EXPECT_NEAR(
    radians,
    2.0 * kPi,
    kTolerance);
}

TEST(MotorTest, ConvertsRadiansToEncoderCounts)
{
  MotorConfig config;

  config.encoder_counts_per_revolution = 4096;
  config.encoder_counts_verified = true;

  Motor motor(config);

  const int64_t counts =
    motor.radians_to_encoder_counts(
      2.0 * kPi);

  EXPECT_EQ(counts, 4096);
}

TEST(MotorTest, RpmConversionRoundTrip)
{
  const double original_rpm = 270.0;

  const double rad_per_sec =
    Motor::rpm_to_rad_per_sec(
      original_rpm);

  const double recovered_rpm =
    Motor::rad_per_sec_to_rpm(
      rad_per_sec);

  EXPECT_NEAR(
    recovered_rpm,
    original_rpm,
    kTolerance);
}

TEST(MotorTest, UnityGearRatioLeavesWheelSpeedEqualToMotorSpeed)
{
  MotorConfig config;
  config.gear_ratio = 1.0;
  const Motor motor(config);

  EXPECT_NEAR(
    motor.motor_rpm_to_wheel_rad_per_sec(60.0), 2.0 * kPi, kTolerance);
  EXPECT_NEAR(
    motor.wheel_rad_per_sec_to_motor_rpm(2.0 * kPi), 60.0, kTolerance);
}

TEST(MotorTest, AppliesGearRatioBetweenMotorAndWheel)
{
  MotorConfig config;
  config.gear_ratio = 4.0;  // 4 motor revolutions per wheel revolution
  const Motor motor(config);

  // 240 motor RPM -> 60 wheel RPM -> 2*pi wheel rad/s.
  EXPECT_NEAR(
    motor.motor_rpm_to_wheel_rad_per_sec(240.0), 2.0 * kPi, kTolerance);
  // 1 wheel rev/s (2*pi rad/s) -> 60 wheel RPM -> 240 motor RPM.
  EXPECT_NEAR(
    motor.wheel_rad_per_sec_to_motor_rpm(2.0 * kPi), 240.0, kTolerance);
  EXPECT_NEAR(
    motor.motor_rpm_to_wheel_rad_per_sec(
      motor.wheel_rad_per_sec_to_motor_rpm(-1.25)),
    -1.25, kTolerance);
}

}  // namespace zlac_drive::driver