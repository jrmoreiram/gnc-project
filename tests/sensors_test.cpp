#include <gtest/gtest.h>
#include <cmath>
#include "sensors/imu.h"
#include "sensors/gps.h"
#include "sensors/barometer.h"

namespace gnc::test {

// IMU Tests
class IMUTest : public ::testing::Test {
 protected:
  void SetUp() override {
    imu_ = std::make_unique<gnc::IMU>();
  }

  std::unique_ptr<gnc::IMU> imu_;
};

TEST_F(IMUTest, IMUInitialization) {
  EXPECT_NE(imu_, nullptr);
}

TEST_F(IMUTest, IMUOutputSize) {
  auto acceleration = imu_->get_acceleration();
  auto angular_velocity = imu_->get_angular_velocity();
  
  EXPECT_EQ(acceleration.size(), 3);
  EXPECT_EQ(angular_velocity.size(), 3);
}

TEST_F(IMUTest, IMUNoiseIsReasonable) {
  auto acceleration = imu_->get_acceleration();
  
  for (int i = 0; i < acceleration.size(); ++i) {
    EXPECT_TRUE(std::isfinite(acceleration[i]));
  }
}

// GPS Tests
class GPSTest : public ::testing::Test {
 protected:
  void SetUp() override {
    gps_ = std::make_unique<gnc::GPS>();
  }

  std::unique_ptr<gnc::GPS> gps_;
};

TEST_F(GPSTest, GPSInitialization) {
  EXPECT_NE(gps_, nullptr);
}

TEST_F(GPSTest, GPSPositionIsValid) {
  auto position = gps_->get_position();
  
  EXPECT_EQ(position.size(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(std::isfinite(position[i]));
  }
}

TEST_F(GPSTest, GPSVelocityIsValid) {
  auto velocity = gps_->get_velocity();
  
  EXPECT_EQ(velocity.size(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(std::isfinite(velocity[i]));
  }
}

// Barometer Tests
class BarometerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    barometer_ = std::make_unique<gnc::Barometer>();
  }

  std::unique_ptr<gnc::Barometer> barometer_;
};

TEST_F(BarometerTest, BarometerInitialization) {
  EXPECT_NE(barometer_, nullptr);
}

TEST_F(BarometerTest, AltitudeIsReasonable) {
  auto altitude = barometer_->get_altitude();
  
  EXPECT_TRUE(std::isfinite(altitude));
  EXPECT_GE(altitude, -500);
  EXPECT_LE(altitude, 150000);
}

TEST_F(BarometerTest, PressureIsReasonable) {
  auto pressure = barometer_->get_pressure();
  
  EXPECT_TRUE(std::isfinite(pressure));
  EXPECT_GT(pressure, 0);
}

}  // namespace gnc::test
