#include "estimation/ekf.h"

namespace gnc::estimation {

Ekf::Ekf() {
  x_.setZero();
  P_.setIdentity();
  P_ *= 10.0;
  Q_.setIdentity();
  Q_ *= 0.05;
}

void Ekf::initialize(const VehicleState& initial_state) {
  x_.segment<3>(0) = initial_state.position_ned;
  x_.segment<3>(3) = initial_state.velocity_ned;
  x_.segment<3>(6) = initial_state.attitude_rpy;
  initialized_ = true;
}

void Ekf::predict(const sensors::ImuMeasurement& imu, double dt_s) {
  if (!initialized_) {
    return;
  }

  // x = [p(3), v(3), attitude(3)]
  x_.segment<3>(3) += imu.accel_mps2 * dt_s;
  x_.segment<3>(0) += x_.segment<3>(3) * dt_s;
  x_.segment<3>(6) += imu.gyro_rps * dt_s;

  Eigen::Matrix<double, 9, 9> F = Eigen::Matrix<double, 9, 9>::Identity();
  F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt_s;

  P_ = F * P_ * F.transpose() + Q_ * dt_s;
}

void Ekf::updateGps(const sensors::GpsMeasurement& gps) {
  if (!initialized_ || !gps.valid) {
    return;
  }

  Eigen::Matrix<double, 6, 9> H = Eigen::Matrix<double, 6, 9>::Zero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  H.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

  Eigen::Matrix<double, 6, 1> z;
  z.segment<3>(0) = gps.position_ned_m;
  z.segment<3>(3) = gps.velocity_ned_mps;

  Eigen::Matrix<double, 6, 1> y = z - H * x_;
  Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Identity() * 2.0;
  Eigen::Matrix<double, 9, 6> K = P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();

  x_ += K * y;
  P_ = (Eigen::Matrix<double, 9, 9>::Identity() - K * H) * P_;
}

void Ekf::updateBarometer(const sensors::BarometerMeasurement& baro) {
  if (!initialized_) {
    return;
  }

  Eigen::Matrix<double, 1, 9> H = Eigen::Matrix<double, 1, 9>::Zero();
  // altitude ~= -z_ned
  H(0, 2) = -1.0;

  Eigen::Matrix<double, 1, 1> z;
  z(0, 0) = baro.altitude_m;

  Eigen::Matrix<double, 1, 1> y = z - H * x_;
  Eigen::Matrix<double, 1, 1> R;
  R(0, 0) = 1.5;

  Eigen::Matrix<double, 9, 1> K = P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();
  x_ += K * y;
  P_ = (Eigen::Matrix<double, 9, 9>::Identity() - K * H) * P_;
}

VehicleState Ekf::getState() const {
  VehicleState s;
  if (!initialized_) {
    return s;
  }
  s.position_ned = x_.segment<3>(0);
  s.velocity_ned = x_.segment<3>(3);
  s.attitude_rpy = x_.segment<3>(6);
  s.altitude_m = -s.position_ned.z();
  return s;
}

}  // namespace gnc::estimation
