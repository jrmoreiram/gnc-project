#include "estimation/state_estimator.h"

namespace gnc::estimation {

StateEstimator::StateEstimator() = default;

void StateEstimator::initialize(const VehicleState& initial) { ekf_.initialize(initial); }

void StateEstimator::step(const sensors::ImuMeasurement& imu,
                          const sensors::GpsMeasurement* gps,
                          const sensors::BarometerMeasurement* baro,
                          double dt_s) {
  ekf_.predict(imu, dt_s);
  if (gps != nullptr) {
    ekf_.updateGps(*gps);
  }
  if (baro != nullptr) {
    ekf_.updateBarometer(*baro);
  }
}

VehicleState StateEstimator::state() const { return ekf_.getState(); }

}  // namespace gnc::estimation
