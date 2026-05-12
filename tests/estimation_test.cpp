#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>
#include "estimation/ekf.h"
#include "estimation/state_estimator.h"

namespace gnc::test {

// EKF Tests
class EKFTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ekf_ = std::make_unique<gnc::EKFilter>();
  }

  std::unique_ptr<gnc::EKFilter> ekf_;
};

TEST_F(EKFTest, EKFInitialization) {
  EXPECT_NE(ekf_, nullptr);
}

TEST_F(EKFTest, EKFStateVectorSize) {
  auto state = ekf_->get_state();
  
  EXPECT_GE(state.rows(), 6);
}

TEST_F(EKFTest, EKFCovaranceMatrix) {
  auto cov = ekf_->get_covariance();
  
  EXPECT_GT(cov.rows(), 0);
  EXPECT_EQ(cov.rows(), cov.cols());
}

// State Estimator Tests
class StateEstimatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    estimator_ = std::make_unique<gnc::StateEstimator>();
  }

  std::unique_ptr<gnc::StateEstimator> estimator_;
};

TEST_F(StateEstimatorTest, StateEstimatorInitialization) {
  EXPECT_NE(estimator_, nullptr);
}

TEST_F(StateEstimatorTest, EstimatedPositionIsValid) {
  auto position = estimator_->get_position();
  
  EXPECT_EQ(position.size(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(std::isfinite(position[i]));
  }
}

TEST_F(StateEstimatorTest, EstimatedVelocityIsValid) {
  auto velocity = estimator_->get_velocity();
  
  EXPECT_EQ(velocity.size(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(std::isfinite(velocity[i]));
  }
}

TEST_F(StateEstimatorTest, EstimatedAttitudeIsValid) {
  auto attitude = estimator_->get_attitude();
  
  EXPECT_EQ(attitude.size(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(std::isfinite(attitude[i]));
  }
}

TEST_F(StateEstimatorTest, CovaranceMatrixIsPositiveDefinite) {
  auto cov = estimator_->get_covariance();
  
  Eigen::EigenSolver<Eigen::MatrixXd> solver(cov);
  auto eigenvalues = solver.eigenvalues();
  
  for (int i = 0; i < eigenvalues.size(); ++i) {
    EXPECT_GT(eigenvalues[i].real(), -1e-6);
  }
}

}  // namespace gnc::test
