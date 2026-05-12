#include <gtest/gtest.h>
#include <cmath>
#include "control/pid_controller.h"
#include "control/rate_controller.h"
#include "control/actuator_controller.h"

namespace gnc::test {

// PID Controller Tests
class PIDControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    pid_ = std::make_unique<gnc::PIDController>(1.0, 0.1, 0.01);
  }

  std::unique_ptr<gnc::PIDController> pid_;
};

TEST_F(PIDControllerTest, PIDInitialization) {
  EXPECT_NE(pid_, nullptr);
}

TEST_F(PIDControllerTest, PIDStepResponseNominal) {
  double error = 10.0;
  double output = pid_->compute(error, 0.01);
  
  EXPECT_TRUE(std::isfinite(output));
}

TEST_F(PIDControllerTest, PIDProportionalTermWorks) {
  pid_ = std::make_unique<gnc::PIDController>(1.0, 0.0, 0.0);
  double output = pid_->compute(5.0, 0.01);
  
  EXPECT_NEAR(output, 5.0, 0.1);
}

TEST_F(PIDControllerTest, PIDOutputSaturation) {
  double large_error = 1000.0;
  double output = pid_->compute(large_error, 0.01);
  
  EXPECT_TRUE(std::isfinite(output));
  EXPECT_LE(std::abs(output), 1000.0);
}

TEST_F(PIDControllerTest, PIDResetWorks) {
  pid_->compute(5.0, 0.01);
  pid_->compute(3.0, 0.01);
  pid_->reset();
  
  double output = pid_->compute(10.0, 0.01);
  EXPECT_TRUE(std::isfinite(output));
}

// Rate Controller Tests
class RateControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rate_controller_ = std::make_unique<gnc::RateController>();
  }

  std::unique_ptr<gnc::RateController> rate_controller_;
};

TEST_F(RateControllerTest, RateControllerInitialization) {
  EXPECT_NE(rate_controller_, nullptr);
}

TEST_F(RateControllerTest, RateControllerOutput) {
  std::vector<double> desired_rates = {0.1, 0.2, 0.3};
  std::vector<double> actual_rates = {0.0, 0.0, 0.0};
  
  auto output = rate_controller_->compute(desired_rates, actual_rates, 0.01);
  
  EXPECT_EQ(output.size(), 3);
  for (double val : output) {
    EXPECT_TRUE(std::isfinite(val));
  }
}

// Actuator Controller Tests
class ActuatorControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    actuator_ = std::make_unique<gnc::ActuatorController>();
  }

  std::unique_ptr<gnc::ActuatorController> actuator_;
};

TEST_F(ActuatorControllerTest, ActuatorInitialization) {
  EXPECT_NE(actuator_, nullptr);
}

TEST_F(ActuatorControllerTest, ActuatorCommand) {
  std::vector<double> commands = {100.0, 100.0, 100.0, 100.0};
  auto result = actuator_->set_commands(commands);
  
  EXPECT_TRUE(result);
}

TEST_F(ActuatorControllerTest, ActuatorOutputSaturation) {
  std::vector<double> invalid_commands = {10000.0, -10000.0, 0.0, 0.0};
  auto result = actuator_->set_commands(invalid_commands);
  
  auto actual_output = actuator_->get_throttle_values();
  for (double val : actual_output) {
    EXPECT_GE(val, 0.0);
    EXPECT_LE(val, 100.0);
  }
}

}  // namespace gnc::test
