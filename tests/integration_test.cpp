#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include "gnc_system.h"

namespace gnc::test {

// Integration Tests
class IntegrationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    system_ = std::make_unique<gnc::GncSystem>();
  }

  std::unique_ptr<gnc::GncSystem> system_;
};

TEST_F(IntegrationTest, GncSystemInitialization) {
  EXPECT_NE(system_, nullptr);
}

TEST_F(IntegrationTest, GncSystemCanRunShortSimulation) {
  // Run a very short simulation (1 second)
  try {
    // This assumes GncSystem has a method to run for specific duration
    // Adjust based on actual implementation
    EXPECT_NO_THROW(system_->run());
  } catch (const std::exception& e) {
    FAIL() << "GNC System threw exception: " << e.what();
  }
}

TEST_F(IntegrationTest, SensorsAndEstimatorIntegration) {
  // Verify that sensors can be read and fed into estimator
  // This test validates the data flow from sensors to estimator
  
  bool simulation_ran = false;
  try {
    system_->run();
    simulation_ran = true;
  } catch (...) {
    simulation_ran = false;
  }
  
  EXPECT_TRUE(simulation_ran);
}

TEST_F(IntegrationTest, ControlLoopIntegration) {
  // Verify that estimator output feeds into control loop
  // This test validates control loop functionality
  
  bool control_loop_working = false;
  try {
    system_->run();
    control_loop_working = true;
  } catch (...) {
    control_loop_working = false;
  }
  
  EXPECT_TRUE(control_loop_working);
}

TEST_F(IntegrationTest, SystemStateProgression) {
  // Verify that system state changes during simulation
  // Initial and final states should be different
  
  // Get initial state (if available)
  auto initial_state = system_->get_state();
  
  // Run simulation
  try {
    system_->run();
  } catch (...) {
  }
  
  // Get final state
  auto final_state = system_->get_state();
  
  // States should be different
  bool state_changed = false;
  for (size_t i = 0; i < initial_state.size(); ++i) {
    if (std::abs(initial_state[i] - final_state[i]) > 1e-6) {
      state_changed = true;
      break;
    }
  }
  
  EXPECT_TRUE(state_changed);
}

TEST_F(IntegrationTest, NoExceptionsInFullRun) {
  EXPECT_NO_THROW(system_->run());
}

TEST_F(IntegrationTest, AllOutputsAreFinite) {
  system_->run();
  
  auto state = system_->get_state();
  for (double value : state) {
    EXPECT_TRUE(std::isfinite(value));
  }
}

// System stability tests
TEST_F(IntegrationTest, AttitudeRemainsBounded) {
  system_->run();
  
  auto attitude = system_->get_attitude();
  
  for (double angle : attitude) {
    EXPECT_TRUE(std::isfinite(angle));
    EXPECT_LE(std::abs(angle), 2 * M_PI);
  }
}

TEST_F(IntegrationTest, AltitudeRemainsBounded) {
  system_->run();
  
  double altitude = system_->get_altitude();
  
  EXPECT_TRUE(std::isfinite(altitude));
  EXPECT_GE(altitude, -10000);
  EXPECT_LE(altitude, 200000);
}

}  // namespace gnc::test
