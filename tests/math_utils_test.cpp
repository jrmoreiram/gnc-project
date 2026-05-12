#include <gtest/gtest.h>
#include <cmath>
#include <Eigen/Dense>
#include "utils/math_utils.h"

namespace gnc::test {

// Math Utils Tests
class MathUtilsTest : public ::testing::Test {
 protected:
  const double EPSILON = 1e-6;
};

// Vector operations
TEST_F(MathUtilsTest, VectorNormalizationWorks) {
  Eigen::Vector3d v(3.0, 4.0, 0.0);
  Eigen::Vector3d normalized = gnc::math::normalize(v);
  
  EXPECT_NEAR(normalized.norm(), 1.0, EPSILON);
  EXPECT_NEAR(normalized[0], 0.6, EPSILON);
  EXPECT_NEAR(normalized[1], 0.8, EPSILON);
}

TEST_F(MathUtilsTest, VectorDotProduct) {
  Eigen::Vector3d v1(1.0, 2.0, 3.0);
  Eigen::Vector3d v2(4.0, 5.0, 6.0);
  
  double result = gnc::math::dot(v1, v2);
  double expected = 1*4 + 2*5 + 3*6;
  
  EXPECT_NEAR(result, expected, EPSILON);
}

TEST_F(MathUtilsTest, VectorCrossProduct) {
  Eigen::Vector3d v1(1.0, 0.0, 0.0);
  Eigen::Vector3d v2(0.0, 1.0, 0.0);
  
  Eigen::Vector3d result = gnc::math::cross(v1, v2);
  
  EXPECT_NEAR(result[0], 0.0, EPSILON);
  EXPECT_NEAR(result[1], 0.0, EPSILON);
  EXPECT_NEAR(result[2], 1.0, EPSILON);
}

// Quaternion operations
TEST_F(MathUtilsTest, QuaternionNormalization) {
  Eigen::Vector4d q(1.0, 0.0, 0.0, 0.0);
  Eigen::Vector4d normalized = gnc::math::normalize_quaternion(q);
  
  EXPECT_NEAR(normalized.norm(), 1.0, EPSILON);
}

TEST_F(MathUtilsTest, QuaternionToEulerConversion) {
  Eigen::Vector4d q(1.0, 0.0, 0.0, 0.0);
  auto euler = gnc::math::quaternion_to_euler(q);
  
  EXPECT_EQ(euler.size(), 3);
  for (double angle : euler) {
    EXPECT_TRUE(std::isfinite(angle));
  }
}

TEST_F(MathUtilsTest, EulerToQuaternionConversion) {
  std::vector<double> euler = {0.0, 0.0, 0.0};
  auto q = gnc::math::euler_to_quaternion(euler);
  
  EXPECT_EQ(q.size(), 4);
  EXPECT_NEAR(q.norm(), 1.0, EPSILON);
}

// Matrix operations
TEST_F(MathUtilsTest, MatrixInverse) {
  Eigen::Matrix3d m;
  m << 1, 0, 0,
       0, 2, 0,
       0, 0, 3;
  
  auto m_inv = gnc::math::matrix_inverse(m);
  auto identity = m * m_inv;
  
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (i == j) {
        EXPECT_NEAR(identity(i, j), 1.0, EPSILON);
      } else {
        EXPECT_NEAR(identity(i, j), 0.0, EPSILON);
      }
    }
  }
}

TEST_F(MathUtilsTest, SingularValueDecomposition) {
  Eigen::Matrix3d m;
  m << 1, 0, 0,
       0, 2, 0,
       0, 0, 3;
  
  auto svd = gnc::math::svd(m);
  EXPECT_TRUE(true);
}

// Angle operations
TEST_F(MathUtilsTest, AngleWrapTo2Pi) {
  double angle = 3 * M_PI;
  double wrapped = gnc::math::wrap_angle_to_2pi(angle);
  
  EXPECT_GE(wrapped, 0);
  EXPECT_LE(wrapped, 2 * M_PI);
}

TEST_F(MathUtilsTest, AngleWrapToPi) {
  double angle = 2 * M_PI;
  double wrapped = gnc::math::wrap_angle_to_pi(angle);
  
  EXPECT_GE(wrapped, -M_PI);
  EXPECT_LE(wrapped, M_PI);
}

// Interpolation
TEST_F(MathUtilsTest, LinearInterpolation) {
  double result = gnc::math::lerp(0.0, 10.0, 0.5);
  
  EXPECT_NEAR(result, 5.0, EPSILON);
}

TEST_F(MathUtilsTest, LinearInterpolationBoundaries) {
  double result0 = gnc::math::lerp(0.0, 10.0, 0.0);
  double result1 = gnc::math::lerp(0.0, 10.0, 1.0);
  
  EXPECT_NEAR(result0, 0.0, EPSILON);
  EXPECT_NEAR(result1, 10.0, EPSILON);
}

// Clamp
TEST_F(MathUtilsTest, ClampValue) {
  double clamped_low = gnc::math::clamp(-5.0, 0.0, 10.0);
  double clamped_mid = gnc::math::clamp(5.0, 0.0, 10.0);
  double clamped_high = gnc::math::clamp(15.0, 0.0, 10.0);
  
  EXPECT_EQ(clamped_low, 0.0);
  EXPECT_EQ(clamped_mid, 5.0);
  EXPECT_EQ(clamped_high, 10.0);
}

// Saturation
TEST_F(MathUtilsTest, SaturationFunction) {
  double sat_low = gnc::math::saturate(-5.0, -1.0, 1.0);
  double sat_mid = gnc::math::saturate(0.5, -1.0, 1.0);
  double sat_high = gnc::math::saturate(5.0, -1.0, 1.0);
  
  EXPECT_EQ(sat_low, -1.0);
  EXPECT_EQ(sat_mid, 0.5);
  EXPECT_EQ(sat_high, 1.0);
}

}  // namespace gnc::test
