#include <gtest/gtest.h>

#include <cmath>

#include "ros_package_template/algorithm.hpp"

using hero_chassis_controller::Algorithm;
using hero_chassis_controller::ChassisPose;
using hero_chassis_controller::ChassisVelocity;
using hero_chassis_controller::WheelVelocity;

namespace {

constexpr double kWheelRadius = 0.07625;

Algorithm MakeAlgorithm() {
  Algorithm algorithm;
  algorithm.setWhellDate(kWheelRadius, 0.4, 0.4);
  return algorithm;
}

}  // namespace

// 逆运动学：前进时四轮同速，横移时对角两轮同向、另一对反向
TEST(AlgorithmTest, InverseKinematics) {
  const Algorithm algorithm = MakeAlgorithm();

  const WheelVelocity forward =
      algorithm.calculateWheelVelocities({0.5, 0.0, 0.0});
  EXPECT_NEAR(0.5 / kWheelRadius, forward.left_front, 1e-9);
  EXPECT_NEAR(forward.left_front, forward.right_front, 1e-9);
  EXPECT_NEAR(forward.left_front, forward.left_back, 1e-9);
  EXPECT_NEAR(forward.left_front, forward.right_back, 1e-9);

  const WheelVelocity strafe =
      algorithm.calculateWheelVelocities({0.0, 0.2, 0.0});
  EXPECT_NEAR(-0.2 / kWheelRadius, strafe.left_front, 1e-9);
  EXPECT_NEAR(0.2 / kWheelRadius, strafe.right_front, 1e-9);
  EXPECT_NEAR(0.2 / kWheelRadius, strafe.left_back, 1e-9);
  EXPECT_NEAR(-0.2 / kWheelRadius, strafe.right_back, 1e-9);
}

// 里程计：朝 +X 走 x 增加，朝 +Y 走 y 增加，自转只改变 yaw
TEST(AlgorithmTest, Odometry) {
  const Algorithm algorithm = MakeAlgorithm();
  const ChassisVelocity forward{0.5, 0.0, 0.0};

  const ChassisPose along_x =
      algorithm.calculateOdometry(forward, {0.0, 0.0, 0.0}, 0.1);
  EXPECT_NEAR(0.05, along_x.x, 1e-9);
  EXPECT_NEAR(0.0, along_x.y, 1e-9);

  const ChassisPose along_y =
      algorithm.calculateOdometry(forward, {0.0, 0.0, M_PI_2}, 0.1);
  EXPECT_NEAR(0.0, along_y.x, 1e-9);
  EXPECT_NEAR(0.05, along_y.y, 1e-9);

  const ChassisPose turning =
      algorithm.calculateOdometry({0.0, 0.0, 0.5}, {1.0, 2.0, 0.0}, 0.2);
  EXPECT_NEAR(1.0, turning.x, 1e-9);
  EXPECT_NEAR(2.0, turning.y, 1e-9);
  EXPECT_NEAR(0.1, turning.yaw, 1e-9);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
