#include <gtest/gtest.h>

#include "ros_package_template/algorithm.hpp"

using namespace hero_chassis_controller;

TEST(Algorithm, calculateWheelVelocities) {
  Algorithm algorithm;
  algorithm.setWhellDate(0.07625, 0.4, 0.4);
  const ChassisVelocity chassis_velocity{0.3, -0.1, 0.5};

  const WheelVelocity wheel_velocities =
      algorithm.calculateWheelVelocities(chassis_velocity);

  EXPECT_NEAR(0.2 / 0.07625, wheel_velocities.left_front, 1e-10);
  EXPECT_NEAR(0.4 / 0.07625, wheel_velocities.right_front, 1e-10);
  EXPECT_NEAR(0.0, wheel_velocities.left_back, 1e-10);
  EXPECT_NEAR(0.6 / 0.07625, wheel_velocities.right_back, 1e-10);
}

TEST(Algorithm, calculateVelocityError) {
  Algorithm algorithm;
  const WheelVelocity target_velocity{5.0, 6.0, 7.0, 8.0};
  const WheelVelocity current_velocity{1.0, 2.0, 3.0, 4.0};

  const WheelVelocity velocity_error =
      algorithm.calculateVelocityError(target_velocity, current_velocity);

  EXPECT_DOUBLE_EQ(4.0, velocity_error.left_front);
  EXPECT_DOUBLE_EQ(4.0, velocity_error.right_front);
  EXPECT_DOUBLE_EQ(4.0, velocity_error.left_back);
  EXPECT_DOUBLE_EQ(4.0, velocity_error.right_back);
}
