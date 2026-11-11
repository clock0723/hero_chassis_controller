// Copyright 2026 clock0723
#include "ros_package_template/algorithm.hpp"

#include <cmath>

namespace hero_chassis_controller
{
void Algorithm::setWheelData(double wheel_r, double half_distance_x, double half_distance_y)
{
  wheel_r_ = wheel_r;
  half_distance_x_ = half_distance_x;
  half_distance_y_ = half_distance_y;
}

WheelVelocity Algorithm::calculateWheelVelocities(const ChassisVelocity& chassis_velocity) const
{
  const double turn_radius = (half_distance_x_ + half_distance_y_) / 2.0;
  const double vx = chassis_velocity.velocity_x;
  const double vy = chassis_velocity.velocity_y;
  const double wz = chassis_velocity.yaw_rate;

  WheelVelocity wheel_velocity;
  wheel_velocity.left_front = (vx - vy - turn_radius * wz) / wheel_r_;
  wheel_velocity.right_front = (vx + vy + turn_radius * wz) / wheel_r_;
  wheel_velocity.left_back = (vx + vy - turn_radius * wz) / wheel_r_;
  wheel_velocity.right_back = (vx - vy + turn_radius * wz) / wheel_r_;
  return wheel_velocity;
}

ChassisVelocity Algorithm::calculateChassisVelocity(const WheelVelocity& wheel_velocities) const
{
  const double turn_radius = (half_distance_x_ + half_distance_y_) / 2.0;
  const double lf = wheel_velocities.left_front;
  const double rf = wheel_velocities.right_front;
  const double lb = wheel_velocities.left_back;
  const double rb = wheel_velocities.right_back;

  ChassisVelocity chassis_velocity;
  chassis_velocity.velocity_x = wheel_r_ * (lf + rf + lb + rb) / 4.0;
  chassis_velocity.velocity_y = wheel_r_ * (-lf + rf + lb - rb) / 4.0;
  chassis_velocity.yaw_rate = wheel_r_ * (-lf + rf - lb + rb) / (4.0 * turn_radius);
  return chassis_velocity;
}

ChassisPose Algorithm::calculateOdometry(const ChassisVelocity& velocity, const ChassisPose& pose, double dt) const
{
  ChassisPose next_pose;
  next_pose.x =
      pose.x + (velocity.velocity_x * std::cos(pose.yaw) - velocity.velocity_y * std::sin(pose.yaw)) * dt;
  next_pose.y =
      pose.y + (velocity.velocity_x * std::sin(pose.yaw) + velocity.velocity_y * std::cos(pose.yaw)) * dt;
  next_pose.yaw = pose.yaw + velocity.yaw_rate * dt;
  return next_pose;
}

WheelVelocity Algorithm::calculateVelocityError(const WheelVelocity& target_velocity,
                                                const WheelVelocity& current_velocity) const
{
  WheelVelocity error;
  error.left_front = target_velocity.left_front - current_velocity.left_front;
  error.right_front = target_velocity.right_front - current_velocity.right_front;
  error.left_back = target_velocity.left_back - current_velocity.left_back;
  error.right_back = target_velocity.right_back - current_velocity.right_back;
  return error;
}

void Algorithm::publishWheelVelocity(realtime_tools::RealtimePublisher<std_msgs::Float64>& target_publisher,
                                     realtime_tools::RealtimePublisher<std_msgs::Float64>& current_publisher,
                                     double target_velocity, double current_velocity) const
{
  if (target_publisher.trylock())
  {
    target_publisher.msg_.data = target_velocity;
    target_publisher.unlockAndPublish();
  }

  if (current_publisher.trylock())
  {
    current_publisher.msg_.data = current_velocity;
    current_publisher.unlockAndPublish();
  }
}

}  // namespace hero_chassis_controller
