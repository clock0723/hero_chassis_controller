#include "ros_package_template/algorithm.hpp"

namespace hero_chassis_controller {

void Algorithm::setWhellDate(double wheel_r, double half_distance_x,
                            double half_distance_y) {
  wheel_r_ = wheel_r;
  half_distance_x_ = half_distance_x;
  half_distance_y_ = half_distance_y;
}

WheelVelocity Algorithm::calculateWheelVelocities(
    const ChassisVelocity& chassis_velocity) const {
  const double turn_radius = (half_distance_x_ + half_distance_y_) / 2.0;
  return {(chassis_velocity.velocity_x - chassis_velocity.velocity_y -
           turn_radius * chassis_velocity.raw) /
              wheel_r_,
          (chassis_velocity.velocity_x + chassis_velocity.velocity_y +
           turn_radius * chassis_velocity.raw) /
              wheel_r_,
          (chassis_velocity.velocity_x + chassis_velocity.velocity_y -
           turn_radius * chassis_velocity.raw) /
              wheel_r_,
          (chassis_velocity.velocity_x - chassis_velocity.velocity_y +
           turn_radius * chassis_velocity.raw) /
              wheel_r_};
}

WheelVelocity Algorithm::calculateVelocityError(
    const WheelVelocity& target_velocity,
    const WheelVelocity& current_velocity) const {
  return {target_velocity.left_front - current_velocity.left_front,
          target_velocity.right_front - current_velocity.right_front,
          target_velocity.left_back - current_velocity.left_back,
          target_velocity.right_back - current_velocity.right_back};
}

void Algorithm::publishWheelVelocity(
    realtime_tools::RealtimePublisher<std_msgs::Float64>& target_publisher,
    realtime_tools::RealtimePublisher<std_msgs::Float64>& current_publisher,
    double target_velocity, double current_velocity) const {
  if (target_publisher.trylock()) {
    target_publisher.msg_.data = target_velocity;
    target_publisher.unlockAndPublish();
  }

  if (current_publisher.trylock()) {
    current_publisher.msg_.data = current_velocity;
    current_publisher.unlockAndPublish();
  }
}

}  // namespace hero_chassis_controller
