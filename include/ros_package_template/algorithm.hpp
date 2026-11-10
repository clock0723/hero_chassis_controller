#pragma once

#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>

namespace hero_chassis_controller {

struct ChassisVelocity {
  double velocity_x;
  double velocity_y;
  double raw;
};

struct ChassisPose {
  double x;
  double y;
  double yaw;
};

struct WheelVelocity {
  double left_front;
  double right_front;
  double left_back;
  double right_back;
};

/*!
 * Class containing the algorithmic part of the package.
 */
class Algorithm {
 public:
  void setWhellDate(double wheel_r, double half_distance_x,
                    double half_distance_y);
  WheelVelocity calculateWheelVelocities(
      const ChassisVelocity& chassis_velocity) const;
  ChassisVelocity calculateChassisVelocity(
      const WheelVelocity& wheel_velocities) const;
  ChassisPose calculateOdometry(const ChassisVelocity& velocity,
                                const ChassisPose& pose, double dt) const;
  WheelVelocity calculateVelocityError(
      const WheelVelocity& target_velocity,
      const WheelVelocity& current_velocity) const;
  void publishWheelVelocity(
      realtime_tools::RealtimePublisher<std_msgs::Float64>& target_publisher,
      realtime_tools::RealtimePublisher<std_msgs::Float64>&
          current_publisher,
      double target_velocity, double current_velocity) const;

 private:
  double wheel_r_{0.07625};
  double half_distance_x_{0.4};
  double half_distance_y_{0.4};
};

}  // namespace hero_chassis_controller
