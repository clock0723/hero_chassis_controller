#include "ros_package_template/hero_chassis_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {

bool HeroChassisController::init(
    hardware_interface::EffortJointInterface* effort_joint_interface,
    ros::NodeHandle& /*root_nh*/, ros::NodeHandle& controller_nh) {
  double wheel_r = 0.07625;
  double half_distance_x = 0.4;
  double half_distance_y = 0.4;
  controller_nh.param("wheel_r", wheel_r, wheel_r);
  controller_nh.param("half_distance_x", half_distance_x, half_distance_x);
  controller_nh.param("half_distance_y", half_distance_y, half_distance_y);
  algorithm_.setWhellDate(wheel_r, half_distance_x, half_distance_y);

  left_front_joint_ =
      effort_joint_interface->getHandle("left_front_wheel_joint");
  right_front_joint_ =
      effort_joint_interface->getHandle("right_front_wheel_joint");
  left_back_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  right_back_joint_ =
      effort_joint_interface->getHandle("right_back_wheel_joint");

  ros::NodeHandle left_front_pid_node(controller_nh,
                                      "left_front_wheel_joint_pid");
  ros::NodeHandle right_front_pid_node(controller_nh,
                                       "right_front_wheel_joint_pid");
  ros::NodeHandle left_back_pid_node(controller_nh,
                                     "left_back_wheel_joint_pid");
  ros::NodeHandle right_back_pid_node(controller_nh,
                                      "right_back_wheel_joint_pid");
  left_front_pid_.init(left_front_pid_node);
  right_front_pid_.init(right_front_pid_node);
  left_back_pid_.init(left_back_pid_node);
    right_back_pid_.init(right_back_pid_node);
  
  cmd_vel_subscriber_ = controller_nh.subscribe(
      "cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);
  cmd_vel_buffer_.writeFromNonRT(geometry_msgs::Twist());

  target_left_front_wheel_velocity_publisher_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Float64>(
          controller_nh, "wheel_velocity/target/left_front", 1));
  target_right_front_wheel_velocity_publisher_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Float64>(
          controller_nh, "wheel_velocity/target/right_front", 1));
  target_left_back_wheel_velocity_publisher_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Float64>(
          controller_nh, "wheel_velocity/target/left_back", 1));
  target_right_back_wheel_velocity_publisher_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Float64>(
          controller_nh, "wheel_velocity/target/right_back", 1));
  current_left_front_wheel_velocity_publisher_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Float64>(
          controller_nh, "wheel_velocity/current/left_front", 1));
  current_right_front_wheel_velocity_publisher_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Float64>(
          controller_nh, "wheel_velocity/current/right_front", 1));
  current_left_back_wheel_velocity_publisher_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Float64>(
          controller_nh, "wheel_velocity/current/left_back", 1));
  current_right_back_wheel_velocity_publisher_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Float64>(
          controller_nh, "wheel_velocity/current/right_back", 1));

  return true;
}

void HeroChassisController::starting(const ros::Time& /*time*/) {
  left_front_pid_.reset();
  right_front_pid_.reset();
  left_back_pid_.reset();
  right_back_pid_.reset();
}

void HeroChassisController::update(const ros::Time& /*time*/,
                                   const ros::Duration& period) {
  const geometry_msgs::Twist chassis_velocity_command =
      *cmd_vel_buffer_.readFromRT();
  const ChassisVelocity chassis_velocity = {
      chassis_velocity_command.linear.x, chassis_velocity_command.linear.y,
      chassis_velocity_command.angular.z};
  const WheelVelocity target_wheel_velocities =
      algorithm_.calculateWheelVelocities(chassis_velocity);
  const WheelVelocity current_wheel_velocities = {
      left_front_joint_.getVelocity(), right_front_joint_.getVelocity(),
      left_back_joint_.getVelocity(), right_back_joint_.getVelocity()};
  const WheelVelocity velocity_errors =
      algorithm_.calculateVelocityError(target_wheel_velocities,
                                        current_wheel_velocities);

  algorithm_.publishWheelVelocity(
      *target_left_front_wheel_velocity_publisher_,
      *current_left_front_wheel_velocity_publisher_,
      target_wheel_velocities.left_front, current_wheel_velocities.left_front);
  algorithm_.publishWheelVelocity(
      *target_right_front_wheel_velocity_publisher_,
      *current_right_front_wheel_velocity_publisher_,
      target_wheel_velocities.right_front,
      current_wheel_velocities.right_front);
  algorithm_.publishWheelVelocity(
      *target_left_back_wheel_velocity_publisher_,
      *current_left_back_wheel_velocity_publisher_,
      target_wheel_velocities.left_back, current_wheel_velocities.left_back);
  algorithm_.publishWheelVelocity(
      *target_right_back_wheel_velocity_publisher_,
      *current_right_back_wheel_velocity_publisher_,
      target_wheel_velocities.right_back, current_wheel_velocities.right_back);

  left_front_joint_.setCommand(
      left_front_pid_.computeCommand(velocity_errors.left_front, period));
  right_front_joint_.setCommand(
      right_front_pid_.computeCommand(velocity_errors.right_front, period));
  left_back_joint_.setCommand(
      left_back_pid_.computeCommand(velocity_errors.left_back, period));
  right_back_joint_.setCommand(
      right_back_pid_.computeCommand(velocity_errors.right_back, period));
}

void HeroChassisController::stopping(const ros::Time& /*time*/) {
  left_front_joint_.setCommand(0.0);
  right_front_joint_.setCommand(0.0);
  left_back_joint_.setCommand(0.0);
  right_back_joint_.setCommand(0.0);
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist& msg) {
  cmd_vel_buffer_.writeFromNonRT(msg);
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)
