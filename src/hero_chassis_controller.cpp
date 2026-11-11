// Copyright 2026 clock0723
#include "ros_package_template/hero_chassis_controller.hpp"

#include <cmath>
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller
{
bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                 ros::NodeHandle& /*root_nh*/, ros::NodeHandle& controller_nh)
{
  double wheel_r = 0.07625;
  double half_distance_x = 0.4;
  double half_distance_y = 0.4;
  controller_nh.param("wheel_r", wheel_r, wheel_r);
  controller_nh.param("half_distance_x", half_distance_x, half_distance_x);
  controller_nh.param("half_distance_y", half_distance_y, half_distance_y);
  controller_nh.param("use_global_velocity", global_cmd_, false);
  controller_nh.param("global_frame", frame_, frame_);
  algorithm_.setWheelData(wheel_r, half_distance_x, half_distance_y);

  left_front_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
  right_front_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
  left_back_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  right_back_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

  ros::NodeHandle left_front_pid_node(controller_nh, "left_front_wheel_joint_pid");
  ros::NodeHandle right_front_pid_node(controller_nh, "right_front_wheel_joint_pid");
  ros::NodeHandle left_back_pid_node(controller_nh, "left_back_wheel_joint_pid");
  ros::NodeHandle right_back_pid_node(controller_nh, "right_back_wheel_joint_pid");
  left_front_pid_.init(left_front_pid_node);
  right_front_pid_.init(right_front_pid_node);
  left_back_pid_.init(left_back_pid_node);
  right_back_pid_.init(right_back_pid_node);

  cmd_vel_subscriber_ =
      controller_nh.subscribe("/cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);
  odom_publisher_ = controller_nh.advertise<nav_msgs::Odometry>("/odom", 50);
  cmd_vel_buffer_.writeFromNonRT(geometry_msgs::Twist());

  target_left_front_wheel_velocity_publisher_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Float64>(controller_nh, "wheel_velocity/target/left_front", 1));
  target_right_front_wheel_velocity_publisher_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Float64>(controller_nh, "wheel_velocity/target/right_front", 1));
  target_left_back_wheel_velocity_publisher_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Float64>(controller_nh, "wheel_velocity/target/left_back", 1));
  target_right_back_wheel_velocity_publisher_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Float64>(controller_nh, "wheel_velocity/target/right_back", 1));
  current_left_front_wheel_velocity_publisher_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Float64>(controller_nh, "wheel_velocity/current/left_front", 1));
  current_right_front_wheel_velocity_publisher_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Float64>(controller_nh, "wheel_velocity/current/right_front", 1));
  current_left_back_wheel_velocity_publisher_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Float64>(controller_nh, "wheel_velocity/current/left_back", 1));
  current_right_back_wheel_velocity_publisher_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Float64>(controller_nh, "wheel_velocity/current/right_back", 1));

  return true;
}

void HeroChassisController::starting(const ros::Time& /*time*/)
{
  left_front_pid_.reset();
  right_front_pid_.reset();
  left_back_pid_.reset();
  right_back_pid_.reset();
  pose_ = {0.0, 0.0, 0.0};
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  const geometry_msgs::Twist cmd = *cmd_vel_buffer_.readFromRT();
  ChassisVelocity cmd_vel;
  cmd_vel.velocity_x = cmd.linear.x;
  cmd_vel.velocity_y = cmd.linear.y;
  cmd_vel.yaw_rate = cmd.angular.z;
  const WheelVelocity target = algorithm_.calculateWheelVelocities(cmd_vel);
  WheelVelocity current;
  current.left_front = left_front_joint_.getVelocity();
  current.right_front = right_front_joint_.getVelocity();
  current.left_back = left_back_joint_.getVelocity();
  current.right_back = right_back_joint_.getVelocity();
  const WheelVelocity error = algorithm_.calculateVelocityError(target, current);

  algorithm_.publishWheelVelocity(*target_left_front_wheel_velocity_publisher_,
                                  *current_left_front_wheel_velocity_publisher_, target.left_front, current.left_front);
  algorithm_.publishWheelVelocity(*target_right_front_wheel_velocity_publisher_,
                                  *current_right_front_wheel_velocity_publisher_, target.right_front,
                                  current.right_front);
  algorithm_.publishWheelVelocity(*target_left_back_wheel_velocity_publisher_,
                                  *current_left_back_wheel_velocity_publisher_, target.left_back, current.left_back);
  algorithm_.publishWheelVelocity(*target_right_back_wheel_velocity_publisher_,
                                  *current_right_back_wheel_velocity_publisher_, target.right_back, current.right_back);

  const ChassisVelocity velocity = algorithm_.calculateChassisVelocity(current);
  pose_ = algorithm_.calculateOdometry(velocity, pose_, period.toSec());

  geometry_msgs::Quaternion orientation;
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = std::sin(pose_.yaw / 2.0);
  orientation.w = std::cos(pose_.yaw / 2.0);

  nav_msgs::Odometry odom;
  odom.header.stamp = time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = pose_.x;
  odom.pose.pose.position.y = pose_.y;
  odom.pose.pose.orientation = orientation;
  odom.twist.twist.linear.x = velocity.velocity_x;
  odom.twist.twist.linear.y = velocity.velocity_y;
  odom.twist.twist.angular.z = velocity.yaw_rate;
  odom_publisher_.publish(odom);

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose_.x, pose_.y, 0.0));
  tf::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, pose_.yaw);
  transform.setRotation(rotation);
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform, time, "odom", "base_link"));

  left_front_joint_.setCommand(left_front_pid_.computeCommand(error.left_front, period));
  right_front_joint_.setCommand(right_front_pid_.computeCommand(error.right_front, period));
  left_back_joint_.setCommand(left_back_pid_.computeCommand(error.left_back, period));
  right_back_joint_.setCommand(right_back_pid_.computeCommand(error.right_back, period));
}

void HeroChassisController::stopping(const ros::Time& /*time*/)
{
  left_front_joint_.setCommand(0.0);
  right_front_joint_.setCommand(0.0);
  left_back_joint_.setCommand(0.0);
  right_back_joint_.setCommand(0.0);
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist& msg)
{
  geometry_msgs::Twist cmd = msg;

  if (global_cmd_)
  {
    const tf::Stamped<tf::Vector3> linear(tf::Vector3(msg.linear.x, msg.linear.y, msg.linear.z), ros::Time(0), frame_);
    const tf::Stamped<tf::Vector3> angular(tf::Vector3(msg.angular.x, msg.angular.y, msg.angular.z), ros::Time(0),
                                           frame_);
    tf::Stamped<tf::Vector3> base_linear;
    tf::Stamped<tf::Vector3> base_angular;

    try
    {
      tf_listener_.transformVector("base_link", linear, base_linear);
      tf_listener_.transformVector("base_link", angular, base_angular);
    }
    catch (const tf::TransformException&)
    {
      cmd_vel_buffer_.writeFromNonRT(geometry_msgs::Twist());
      return;
    }

    cmd.linear.x = base_linear.x();
    cmd.linear.y = base_linear.y();
    cmd.linear.z = base_linear.z();
    cmd.angular.x = base_angular.x();
    cmd.angular.y = base_angular.y();
    cmd.angular.z = base_angular.z();
  }

  cmd_vel_buffer_.writeFromNonRT(cmd);
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
