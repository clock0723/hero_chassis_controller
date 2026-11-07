#pragma once

#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>
#include <memory>

#include "ros_package_template/algorithm.hpp"

namespace hero_chassis_controller {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class HeroChassisController : public controller_interface::Controller<
                                  hardware_interface::EffortJointInterface> {
 public:
  /*!
   * Constructor.
   */
  HeroChassisController() = default;

  /*!
   * Destructor.
   */
  ~HeroChassisController() override = default;

  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool init(hardware_interface::EffortJointInterface* effort_joint_interface,
            ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

 private:
  /*!
   * ROS topic callback method.
   * @param message the received message.
   */
  void cmdVelCallback(const geometry_msgs::Twist& msg);

  hardware_interface::JointHandle left_front_joint_;
  hardware_interface::JointHandle right_front_joint_;
  hardware_interface::JointHandle left_back_joint_;
  hardware_interface::JointHandle right_back_joint_;

  control_toolbox::Pid left_front_pid_;
  control_toolbox::Pid right_front_pid_;
  control_toolbox::Pid left_back_pid_;
  control_toolbox::Pid right_back_pid_;

  //! ROS topic subscriber.
  ros::Subscriber cmd_vel_subscriber_;
  realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmd_vel_buffer_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>>
      target_left_front_wheel_velocity_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>>
      target_right_front_wheel_velocity_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>>
      target_left_back_wheel_velocity_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>>
      target_right_back_wheel_velocity_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>>
      current_left_front_wheel_velocity_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>>
      current_right_front_wheel_velocity_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>>
      current_left_back_wheel_velocity_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>>
      current_right_back_wheel_velocity_publisher_;
  //! Algorithm computation object.
  Algorithm algorithm_;
};

}  // namespace hero_chassis_controller
