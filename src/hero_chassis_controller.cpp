#include<hero_chassis_controller.h>
#include<pluginlib/class_list_macros.h>

namespace hero_chassis_controller{
    bool HeroChassisController::init(hardware_interface::VelocityJointInterface *velocity_joint_interface,ros::NodeHandle &root_nh,ros::NodeHandle &controller_nh){
//调用接口类成员函数给关节赋值
  ront_left_joint_=velocity_joint_interface->getHandle("front_left_wheel_joint");
  front_right_joint_=velocity_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ =velocity_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ =velocity_joint_interface->getHandle("right_back_wheel_joint");

  return true;
}
 void HeroChassisController::update(const ros::Time &time,ros::Duration &period ){

 }
 PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,controller_interface::ControllerBase)
}