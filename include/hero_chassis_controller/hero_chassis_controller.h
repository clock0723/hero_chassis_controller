#ifndef HREO_CHASSIS_CONTOROLLER_H
#define HREO_CHASSIS_CONTOROLLER_H



// 包含的库，有控制接口，硬件接口，插件的头文件组成
#include<controller_interface/controller.h>
#include<hardware_interface/joint_command_interface.h>

// 命名空间，
namespace hero_chassis_controller {
//继承控制类
class HeroChassisController :public controller_interface::Controller<hardware_interface::VelocityJointInterface>{
public:
//构造函数
HeroChassisController()=default;
~HeroChassisController() override=default;
//init函数
bool init(hardware_interface::VelocityJointInterface *velocity_joint_interface,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
//update函数
void update(const ros::Time &time,ros::Duration &perio);
}
//定义JointHandle
hardware_interface::JointHandle front_left_joint_,front_right_joint_, back_left_joint_, back_right_joint_;
 
} /* namespace */
#endif