#ifndef FRICTION_CONTROLLER_FRICTION_VELOCITY_CONTROLLER_H_
#define FRICTION_CONTROLLER_FRICTION_VELOCITY_CONTROLLER_H_

#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <robot_msgs/Float64Stamped.h>
#include <robot_toolbox/function_tool.h>

namespace friction_controller
{
class FrictionVelocityController: public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
  private:
    typedef struct {
        hardware_interface::JointHandle handle;                                                                /* 电机句柄 */
        std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>> statePublisher; /* 状态发布者 */
        std::unique_ptr<robot_toolbox::FunctionTool> speedFunction;                                            /* 摩擦轮转速-射速函数 */
    } MotorInfo;
    std::map<std::string, MotorInfo> joints_; /* 电机信息 */
    double targetShotSpeed_ = 0;              /* 目标射速 */
    ros::Subscriber shotSpeedSubscriber_;     /* 射速订阅者 */

    /**
     * 射速话题回调
     * 
     * @param msg 消息
     */
    void shotSpeedCallback(const robot_msgs::Float64StampedConstPtr& msg);

  public:
    FrictionVelocityController();

    /**
     * ROS Control的Controller Init接口
     */
    bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& node);

    /**
     * ROS Control的Controller Update接口
     */
    void update(const ros::Time& time, const ros::Duration& period);

    /**
     * ROS Control的Controller Starting接口
     */
    void starting(const ros::Time& time);

    /**
     * ROS Control的Controller Stopping接口
     */
    void stopping(const ros::Time& time);
};

}  // namespace friction_controller

#endif