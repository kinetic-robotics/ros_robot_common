#include <pluginlib/class_list_macros.h>


#include "friction_controller/friction_velocity_controller.h"
#include <robot_toolbox/tool.h>

namespace friction_controller
{
FrictionVelocityController::FrictionVelocityController()
{
}

void FrictionVelocityController::shotSpeedCallback(const robot_msgs::Float64StampedConstPtr& msg)
{
    targetShotSpeed_ = msg->result;
}

bool FrictionVelocityController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& node)
{
    /* 读取配置 */
    XmlRpc::XmlRpcValue jointList;
    node.getParam("joint", jointList);
    CONFIG_ASSERT("joint", jointList.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (auto iter = jointList.begin(); iter != jointList.end(); ++iter) {
        CONFIG_ASSERT("joint/" + iter->first, iter->second.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        joints_[iter->first].handle = hw->getHandle(iter->first);
        /* 初始化超级电容限制函数 */
        joints_[iter->first].speedFunction.reset(new robot_toolbox::FunctionTool(ros::NodeHandle(node, "joint/" + iter->first + "/function")));
        if (!joints_[iter->first].speedFunction->init()) {
            ROS_FATAL("Joint speed function init failed at joint %s.", iter->first.c_str());
            return false;
        }
        joints_[iter->first].statePublisher.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(node, "joint/" + iter->first + "/state", 1000));
        joints_[iter->first].statePublisher->msg_.p = 0;
        joints_[iter->first].statePublisher->msg_.i = 0;
        joints_[iter->first].statePublisher->msg_.d = 0;
        joints_[iter->first].statePublisher->msg_.i_clamp = 0;
        joints_[iter->first].statePublisher->msg_.antiwindup    = false;
        ROS_INFO("Parsed from parameters, joint = %s.", iter->first.c_str());
    }
    shotSpeedSubscriber_ = node.subscribe<robot_msgs::Float64Stamped>("speed/command", 1000, &FrictionVelocityController::shotSpeedCallback, this);
    ROS_INFO("Friction Velocity Controller started.");
    return true;
}

void FrictionVelocityController::update(const ros::Time& time, const ros::Duration& period)
{
    /* 发布话题 */
    for (auto iter = joints_.begin();iter != joints_.end();++iter) {
        double targetSpeed = iter->second.speedFunction->compute(targetShotSpeed_);
        double error = targetSpeed - iter->second.handle.getVelocity();
        iter->second.handle.setCommand(targetSpeed);
        if (iter->second.statePublisher && iter->second.statePublisher->trylock()) {
            iter->second.statePublisher->msg_.set_point     = targetSpeed;
            iter->second.statePublisher->msg_.process_value = iter->second.handle.getVelocity();
            iter->second.statePublisher->msg_.error         = error;
            iter->second.statePublisher->msg_.time_step     = period.toSec();
            iter->second.statePublisher->msg_.command       = targetSpeed;
            iter->second.statePublisher->msg_.header.stamp  = time;
            iter->second.statePublisher->msg_.header.seq++;
            iter->second.statePublisher->unlockAndPublish();
        }
    }
}

void FrictionVelocityController::starting(const ros::Time& time)
{
}

void FrictionVelocityController::stopping(const ros::Time& time)
{
    for (auto iter = joints_.begin();iter != joints_.end();++iter) {
        iter->second.handle.setCommand(0);
    }
}

}  // namespace fire_controller

PLUGINLIB_EXPORT_CLASS(friction_controller::FrictionVelocityController, controller_interface::ControllerBase);