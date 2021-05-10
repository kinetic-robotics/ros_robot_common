#include <pluginlib/class_list_macros.h>


#include "friction_controller/friction_controller.h"
#include <robot_toolbox/tool.h>

namespace friction_controller
{
FrictionController::FrictionController()
{
}

void FrictionController::shotSpeedCallback(const robot_msgs::Float64StampedConstPtr& msg)
{
    targetShotSpeed_ = msg->result;
}

bool FrictionController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& node)
{
    /* 读取配置 */
    XmlRpc::XmlRpcValue jointList;
    node.getParam("joint", jointList);
    CONFIG_ASSERT("joint", jointList.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (auto iter = jointList.begin(); iter != jointList.end(); iter++) {
        CONFIG_ASSERT("joint/" + iter->first, iter->second.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        joints_[iter->first].handle = hw->getHandle(iter->first);
        /* 初始化超级电容限制函数 */
        joints_[iter->first].speedFunction.reset(new robot_toolbox::FunctionTool(ros::NodeHandle(node, "joint/" + iter->first + "/function"), ros::NodeHandle(node, "joint/" + iter->first + "/function")));
        if (!joints_[iter->first].speedFunction->init()) {
            ROS_FATAL("Joint speed function init failed at joint %s.", iter->first.c_str());
            return false;
        }
        if (!joints_[iter->first].pid.init(ros::NodeHandle(node, "joint/" + iter->first + "/pid"))) {
            ROS_FATAL("PID Controller init failed at joint: %s.", iter->first.c_str());
            return false;
        }
        joints_[iter->first].statePublisher.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(node, "joint/" + iter->first + "/state", 1000));
        ROS_INFO("Parsed from parameters, joint = %s.", iter->first.c_str());
    }
    shotSpeedSubscriber_ = node.subscribe<robot_msgs::Float64Stamped>("speed/command", 1000, &FrictionController::shotSpeedCallback, this);
    ROS_INFO("Friction Controller started.");
    return true;
}

void FrictionController::update(const ros::Time& time, const ros::Duration& period)
{
    /* PID计算与发布话题 */
    for (auto iter = joints_.begin();iter != joints_.end();iter++) {
        double targetSpeed = iter->second.speedFunction->compute(targetShotSpeed_);
        double error = targetSpeed - iter->second.handle.getVelocity();
        double output = iter->second.pid.computeCommand(error, period);
        iter->second.handle.setCommand(output);
        if (iter->second.statePublisher && iter->second.statePublisher->trylock()) {
            double _;
            bool antiwindup;
            iter->second.pid.getGains(
                iter->second.statePublisher->msg_.p, iter->second.statePublisher->msg_.i, iter->second.statePublisher->msg_.d,
                iter->second.statePublisher->msg_.i_clamp, _, antiwindup);
            iter->second.statePublisher->msg_.antiwindup    = static_cast<char>(antiwindup);
            iter->second.statePublisher->msg_.set_point     = targetSpeed;
            iter->second.statePublisher->msg_.process_value = iter->second.handle.getVelocity();
            iter->second.statePublisher->msg_.error         = error;
            iter->second.statePublisher->msg_.time_step     = period.toSec();
            iter->second.statePublisher->msg_.command       = output;
            iter->second.statePublisher->msg_.header.stamp  = time;
            iter->second.statePublisher->msg_.header.seq++;
            iter->second.statePublisher->unlockAndPublish();
        }
    }
}

void FrictionController::starting(const ros::Time& time)
{
}

void FrictionController::stopping(const ros::Time& time)
{
    for (auto iter = joints_.begin();iter != joints_.end();iter++) {
        iter->second.handle.setCommand(0);
    }
}

}  // namespace fire_controller

PLUGINLIB_EXPORT_CLASS(friction_controller::FrictionController, controller_interface::ControllerBase);