#include <angles/angles.h>
#include <robot_toolbox/tool.h>

#include "rm_control/module/safety.h"
#include <std_msgs/Bool.h>

namespace rm_control
{
SafetyModule::SafetyModule(ros::NodeHandle& node, ros::NodeHandle& nodeParam)
    : ModuleInterface(node, nodeParam_), node_(node), nodeParam_(nodeParam)
{
}

bool SafetyModule::init()
{
    /* 读取配置并注册发布者 */
    CONFIG_ASSERT("safety/topic", nodeParam_.getParam("safety/topic", commandTopic_));
    /* 订阅电机信息 */
    commandPublisher_ = node_.advertise<std_msgs::Bool>(commandTopic_, 1000, false);
    return true;
}

void SafetyModule::getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, bool& isEnable, ros::Duration period)
{
    std_msgs::Bool msg;
    msg.data = isEnable;
    commandPublisher_.publish(msg);
}
}  // namespace rm_control