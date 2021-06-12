#include <robot_toolbox/tool.h>

#include "rm_control/module/safety.h"
#include <robot_msgs/BoolStamped.h>

namespace rm_control
{
SafetyModule::SafetyModule(ros::NodeHandle& node, ros::NodeHandle& nodeParam)
    : ModuleInterface(node, nodeParam_), node_(node), nodeParam_(nodeParam)
{
}

void SafetyModule::rcOnlineCallback(const robot_msgs::BoolStampedConstPtr& msg)
{
    isRCOnline_ = msg->result;
}

bool SafetyModule::init()
{
    /* 遥控器离线自动切断输出 */
    CONFIG_ASSERT("safety/rc_online_topic", nodeParam_.getParam("safety/rc_online_topic", rcOnlineTopic_));
    /* 读取配置并注册发布者 */
    CONFIG_ASSERT("safety/command_topic", nodeParam_.getParam("safety/command_topic", commandTopic_));
    /* 订阅和发布 */
    commandPublisher_ = node_.advertise<robot_msgs::BoolStamped>(commandTopic_, 1000, false);
    rcOnlineSubscriber_ = node_.subscribe<robot_msgs::BoolStamped>(rcOnlineTopic_, 1000, &SafetyModule::rcOnlineCallback, this);
    return true;
}

void SafetyModule::getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, ShotStatus& shotStatus, bool& isEnable, ros::Duration period, std::map<std::string, bool>& enableModules)
{
    robot_msgs::BoolStamped msg;
    msg.header.seq = commandSeq_++;
    msg.header.stamp = ros::Time::now();
    msg.result = isEnable || !isRCOnline_;
    commandPublisher_.publish(msg);
}
}  // namespace rm_control