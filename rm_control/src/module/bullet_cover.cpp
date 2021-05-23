#include <robot_toolbox/tool.h>
#include <robot_msgs/BoolStamped.h>

#include "rm_control/module/bullet_cover.h"

namespace rm_control
{
BulletCoverModule::BulletCoverModule(ros::NodeHandle& node, ros::NodeHandle& nodeParam)
    : ModuleInterface(node, nodeParam_), node_(node), nodeParam_(nodeParam)
{
}

bool BulletCoverModule::init()
{
    /* 读取配置 */
    CONFIG_ASSERT("bullet_cover/command_topic", nodeParam_.getParam("bullet_cover/command_topic", commandTopic_));
    commandTopicPublisher_ = node_.advertise<robot_msgs::BoolStamped>(commandTopic_, 1000, false);
    return true;
}

void BulletCoverModule::getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, ShotStatus& shotStatus, bool& isEnable, ros::Duration period)
{
    robot_msgs::BoolStamped msg;
    msg.header.seq++;
    msg.header.stamp = ros::Time::now();
    msg.result = isEnable;
    commandTopicPublisher_.publish(msg);
}
}  // namespace rm_control