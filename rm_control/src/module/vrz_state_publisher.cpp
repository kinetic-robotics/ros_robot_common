#include <robot_toolbox/tool.h>

#include "rm_control/module/vrz_state_publisher.h"
#include <robot_msgs/BoolStamped.h>

namespace rm_control
{
VRZStatePublisherModule::VRZStatePublisherModule(ros::NodeHandle& node, ros::NodeHandle& nodeParam)
    : ModuleInterface(node, nodeParam_), node_(node), nodeParam_(nodeParam)
{
}

bool VRZStatePublisherModule::init()
{
    /* 注册发布者 */
    vrzStatusPublisher_ = nodeParam_.advertise<robot_msgs::BoolStamped>("vrz_state_publisher/status", 1000, false);
    return true;
}

void VRZStatePublisherModule::getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, ShotStatus& shotStatus, bool& isEnable, ros::Duration period, std::map<std::string, bool>& enableModules)
{
    robot_msgs::BoolStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.seq = vrzStatusPublisherSeq_++;
    msg.result = isEnable;
    vrzStatusPublisher_.publish(msg);
}
}  // namespace rm_control