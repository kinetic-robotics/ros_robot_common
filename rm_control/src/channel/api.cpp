#include <robot_toolbox/tool.h>

#include "rm_control/channel/api.h"

namespace rm_control
{
ApiChannel::ApiChannel(ros::NodeHandle& node, ros::NodeHandle& nodeParam)
    : ChannelInterface(node, nodeParam_), node_(node), nodeParam_(nodeParam)
{
}

void ApiChannel::cmdVelCallback(const geometry_msgs::TwistConstPtr& msg)
{
    vx_  = msg->linear.x;
    vy_  = msg->linear.y;
    vrz_ = msg->angular.z;
}

void ApiChannel::pitchCallback(const robot_msgs::Float64StampedConstPtr& msg)
{
    pitchAngleDelta_ = msg->result;
}

void ApiChannel::yawCallback(const robot_msgs::Float64StampedConstPtr& msg)
{
    yawAngleDelta_ = msg->result;
}

bool ApiChannel::init()
{
    /* 订阅 */
    cmdVelSubscriber_ = nodeParam_.subscribe<geometry_msgs::Twist>("api/cmd_vel", 1000, &ApiChannel::cmdVelCallback, this);
    pitchSubscriber_  = nodeParam_.subscribe<robot_msgs::Float64Stamped>("api/pitch_angle", 1000, &ApiChannel::pitchCallback, this);
    yawSubscriber_    = nodeParam_.subscribe<robot_msgs::Float64Stamped>("api/yaw_angle", 1000, &ApiChannel::yawCallback, this);
    /* 发布 */
    return true;
}

void ApiChannel::getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, ShotStatus& shotStatus, ros::Duration period, std::map<std::string, bool>& enableModules)
{
    vx += vx_;
    vy += vy_;
    vrz += vrz_;
    yawAngle += yawAngleDelta_;
    yawAngleDelta_ = 0;
    pitchAngle += pitchAngleDelta_;
    pitchAngleDelta_ = 0;
}

}  // namespace rm_control