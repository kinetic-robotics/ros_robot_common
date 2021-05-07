#include <geometry_msgs/Twist.h>
#include <robot_toolbox/tool.h>
#include <std_msgs/Float64.h>

#include "rm_control/channel/channel.h"
#include "rm_control/channel/channel_manager.h"
#include "rm_control/channel/joystick.h"
#include "rm_control/channel/keyboard.h"
#include "rm_control/channel/mouse.h"

namespace rm_control
{
ChannelManager::ChannelManager(ros::NodeHandle& node, ros::NodeHandle& nodeParam, std::map<std::string, bool>& modulesStatus)
    : node_(node), nodeParam_(nodeParam), modulesStatus_(modulesStatus)
{
}

bool ChannelManager::init()
{
    /* 初始化速度信息和云台信息发布者 */
    std::string twistTopic, pitchAngleTopic, yawAngleTopic; /* 速度和云台两轴话题 */
    CONFIG_ASSERT("twist_topic", nodeParam_.getParam("twist_topic", twistTopic));
    CONFIG_ASSERT("yaw_angle_topic", nodeParam_.getParam("yaw_angle_topic", yawAngleTopic));
    CONFIG_ASSERT("pitch_angle_topic", nodeParam_.getParam("pitch_angle_topic", pitchAngleTopic));
    twistPublisher_      = node_.advertise<geometry_msgs::Twist>(twistTopic, 1000);
    yawAnglePublisher_   = node_.advertise<std_msgs::Float64>(yawAngleTopic, 1000);
    pitchAnglePublisher_ = node_.advertise<std_msgs::Float64>(pitchAngleTopic, 1000);
    /* 加载各指令通道 */
    std::vector<std::string> enableChannels; /* 启用的通道 */
    nodeParam_.getParam("channels", enableChannels);
    if (std::find(enableChannels.begin(), enableChannels.end(), "joystick") != enableChannels.end()) {
        channels_["joystick"] = std::make_shared<rm_control::JoystickChannel>(node_, nodeParam_);
    }
    if (std::find(enableChannels.begin(), enableChannels.end(), "keyboard") != enableChannels.end()) {
        channels_["keyboard"] = std::make_shared<rm_control::KeyboardChannel>(node_, nodeParam_);
    }
    if (std::find(enableChannels.begin(), enableChannels.end(), "mouse") != enableChannels.end()) {
        channels_["mouse"] = std::make_shared<rm_control::MouseChannel>(node_, nodeParam_);
    }
    /* 初始化通道 */
    for (auto iter = channels_.begin(); iter != channels_.end(); iter++) {
        if (!iter->second->init()) {
            ROS_FATAL("Load Channel[%s] failed!", iter->first.c_str());
            return false;
        }
    }
    return true;
}
void ChannelManager::update(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, ros::Duration period)
{
    for (auto iter = channels_.begin(); iter != channels_.end(); iter++) {
        iter->second->getValue(vx, vy, vrz, yawAngle, pitchAngle, period, modulesStatus_);
    }
    /* 发布速度信息和云台信息 */
    geometry_msgs::Twist twist;
    twist.linear.x  = vx;
    twist.linear.y  = vy;
    twist.angular.z = vrz;
    twistPublisher_.publish(twist);
    std_msgs::Float64 pitchCMD;
    pitchCMD.data = pitchAngle;
    pitchAnglePublisher_.publish(pitchCMD);
    std_msgs::Float64 yawCMD;
    yawCMD.data = yawAngle;
    yawAnglePublisher_.publish(yawCMD);
}
}  // namespace rm_control