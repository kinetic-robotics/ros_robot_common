#include <boost/algorithm/string/case_conv.hpp>

#include <robot_toolbox/tool.h>
#include <std_msgs/Float64.h>

#include "rm_control/module/supercap.h"

namespace rm_control
{
SupercapModule::SupercapModule(ros::NodeHandle& node, ros::NodeHandle& nodeParam)
    : ModuleInterface(node, nodeParam_), node_(node), nodeParam_(nodeParam)
{
}

void SupercapModule::stateCallback(const supercap_controller::SupercapStateConstPtr& msg)
{
    percent_ = msg->percent;
}

void SupercapModule::robotStatusCallback(const rm_referee_controller::RobotStatusConstPtr& msg)
{
    targetPower_ = msg->chassisPowerLimit;
}

bool SupercapModule::init()
{
    /* 初始化超级电容容量限制函数 */
    limitFunction_.reset(new robot_toolbox::FunctionTool(ros::NodeHandle("~supercap/capacity_function")));
    if (!limitFunction_->init()) {
        ROS_FATAL("Init supercap capacity limit speed function failed!");
        return false;
    }
    /* 初始化超级电容功率限制函数 */
    powerFunction_.reset(new robot_toolbox::FunctionTool(ros::NodeHandle("~supercap/power_function")));
    if (!powerFunction_->init()) {
        ROS_FATAL("Init supercap power limit speed function failed!");
        return false;
    }
    /* 读取配置 */
    CONFIG_ASSERT("referee_system/robot_status_topic", nodeParam_.getParam("referee_system/robot_status_topic", robotStatusTopic_));
    CONFIG_ASSERT("supercap/state_topic", nodeParam_.getParam("supercap/state_topic", stateTopic_));
    CONFIG_ASSERT("supercap/command_topic", nodeParam_.getParam("supercap/command_topic", commandTopic_));
    stateSubscriber_       = node_.subscribe<supercap_controller::SupercapState>(stateTopic_, 1000, &SupercapModule::stateCallback, this);
    robotStatusSubscriber_ = node_.subscribe<rm_referee_controller::RobotStatus>(robotStatusTopic_, 1000, &SupercapModule::robotStatusCallback, this);
    commandTopicPublisher_ = node_.advertise<std_msgs::Float64>(commandTopic_, 1000, false);
    return true;
}

void SupercapModule::getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, ShotStatus& shotStatus, bool& isEnable, ros::Duration period)
{
    if (isEnable) {
        double percent = limitFunction_->compute(percent_) * powerFunction_->compute(targetPower_);
        vx *= percent;
        vy *= percent;
        vrz *= percent;
    }
    /* 发布超级电容目标功率 */
    std_msgs::Float64 msg;
    msg.data = targetPower_;
    commandTopicPublisher_.publish(msg);
}
}  // namespace rm_control