#include <robot_toolbox/tool.h>

#include "rm_control/module/laser.h"

namespace rm_control
{
LaserModule::LaserModule(ros::NodeHandle& node, ros::NodeHandle& nodeParam)
    : ModuleInterface(node, nodeParam_), node_(node), nodeParam_(nodeParam)
{
}

void LaserModule::gameStatusCallback(const rm_referee_controller::GameStatusConstPtr& msg)
{
    if (!isEnableAutoControlLaser_) return;
    bool shouldStartLaser = msg->process != rm_referee_controller::GameStatus::PROCESS_NOT_START;
    if (lastShouldStartLaser != shouldStartLaser) {
        if (shouldStartLaser) {
            ROS_INFO("Game started, now start laser.");
            isShouldStartOnce_ = true;
        } else {
            ROS_INFO("Game stopped, now stop laser.");
            isShouldStopOnce_ = true;
        }
    }
    lastShouldStartLaser = shouldStartLaser;
}

bool LaserModule::init()
{
    /* 读取配置 */
    CONFIG_ASSERT("referee_system/game_status_topic", nodeParam_.getParam("referee_system/game_status_topic", gameStatusTopic_));
    CONFIG_ASSERT("laser/command_topic", nodeParam_.getParam("laser/command_topic", commandTopic_));
    CONFIG_ASSERT("laser/auto_control_laser", nodeParam_.getParam("laser/auto_control_laser", isEnableAutoControlLaser_));
    /* 订阅裁判系统配置 */
    gameStatusSubscriber_ = node_.subscribe<rm_referee_controller::GameStatus>(gameStatusTopic_, 1000, &LaserModule::gameStatusCallback, this);
    /* 初始化发布者 */
    commandPublisher_ = node_.advertise<robot_msgs::BoolStamped>(commandTopic_, 1000);
    return true;
}

void LaserModule::getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, ShotStatus& shotStatus, bool& isEnable, ros::Duration period, std::map<std::string, bool>& enableModules)
{
    if (isShouldStopOnce_) {
        isEnable = false;
        isShouldStopOnce_ = false;
    }
    if (isShouldStartOnce_) {
        isEnable = true;
        isShouldStartOnce_ = false;
    }
    robot_msgs::BoolStamped msg;
    msg.header.seq = commandHeaderSeq_++;
    msg.header.stamp = ros::Time::now();
    msg.result = isEnable;
    commandPublisher_.publish(msg);
}
}  // namespace rm_control