#include <robot_toolbox/tool.h>

#include "rm_control/module/friction.h"
#include <rm_referee_controller/RobotStatus.h>

namespace rm_control
{
FrictionModule::FrictionModule(ros::NodeHandle& node, ros::NodeHandle& nodeParam)
    : ModuleInterface(node, nodeParam_), node_(node), nodeParam_(nodeParam)
{
}

void FrictionModule::gameStatusCallback(const rm_referee_controller::GameStatusConstPtr& msg)
{
    if (!isEnableAutoControlFriction_) return;
    bool shouldStartFriction = msg->process != rm_referee_controller::GameStatus::PROCESS_NOT_START;
    if (lastShouldStartFriction != shouldStartFriction) {
        if (shouldStartFriction) {
            ROS_INFO("Game started, now start friction.");
            isShouldStartOnce_ = true;
        } else {
            ROS_INFO("Game stopped, now stop friction.");
            isShouldStopOnce_ = true;
        }
    }
    lastShouldStartFriction = shouldStartFriction;
}

void FrictionModule::robotStatusCallback(const rm_referee_controller::RobotStatusConstPtr& msg)
{
    switch (shooterType_) {
        case ShooterType::FIRST17MM:
            targetSpeed_ = msg->shooter.first17mm.speedLimit;
            break;
        case ShooterType::SECOND17MM:
            targetSpeed_ = msg->shooter.second17mm.speedLimit;
            break;
        case ShooterType::FIRST42MM:
            targetSpeed_ = msg->shooter.first42mm.speedLimit;
            break;
    }
}

bool FrictionModule::init()
{
    /* 读取配置 */
    int shooterType = 0;
    CONFIG_ASSERT("friction/shooter_type", nodeParam_.getParam("friction/shooter_type", shooterType) && shooterType <= 2 && shooterType >=0);
    CONFIG_ASSERT("referee_system/robot_status_topic", nodeParam_.getParam("referee_system/robot_status_topic", robotStatusTopic_));
    CONFIG_ASSERT("referee_system/game_status_topic", nodeParam_.getParam("referee_system/game_status_topic", gameStatusTopic_));
    CONFIG_ASSERT("friction/speed_topic", nodeParam_.getParam("friction/speed_topic", speedTopic_));
    CONFIG_ASSERT("friction/auto_control_friction", nodeParam_.getParam("friction/auto_control_friction", isEnableAutoControlFriction_));
    shooterType_ = static_cast<ShooterType>(shooterType);
    /* 订阅裁判系统配置 */
    robotStatusSubscriber_ = node_.subscribe<rm_referee_controller::RobotStatus>(robotStatusTopic_, 1000, &FrictionModule::robotStatusCallback, this);
    gameStatusSubscriber_ = node_.subscribe<rm_referee_controller::GameStatus>(gameStatusTopic_, 1000, &FrictionModule::gameStatusCallback, this);
    /* 初始化发布者 */
    speedPublisher_ = node_.advertise<robot_msgs::Float64Stamped>(speedTopic_, 1000);
    return true;
}

void FrictionModule::getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, ShotStatus& shotStatus, bool& isEnable, ros::Duration period, std::map<std::string, bool>& enableModules)
{
    if (isShouldStopOnce_) {
        isEnable = false;
        isShouldStopOnce_ = false;
    }
    if (isShouldStartOnce_) {
        isEnable = true;
        isShouldStartOnce_ = false;
    }
    robot_msgs::Float64Stamped speed;
    speed.header.seq = speedHeaderSeq_++;
    speed.header.stamp = ros::Time::now();
    speed.result = isEnable ? targetSpeed_ : 0;
    speedPublisher_.publish(speed);
}
}  // namespace rm_control