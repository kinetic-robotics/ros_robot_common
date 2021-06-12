#include <robot_toolbox/tool.h>

#include "rm_control/module/heat_limit.h"

namespace rm_control
{
HeatLimitModule::HeatLimitModule(ros::NodeHandle& node, ros::NodeHandle& nodeParam)
    : ModuleInterface(node, nodeParam_), node_(node), nodeParam_(nodeParam)
{
}

void HeatLimitModule::robotStatusCallback(const rm_referee_controller::RobotStatusConstPtr& msg)
{
    switch (shooterType_) {
        case ShooterType::FIRST17MM:
            nowMaxHeat_ = msg->shooter.first17mm.coolingLimit;
            break;
        case ShooterType::SECOND17MM:
            nowMaxHeat_ = msg->shooter.second17mm.coolingLimit;
            break;
        case ShooterType::FIRST42MM:
            nowMaxHeat_ = msg->shooter.first42mm.coolingLimit;
            break;
    }
}

void HeatLimitModule::heatCallback(const rm_referee_controller::PowerHeatConstPtr& msg)
{
    switch (shooterType_) {
        case ShooterType::FIRST17MM:
            nowHeat_ = msg->shooterHeat.first17mm;
            break;
        case ShooterType::SECOND17MM:
            nowHeat_ = msg->shooterHeat.second17mm;
            break;
        case ShooterType::FIRST42MM:
            nowHeat_ = msg->shooterHeat.first42mm;
            break;
    }
}

void HeatLimitModule::onlineCallback(const robot_msgs::BoolStampedConstPtr& msg)
{
    isOnline_ = msg->result;
}

bool HeatLimitModule::init()
{
    /* 裁判系统相关话题订阅 */
    int shooterType = 0;
    CONFIG_ASSERT("referee_system/online_topic", nodeParam_.getParam("referee_system/online_topic", onlineTopic_));
    CONFIG_ASSERT("referee_system/power_heat_topic", nodeParam_.getParam("referee_system/power_heat_topic", heatTopic_));
    CONFIG_ASSERT("referee_system/robot_status_topic", nodeParam_.getParam("referee_system/robot_status_topic", robotStatusTopic_));
    CONFIG_ASSERT("heat_limit/shooter_type", nodeParam_.getParam("heat_limit/shooter_type", shooterType) && shooterType <= 2 && shooterType >=0);
    CONFIG_ASSERT("heat_limit/threshold_heat/once", nodeParam_.getParam("heat_limit/threshold_heat/once", shotOnceThresholdHeat_) && shotOnceThresholdHeat_ > 0);
    CONFIG_ASSERT("heat_limit/threshold_heat/continous", nodeParam_.getParam("heat_limit/threshold_heat/continous", shotContinousThresholdHeat_) && shotContinousThresholdHeat_ > 0);
    shooterType_ = static_cast<ShooterType>(shooterType);
    /* 订阅和发布 */
    robotStatusSubscriber_ = node_.subscribe<rm_referee_controller::RobotStatus>(robotStatusTopic_, 1000, &HeatLimitModule::robotStatusCallback, this);
    heatSubscriber_ = node_.subscribe<rm_referee_controller::PowerHeat>(heatTopic_, 1000, &HeatLimitModule::heatCallback, this);
    onlineSubscriber_ = node_.subscribe<robot_msgs::BoolStamped>(onlineTopic_, 1000, &HeatLimitModule::onlineCallback, this);
    return true;
}

void HeatLimitModule::getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, ShotStatus& shotStatus, bool& isEnable, ros::Duration period, std::map<std::string, bool>& enableModules)
{
    if (shotStatus == ShotStatus::SHOT_CONTINOUS) isNowContinousMode_ = true;
    if (shotStatus == ShotStatus::SHOT_CONTINOUS_STOP || shotStatus == ShotStatus::SHOT_ONCE) isNowContinousMode_ = false;
    if (isEnable && isOnline_) {
        if (shotStatus == ShotStatus::SHOT_ONCE && nowMaxHeat_ - nowHeat_ < shotOnceThresholdHeat_) shotStatus = ShotStatus::NONE;
        if (isNowContinousMode_ && nowMaxHeat_ - nowHeat_ < shotContinousThresholdHeat_) shotStatus = ShotStatus::SHOT_CONTINOUS_STOP;
    }
}
}  // namespace rm_control