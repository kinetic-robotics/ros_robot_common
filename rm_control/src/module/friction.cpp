#include <robot_toolbox/tool.h>

#include "rm_control/module/friction.h"
#include <rm_referee_controller/RobotStatus.h>

namespace rm_control
{
FrictionModule::FrictionModule(ros::NodeHandle& node, ros::NodeHandle& nodeParam)
    : ModuleInterface(node, nodeParam_), node_(node), nodeParam_(nodeParam)
{
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
    CONFIG_ASSERT("friction/robot_status_topic", nodeParam_.getParam("friction/robot_status_topic", robotStatusTopic_));
    CONFIG_ASSERT("friction/speed_topic", nodeParam_.getParam("friction/speed_topic", speedTopic_));
    shooterType_ = static_cast<ShooterType>(shooterType);
    /* 初始化转速函数 */
    speedFunction_.reset(new robot_toolbox::FunctionTool(ros::NodeHandle("~friction/function"), ros::NodeHandle("~friction/function")));
    if (!speedFunction_->init()) {
        ROS_FATAL("Init friction speed function failed!");
        return false;
    }
    /* 订阅裁判系统配置 */
    robotStatusSubscriber_ = node_.subscribe<rm_referee_controller::RobotStatus>(robotStatusTopic_, 1000, &FrictionModule::robotStatusCallback, this);
    /* 初始化发布者 */
    speedPublisher_ = node_.advertise<robot_msgs::Float64Stamped>(speedTopic_, 1000);
    return true;
}

void FrictionModule::getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, bool& isEnable, ros::Duration period)
{
    /* 上电关摩擦轮 */
    if (isFirstLoop_) isEnable = false;
    isFirstLoop_ = false;
    robot_msgs::Float64Stamped speed;
    speed.header.seq = speedHeaderSeq_++;
    speed.header.stamp = ros::Time::now();
    speed.result = isEnable ? targetSpeed_ : 0;
    speedPublisher_.publish(speed);
}
}  // namespace rm_control