#include <boost/algorithm/string/case_conv.hpp>

#include <robot_toolbox/tool.h>

#include "rc_control/module/chassis_follow_gimbal.h"

namespace rc_control
{
ChassisFollowGimbalModule::ChassisFollowGimbalModule(ros::NodeHandle& node, ros::NodeHandle& nodeParam)
    : ModuleInterface(node, nodeParam_), node_(node), nodeParam_(nodeParam)
{
}

void ChassisFollowGimbalModule::stateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    jointPosition_ = msg->position;
}

bool ChassisFollowGimbalModule::init()
{
    /* 初始化底盘跟随云台PID和电机角度信息 */
    pid_.init(ros::NodeHandle("~/chassis_follow_gimbal/pid"));
    CONFIG_ASSERT("chassis_follow_gimbal/yaw_number", nodeParam_.getParam("chassis_follow_gimbal/yaw_number", yawNumber_) && yawNumber_ >= 0);
    return true;
}

void ChassisFollowGimbalModule::getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, bool& isEnable, ros::Duration period)
{
    /* 计算底盘跟随云台PID值 */
    if (isEnable && jointPosition_.size() > yawNumber_) {
        vrz = pid_.computeCommand(0 - jointPosition_[yawNumber_], period);
    } else {
        pid_.reset();
    }
}
}  // namespace rc_control