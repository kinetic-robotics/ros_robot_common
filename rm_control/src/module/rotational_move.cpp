#include <robot_toolbox/tool.h>

#include "rm_control/module/rotational_move.h"

namespace rm_control
{
RotationalMoveModule::RotationalMoveModule(ros::NodeHandle& node, ros::NodeHandle& nodeParam)
    : ModuleInterface(node, nodeParam_), node_(node), nodeParam_(nodeParam)
{
}

void RotationalMoveModule::stateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    for (size_t i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == yawName_) {
            yawPosition_ = msg->position[i];
            return;
        }
    }
    ROS_WARN("Can't find yaw joint in JointState message!");
}

bool RotationalMoveModule::init()
{
    /* 初始化底盘跟随云台PID和电机角度信息 */
    CONFIG_ASSERT("joint_state_topic", nodeParam_.getParam("joint_state_topic", stateTopic_));
    CONFIG_ASSERT("yaw_name", nodeParam_.getParam("yaw_name", yawName_));
    /* 订阅电机信息 */
    stateSubscriber_ = node_.subscribe<sensor_msgs::JointState>(stateTopic_, 1000, &RotationalMoveModule::stateCallback, this);
    return true;
}

void RotationalMoveModule::getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, ShotStatus& shotStatus, bool& isEnable, ros::Duration period, std::map<std::string, bool>& enableModules)
{
    if (isEnable) {
        double vxTarget = vx, vyTarget = vy;
        vx = cos(yawPosition_) * vxTarget - sin(yawPosition_) * vyTarget;
        vy = cos(yawPosition_) * vyTarget + sin(yawPosition_) * vxTarget;
    }
}
}  // namespace rm_control