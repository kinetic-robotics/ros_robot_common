#include <angles/angles.h>
#include <robot_toolbox/tool.h>

#include "rm_control/module/chassis_follow_gimbal.h"

namespace rm_control
{
ChassisFollowGimbalModule::ChassisFollowGimbalModule(ros::NodeHandle& node, ros::NodeHandle& nodeParam)
    : ModuleInterface(node, nodeParam_), node_(node), nodeParam_(nodeParam)
{
}

void ChassisFollowGimbalModule::stateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    for (size_t i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == yawName_) {
            yawPosition_ = msg->position[i];
            return;
        }
    }
    ROS_WARN("Can't find yaw joint in JointState message!");
}

bool ChassisFollowGimbalModule::init()
{
    /* 初始化底盘跟随云台PID和电机角度信息 */
    pid_.init(ros::NodeHandle("~/chassis_follow_gimbal/pid"));
    CONFIG_ASSERT("joint_state_topic", nodeParam_.getParam("joint_state_topic", stateTopic_));
    CONFIG_ASSERT("yaw_name", nodeParam_.getParam("yaw_name", yawName_));
    /* 订阅电机信息 */
    stateSubscriber_ = node_.subscribe<sensor_msgs::JointState>(stateTopic_, 1000, &ChassisFollowGimbalModule::stateCallback, this);
    return true;
}

void ChassisFollowGimbalModule::getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, ShotStatus& shotStatus, bool& isEnable, ros::Duration period)
{
    /* 计算底盘跟随云台PID值 */
    if (isEnable) {
        vrz += pid_.computeCommand(angles::shortest_angular_distance(yawPosition_, 0), period);
    } else {
        pid_.reset();
    }
}
}  // namespace rm_control