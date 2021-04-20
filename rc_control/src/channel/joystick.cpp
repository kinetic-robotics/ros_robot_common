#include <robot_toolbox/tool.h>

#include "rc_control/channel/joystick.h"

namespace rc_control
{
JoystickChannel::JoystickChannel(ros::NodeHandle& node, ros::NodeHandle& nodeParam)
    : ChannelInterface(node, nodeParam_), node_(node), nodeParam_(nodeParam)
{
}

void JoystickChannel::joyCallback(const sensor_msgs::JoyConstPtr& msg)
{
    vx_ = vxFunction_->compute(msg->axes[vxAxesNumber_]);
    vy_ = vyFunction_->compute(msg->axes[vyAxesNumber_]);
    yawAngle_ += yawAngleFunction_->compute(msg->axes[yawAngleAxesNumber_]);
    pitchAngle_ += pitchAngleFunction_->compute(msg->axes[pitchAngleAxesNumber_]);
}

bool JoystickChannel::init()
{
    /* 读取配置 */
    CONFIG_ASSERT("joystick/vx/axes_number", nodeParam_.getParam("joystick/vx/axes_number", vxAxesNumber_) && vxAxesNumber_ >= 0);
    CONFIG_ASSERT("joystick/vy/axes_number", nodeParam_.getParam("joystick/vy/axes_number", vyAxesNumber_) && vyAxesNumber_ >= 0);
    CONFIG_ASSERT("joystick/yaw_angle/axes_number", nodeParam_.getParam("joystick/yaw_angle/axes_number", yawAngleAxesNumber_) && yawAngleAxesNumber_ >= 0);
    CONFIG_ASSERT("joystick/pitch_angle/axes_number", nodeParam_.getParam("joystick/pitch_angle/axes_number", pitchAngleAxesNumber_) && pitchAngleAxesNumber_ >= 0);
    CONFIG_ASSERT("joystick/topic", nodeParam_.getParam("joystick/topic", joyTopic_));
    /* 初始化函数类 */
    vxFunction_.reset(new robot_toolbox::FunctionTool(ros::NodeHandle("~joystick/vx/function"), ros::NodeHandle("~joystick/vx/function")));
    vyFunction_.reset(new robot_toolbox::FunctionTool(ros::NodeHandle("~joystick/vy/function"), ros::NodeHandle("~joystick/vy/function")));
    yawAngleFunction_.reset(new robot_toolbox::FunctionTool(ros::NodeHandle("~joystick/yaw_angle/function"), ros::NodeHandle("~joystick/yaw_angle/function")));
    pitchAngleFunction_.reset(new robot_toolbox::FunctionTool(ros::NodeHandle("~joystick/pitch_angle/function"), ros::NodeHandle("~joystick/pitch_angle/function")));
    if (!vxFunction_->init() || !vyFunction_->init() || !yawAngleFunction_->init() || !pitchAngleFunction_->init()) return false;
    /* 订阅 */
    joySubscriber_ = node_.subscribe<sensor_msgs::Joy>(joyTopic_, 1000, &JoystickChannel::joyCallback, this);
    return true;
}

void JoystickChannel::getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, std::map<std::string, bool>& enableModules)
{
    vx += vx_;
    vy += vy_;
    yawAngle += yawAngle_;
    pitchAngle += pitchAngle_;
}

}  // namespace rc_control