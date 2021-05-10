#include <robot_toolbox/tool.h>

#include "rm_control/channel/joystick.h"

namespace rm_control
{
JoystickChannel::JoystickChannel(ros::NodeHandle& node, ros::NodeHandle& nodeParam)
    : ChannelInterface(node, nodeParam_), node_(node), nodeParam_(nodeParam)
{
}

void JoystickChannel::joyCallback(const sensor_msgs::JoyConstPtr& msg)
{
    vx_ = vxFunction_->compute(msg->axes[vxAxesNumber_]);
    vy_ = vyFunction_->compute(msg->axes[vyAxesNumber_]);
    vrz_ = vrzFunction_->compute(msg->axes[vrzAxesNumber_]);
    yawAngle_ += yawAngleFunction_->compute(msg->axes[yawAngleAxesNumber_]);
    pitchAngle_ += pitchAngleFunction_->compute(msg->axes[pitchAngleAxesNumber_]);
    /* 上拉左拨杆并回位开启或关闭摩擦轮 */
    if (lastFrictionButtonState_ != msg->buttons[frictionButtonNumber_] && lastFrictionButtonState_ == frictionToggleState_) {
        isShouldToggleFriction_ = true;
    }
    lastFrictionButtonState_ = msg->buttons[frictionButtonNumber_];
}

bool JoystickChannel::init()
{
    /* 读取配置 */
    CONFIG_ASSERT("joystick/vx/axes_number", nodeParam_.getParam("joystick/vx/axes_number", vxAxesNumber_) && vxAxesNumber_ >= 0);
    CONFIG_ASSERT("joystick/vy/axes_number", nodeParam_.getParam("joystick/vy/axes_number", vyAxesNumber_) && vyAxesNumber_ >= 0);
    CONFIG_ASSERT("joystick/vrz/axes_number", nodeParam_.getParam("joystick/vrz/axes_number", vrzAxesNumber_) && vrzAxesNumber_ >= 0);
    CONFIG_ASSERT("joystick/yaw_angle/axes_number", nodeParam_.getParam("joystick/yaw_angle/axes_number", yawAngleAxesNumber_) && yawAngleAxesNumber_ >= 0);
    CONFIG_ASSERT("joystick/pitch_angle/axes_number", nodeParam_.getParam("joystick/pitch_angle/axes_number", pitchAngleAxesNumber_) && pitchAngleAxesNumber_ >= 0);
    CONFIG_ASSERT("joystick/pitch_angle/max_angle", nodeParam_.getParam("joystick/pitch_angle/max_angle", maxPitchAngle_));
    CONFIG_ASSERT("joystick/pitch_angle/min_angle", nodeParam_.getParam("joystick/pitch_angle/min_angle", minPitchAngle_) && maxPitchAngle_ > minPitchAngle_);
    CONFIG_ASSERT("joystick/topic", nodeParam_.getParam("joystick/topic", joyTopic_));
    CONFIG_ASSERT("joystick/friction/button_number", nodeParam_.getParam("joystick/friction/button_number", frictionButtonNumber_) && frictionButtonNumber_ >= 0);
    CONFIG_ASSERT("joystick/friction/toggle_state", nodeParam_.getParam("joystick/friction/toggle_state", frictionToggleState_));
    /* 初始化函数类 */
    vxFunction_.reset(new robot_toolbox::FunctionTool(ros::NodeHandle("~joystick/vx/function"), ros::NodeHandle("~joystick/vx/function")));
    vyFunction_.reset(new robot_toolbox::FunctionTool(ros::NodeHandle("~joystick/vy/function"), ros::NodeHandle("~joystick/vy/function")));
    vrzFunction_.reset(new robot_toolbox::FunctionTool(ros::NodeHandle("~joystick/vrz/function"), ros::NodeHandle("~joystick/vrz/function")));
    yawAngleFunction_.reset(new robot_toolbox::FunctionTool(ros::NodeHandle("~joystick/yaw_angle/function"), ros::NodeHandle("~joystick/yaw_angle/function")));
    pitchAngleFunction_.reset(new robot_toolbox::FunctionTool(ros::NodeHandle("~joystick/pitch_angle/function"), ros::NodeHandle("~joystick/pitch_angle/function")));
    if (!vxFunction_->init() || !vyFunction_->init() || !vrzFunction_->init() || !yawAngleFunction_->init() || !pitchAngleFunction_->init()) return false;
    /* 订阅 */
    joySubscriber_ = node_.subscribe<sensor_msgs::Joy>(joyTopic_, 1000, &JoystickChannel::joyCallback, this);
    return true;
}

void JoystickChannel::getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, ros::Duration period, std::map<std::string, bool>& enableModules)
{
    vx += vx_;
    vy += vy_;
    vrz += vrz_;
    yawAngle += yawAngle_;
    LIMIT(pitchAngle_, minPitchAngle_, maxPitchAngle_);
    pitchAngle += pitchAngle_;
    if (isShouldToggleFriction_) {
        enableModules["friction"] = !enableModules["friction"];
        isShouldToggleFriction_ = false;
    }
}

}  // namespace rm_control