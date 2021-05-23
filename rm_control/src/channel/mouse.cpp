#include <boost/algorithm/string/case_conv.hpp>

#include <robot_toolbox/tool.h>

#include "rm_control/channel/mouse.h"

namespace rm_control
{
MouseChannel::MouseChannel(ros::NodeHandle& node, ros::NodeHandle& nodeParam)
    : ChannelInterface(node, nodeParam_), node_(node), nodeParam_(nodeParam)
{
}

void MouseChannel::mouseCallback(const rm_rc_controller::MouseConstPtr& msg)
{
    yawAngle_ += yawAngleFunction_->compute(msg->x);
    if (yawAngle_ > M_PI * 2) yawAngle_ -= M_PI * 2;
    if (yawAngle_ < 0) yawAngle_ += M_PI * 2;
    pitchAngle_ += pitchAngleFunction_->compute(msg->y);
    LIMIT(pitchAngle_, minPitchAngle_, maxPitchAngle_);
    if (msg->leftButton) {
        if (isLastLeftButtonPress_) {
            if (ros::Time::now() - lastLeftButtonPressTime_ >= shotContinousCheckTime_) {
                shotStatus_ = ShotStatus::SHOT_CONTINOUS;
            }
        } else {
            isLastLeftButtonPress_ = true;
            lastLeftButtonPressTime_ = ros::Time::now();
            shotStatus_ = ShotStatus::SHOT_ONCE;
        }
    } else {
        isLastLeftButtonPress_ = false;
        lastLeftButtonPressTime_ = lastLeftButtonPressTime_.fromSec(0);
        shotStatus_ = ShotStatus::SHOT_CONTINOUS_STOP;
    }
}

bool MouseChannel::init()
{
    double shotContinousCheckTime = 0;
    /* 读取配置 */
    CONFIG_ASSERT("mouse/topic", nodeParam_.getParam("mouse/topic", mouseTopic_));
    CONFIG_ASSERT("mouse/pitch_angle/max_angle", nodeParam_.getParam("mouse/pitch_angle/max_angle", maxPitchAngle_));
    CONFIG_ASSERT("mouse/pitch_angle/min_angle", nodeParam_.getParam("mouse/pitch_angle/min_angle", minPitchAngle_) && maxPitchAngle_ > minPitchAngle_);
    CONFIG_ASSERT("mouse/shot/continous/check_time", nodeParam_.getParam("mouse/shot/continous/check_time", shotContinousCheckTime));
    shotContinousCheckTime_ = ros::Duration(shotContinousCheckTime);
    /* 初始化函数类 */
    yawAngleFunction_.reset(new robot_toolbox::FunctionTool(ros::NodeHandle("~mouse/yaw_angle/function"), ros::NodeHandle("~mouse/yaw_angle/function")));
    pitchAngleFunction_.reset(new robot_toolbox::FunctionTool(ros::NodeHandle("~mouse/pitch_angle/function"), ros::NodeHandle("~mouse/pitch_angle/function")));
    if (!yawAngleFunction_->init() || !pitchAngleFunction_->init()) return false;
    /* 订阅 */
    mouseSubscriber_ = node_.subscribe<rm_rc_controller::Mouse>(mouseTopic_, 1000, &MouseChannel::mouseCallback, this);
    /* 发布 */
    return true;
}

void MouseChannel::getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, ShotStatus& shotStatus, ros::Duration period, std::map<std::string, bool>& enableModules)
{
    yawAngle += yawAngle_;
    pitchAngle += pitchAngle_;
    shotStatus = shotStatus_;
}

}  // namespace rm_control