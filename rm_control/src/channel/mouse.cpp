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
    pitchAngle_ += pitchAngleFunction_->compute(msg->y);
}

bool MouseChannel::init()
{
    /* 读取配置 */
    CONFIG_ASSERT("mouse/topic", nodeParam_.getParam("mouse/topic", mouseTopic_));
    /* 初始化函数类 */
    yawAngleFunction_.reset(new robot_toolbox::FunctionTool(ros::NodeHandle("~mouse/yaw_angle/function"), ros::NodeHandle("~mouse/yaw_angle/function")));
    pitchAngleFunction_.reset(new robot_toolbox::FunctionTool(ros::NodeHandle("~mouse/pitch_angle/function"), ros::NodeHandle("~mouse/pitch_angle/function")));
    if (!yawAngleFunction_->init() || !pitchAngleFunction_->init()) return false;
    /* 订阅 */
    mouseSubscriber_ = node_.subscribe<rm_rc_controller::Mouse>(mouseTopic_, 1000, &MouseChannel::mouseCallback, this);
    return true;
}

void MouseChannel::getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, ros::Duration period, std::map<std::string, bool>& enableModules)
{
    yawAngle += yawAngle_;
    pitchAngle += pitchAngle_;
}

}  // namespace rm_control