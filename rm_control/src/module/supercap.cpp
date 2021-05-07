#include <boost/algorithm/string/case_conv.hpp>

#include <robot_toolbox/tool.h>

#include "rm_control/module/supercap.h"

namespace rm_control
{
SupercapModule::SupercapModule(ros::NodeHandle& node, ros::NodeHandle& nodeParam)
    : ModuleInterface(node, nodeParam_), node_(node), nodeParam_(nodeParam)
{
}

void SupercapModule::stateCallback(const supercap_controller::supercap_stateConstPtr& msg)
{
    percent_ = msg->percent;
}

bool SupercapModule::init()
{
    /* 初始化超级电容限制函数 */
    limitFunction_.reset(new robot_toolbox::FunctionTool(ros::NodeHandle("~supercap/function"), ros::NodeHandle("~supercap/function")));
    if (!limitFunction_->init()) {
        ROS_FATAL("Init supercap limit function failed!");
        return false;
    }
    /* 读取配置 */
    CONFIG_ASSERT("supercap/topic", nodeParam_.getParam("supercap/topic", stateTopic_));
    stateSubscriber_ = node_.subscribe<supercap_controller::supercap_state>(stateTopic_, 1000, &SupercapModule::stateCallback, this);
    return true;
}

void SupercapModule::getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, bool& isEnable, ros::Duration period)
{
    if (isEnable) {
        double percent = limitFunction_->compute(percent_);
        vx *= percent;
        vy *= percent;
        yawAngle *= percent;
        pitchAngle *= percent;
    }
}
}  // namespace rm_control