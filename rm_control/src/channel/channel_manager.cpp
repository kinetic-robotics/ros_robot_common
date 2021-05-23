
#include "rm_control/channel/channel.h"
#include "rm_control/channel/channel_manager.h"
#include "rm_control/channel/joystick.h"
#include "rm_control/channel/keyboard.h"
#include "rm_control/channel/mouse.h"

namespace rm_control
{
ChannelManager::ChannelManager(ros::NodeHandle& node, ros::NodeHandle& nodeParam, std::map<std::string, bool>& modulesStatus)
    : node_(node), nodeParam_(nodeParam), modulesStatus_(modulesStatus)
{
}

bool ChannelManager::init()
{
    /* 加载各指令通道 */
    std::vector<std::string> enableChannels; /* 启用的通道 */
    nodeParam_.getParam("channels", enableChannels);
    if (std::find(enableChannels.begin(), enableChannels.end(), "joystick") != enableChannels.end()) {
        channels_["joystick"] = std::make_shared<rm_control::JoystickChannel>(node_, nodeParam_);
    }
    if (std::find(enableChannels.begin(), enableChannels.end(), "keyboard") != enableChannels.end()) {
        channels_["keyboard"] = std::make_shared<rm_control::KeyboardChannel>(node_, nodeParam_);
    }
    if (std::find(enableChannels.begin(), enableChannels.end(), "mouse") != enableChannels.end()) {
        channels_["mouse"] = std::make_shared<rm_control::MouseChannel>(node_, nodeParam_);
    }
    /* 初始化通道 */
    for (auto iter = channels_.begin(); iter != channels_.end(); ++iter) {
        if (!iter->second->init()) {
            ROS_FATAL("Load Channel[%s] failed!", iter->first.c_str());
            return false;
        }
    }
    return true;
}
void ChannelManager::update(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, ShotStatus& shotStatus, ros::Duration period)
{
    for (auto iter = channels_.begin(); iter != channels_.end(); ++iter) {
        iter->second->getValue(vx, vy, vrz, yawAngle, pitchAngle, shotStatus, period, modulesStatus_);
    }
}
}  // namespace rm_control