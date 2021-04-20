#include <ros/ros.h>

#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <robot_toolbox/tool.h>

#include "key_controller/key_controller.h"
#include "std_msgs/Bool.h"

namespace key_controller
{
KeyController::KeyController()
{
}

bool KeyController::init(robot_interface::IOInterface *hw, ros::NodeHandle &node)
{
    /* 读取配置并注册发布者 */
    node.param<double>("publish_rate", publishRate_, 100);
    XmlRpc::XmlRpcValue keyList;
    node.getParam("key", keyList);
    CONFIG_ASSERT("key", keyList.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (auto iter = keyList.begin(); iter != keyList.end(); iter++) {
        CONFIG_ASSERT("key/" + iter->first, iter->second.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        CONFIG_ASSERT("key/" + iter->first + "/active_low", iter->second["active_low"].getType() == XmlRpc::XmlRpcValue::TypeBoolean);
        keys_[iter->first].isActiveLow = static_cast<bool>(iter->second["active_low"]);
        CONFIG_ASSERT("key/" + iter->first + "/anti_shake_time", iter->second["anti_shake_time"].getType() == XmlRpc::XmlRpcValue::TypeDouble && static_cast<double>(iter->second["anti_shake_time"]) >= 0);
        keys_[iter->first].antiShakeTime = static_cast<double>(iter->second["anti_shake_time"]);
        keys_[iter->first].handle        = hw->getHandle(iter->first);
        keys_[iter->first].statePublisher.reset(new realtime_tools::RealtimePublisher<std_msgs::Bool>(node, "key/" + iter->first + "/state", 1000));
        keys_[iter->first].lastActiveTime = keys_[iter->first].lastActiveTime.fromSec(0);
        ROS_INFO("Key initialize success, name = %s, active_low = %s, anti_shake_time = %f.", iter->first.c_str(), keys_[iter->first].isActiveLow ? "true" : "false", keys_[iter->first].antiShakeTime);
    }
    ROS_INFO("Key Controller started!");
    return true;
}

void KeyController::update(const ros::Time &time, const ros::Duration &period)
{
    /* 判断按钮是否已经激活,激活后进行防抖判断 */
    for (auto iter = keys_.begin(); iter != keys_.end(); iter++) {
        if (iter->second.handle.getCurrentLevel() == !iter->second.isActiveLow) {
            if (iter->second.lastActiveTime.is_zero()) {
                iter->second.lastActiveTime = time;
            }
        } else {
            iter->second.lastActiveTime = iter->second.lastActiveTime.fromSec(0);
        }
    }
    /* 发送话题 */
    lastPublishDuration_ += period;
    if (lastPublishDuration_.toSec() >= 1 / publishRate_) {
        lastPublishDuration_ = ros::Duration(0);
        for (auto iter = keys_.begin(); iter != keys_.end(); iter++) {
            if (iter->second.statePublisher && iter->second.statePublisher->trylock()) {
                iter->second.statePublisher->msg_.data = !iter->second.lastActiveTime.is_zero() && time - iter->second.lastActiveTime > ros::Duration(iter->second.antiShakeTime);
                iter->second.statePublisher->unlockAndPublish();
            }
        }
    }
}

void KeyController::starting(const ros::Time &time)
{
}

void KeyController::stopping(const ros::Time &time)
{
}

}  // namespace key_controller

PLUGINLIB_EXPORT_CLASS(key_controller::KeyController, controller_interface::ControllerBase);