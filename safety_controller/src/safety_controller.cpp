#include <ros/ros.h>

#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <robot_toolbox/tool.h>

#include "safety_controller/safety_controller.h"
#include "robot_msgs/BoolStamped.h"

namespace safety_controller
{
SafetyController::SafetyController()
{
}

void SafetyController::commandCallback(const robot_msgs::BoolStampedConstPtr& msg)
{
    handle_.setStatus(msg->result);
    timeoutTimer_.stop();
    timeoutTimer_.start();
}

bool SafetyController::init(robot_interface::SafetyInterface *hw, ros::NodeHandle &node)
{
    /* 读取配置并注册发布者 */
    std::string handleName;
    node.param<double>("publish_rate", publishRate_, 100);
    node.param<std::string>("handle_name", handleName, "safety");
    node.param<double>("timeout", timeout_, 0.1);
    handle_ = hw->getHandle(handleName);
    commandSubscriber_ = node.subscribe<robot_msgs::BoolStamped>("command", 1000, &SafetyController::commandCallback, this);
    timeoutTimer_ = node.createTimer(ros::Duration(timeout_), boost::bind(&SafetyController::timeoutCallback, this), true, true);
    statePublisher_.reset(new realtime_tools::RealtimePublisher<robot_msgs::BoolStamped>(node, "state", 1000));
    ROS_INFO("Safety Controller started!");
    return true;
}

void SafetyController::timeoutCallback()
{
    handle_.setStatus(true);
}

void SafetyController::update(const ros::Time &time, const ros::Duration &period)
{
    /* 发送话题 */
    lastPublishDuration_ += period;
    if (lastPublishDuration_.toSec() >= 1 / publishRate_) {
        lastPublishDuration_ = ros::Duration(0);
        if (statePublisher_ && statePublisher_->trylock()) {
            statePublisher_->msg_.header.seq++;
            statePublisher_->msg_.header.stamp = time;
            statePublisher_->msg_.result = handle_.getCurrentStatus();
            statePublisher_->unlockAndPublish();
        }
    }
}

void SafetyController::starting(const ros::Time &time)
{
    handle_.setStatus(true);
}

void SafetyController::stopping(const ros::Time &time)
{
    handle_.setStatus(true);
}

}  // namespace key_controller

PLUGINLIB_EXPORT_CLASS(safety_controller::SafetyController, controller_interface::ControllerBase);