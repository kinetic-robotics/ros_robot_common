#include <pluginlib/class_list_macros.h>
#include <robot_toolbox/tool.h>

#include "laser_controller/laser_controller.h"

namespace laser_controller
{
LaserController::LaserController()
{
}

void LaserController::switchCallback(const robot_msgs::BoolStampedConstPtr& msg)
{
    isOn_ = msg->result;
}

bool LaserController::init(robot_interface::IOInterface* hw, ros::NodeHandle& node)
{
    /* 读取配置 */
    std::string handleName;
    CONFIG_ASSERT("handle_name", node.getParam("handle_name", handleName));
    node.param<double>("publish_rate", publishRate_, 100);
    handle_ = hw->getHandle(handleName);
    statePublisher_.reset(new realtime_tools::RealtimePublisher<robot_msgs::BoolStamped>(node, "state", 1000));
    statePublisher_->msg_.header.seq = 0;
    switchSubscriber_                = node.subscribe<robot_msgs::BoolStamped>("command", 1000, &LaserController::switchCallback, this);
    ROS_INFO("Laser Controller started.");
    return true;
}

void LaserController::update(const ros::Time& time, const ros::Duration& period)
{
    /* 发布话题 */
    handle_.setLevel(isOn_);
    lastPublishDuration_ += period;
    if (lastPublishDuration_.toSec() >= 1 / publishRate_) {
        lastPublishDuration_ = ros::Duration(0);
        if (statePublisher_ && statePublisher_->trylock()) {
            statePublisher_->msg_.result       = handle_.getCurrentLevel();
            statePublisher_->msg_.header.stamp = time;
            statePublisher_->msg_.header.seq++;
            statePublisher_->unlockAndPublish();
        }
    }
}

void LaserController::starting(const ros::Time& time)
{
}

void LaserController::stopping(const ros::Time& time)
{
    handle_.setLevel(false);
}

}  // namespace laser_controller

PLUGINLIB_EXPORT_CLASS(laser_controller::LaserController, controller_interface::ControllerBase);