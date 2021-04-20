#include <ros/ros.h>

#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>

#include "supercap_controller/supercap_controller.h"
#include "supercap_controller/supercap_state.h"

#include <std_msgs/Float64.h>

namespace supercap_controller
{
SupercapController::SupercapController()
{
}

bool SupercapController::init(robot_interface::SupercapInterface *hw, ros::NodeHandle &node)
{
    std::string handleName;
    double defaultPower = 0;
    node.param<std::string>("handle_name", handleName, "chassis_supercap");
    node.param<double>("publish_rate", publishRate_, 100);
    node.param<double>("default_power", defaultPower, 30);
    handle_ = hw->getHandle(handleName);
    handle_.setCMDTargetPower(defaultPower);
    statePublisher_.reset(new realtime_tools::RealtimePublisher<supercap_state>(node, "state", 1000));
    targetPowerSubscriber_ = node.subscribe<std_msgs::Float64>("command", 1000, &SupercapController::commandCallback, this);
    ROS_INFO("Supercap controller started, handle_name: %s, publish_rate: %f, default_power: %f", handleName.c_str(), publishRate_, defaultPower);
    return true;
}

void SupercapController::commandCallback(const std_msgs::Float64ConstPtr& msg)
{
    handle_.setCMDTargetPower(msg->data);
}



void SupercapController::update(const ros::Time &time, const ros::Duration &period)
{
    /* 发布话题 */
    lastPublishDuration_ += period;
    if (lastPublishDuration_.toSec() >= 1 / publishRate_) {
        lastPublishDuration_ = ros::Duration(0);
        if (statePublisher_ && statePublisher_->trylock()) {
            statePublisher_->msg_.header.seq++;
            statePublisher_->msg_.header.stamp = time;
            statePublisher_->msg_.percent = handle_.getPercent();
            statePublisher_->msg_.inputVoltage = handle_.getInputVoltage();
            statePublisher_->msg_.inputCurrent = handle_.getInputCurrent();
            statePublisher_->msg_.capVoltage = handle_.getCapVoltage();
            statePublisher_->msg_.targetPower = handle_.getNowTargetPower();
            statePublisher_->unlockAndPublish();
        }
    }
}

void SupercapController::starting(const ros::Time &time)
{

}

void SupercapController::stopping(const ros::Time &time)
{

}

}  // namespace supercap_controller

PLUGINLIB_EXPORT_CLASS(supercap_controller::SupercapController, controller_interface::ControllerBase);