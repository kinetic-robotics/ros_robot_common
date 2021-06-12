#include <pluginlib/class_list_macros.h>


#include "bullet_cover_controller/bullet_cover_controller.h"
#include <robot_toolbox/tool.h>

namespace bullet_cover_controller
{
BulletCoverController::BulletCoverController()
{
}

void BulletCoverController::switchCallback(const robot_msgs::BoolStampedConstPtr& msg)
{
    isOn_ = msg->result;
}

bool BulletCoverController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& node)
{
    /* 读取配置 */
    std::string handleName;
    CONFIG_ASSERT("handle_name", node.getParam("handle_name", handleName));
    CONFIG_ASSERT("position/switch_on", node.getParam("position/switch_on", targetOnPosition_) && targetOnPosition_ >= 0 && targetOnPosition_ <= M_PI * 2);
    CONFIG_ASSERT("position/switch_off", node.getParam("position/switch_off", targetOffPosition_) && targetOffPosition_ >= 0 && targetOffPosition_ <= M_PI * 2);
    handle_ = hw->getHandle(handleName);
    statePublisher_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(node, "state", 1000));
    statePublisher_->msg_.p = 0;
    statePublisher_->msg_.i = 0;
    statePublisher_->msg_.d = 0;
    statePublisher_->msg_.i_clamp = 0;
    statePublisher_->msg_.antiwindup    = false;
    switchSubscriber_ = node.subscribe<robot_msgs::BoolStamped>("command", 1000, &BulletCoverController::switchCallback, this);
    ROS_INFO("Bullet Cover Controller started.");
    return true;
}

void BulletCoverController::update(const ros::Time& time, const ros::Duration& period)
{
    /* 发布话题 */
    double targetPosition = isOn_ ? targetOnPosition_ : targetOffPosition_;
    double error = targetPosition - handle_.getPosition();
    handle_.setCommand(targetPosition);
    if (statePublisher_ && statePublisher_->trylock()) {
        statePublisher_->msg_.set_point     = targetPosition;
        statePublisher_->msg_.process_value = handle_.getVelocity();
        statePublisher_->msg_.error         = error;
        statePublisher_->msg_.time_step     = period.toSec();
        statePublisher_->msg_.command       = targetPosition;
        statePublisher_->msg_.header.stamp  = time;
        statePublisher_->msg_.header.seq++;
        statePublisher_->unlockAndPublish();
    }
}

void BulletCoverController::starting(const ros::Time& time)
{
}

void BulletCoverController::stopping(const ros::Time& time)
{
    handle_.setCommand(targetOffPosition_);
}

}  // namespace fire_controller

PLUGINLIB_EXPORT_CLASS(bullet_cover_controller::BulletCoverController, controller_interface::ControllerBase);