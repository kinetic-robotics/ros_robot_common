#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <robot_interface/remote_controller.h>

#include "rm_rc_controller/rm_rc_controller.h"

namespace rm_rc_controller
{
RMRCController::RMRCController()
{
}

bool RMRCController::init(robot_interface::RemoteControllerInterface* hw, ros::NodeHandle& node)
{
    std::string handleName;
    node.param<double>("publish_rate", publishRate_, 100);
    node.param<std::string>("handle_name", handleName, "rm_remote_controller");
    handle_ = hw->getHandle(handleName);
    if (handle_.getCHPtr()->size() != 8 || handle_.getSWPtr()->size() != 20) {
        ROS_FATAL("The RC interface must have 8 channels and 20 switch, or we can't publish keyboard and mouse data!");
        return false;
    }
    joyPublisher_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::Joy>(node, "joy", 1000));
    kbPublisher_.reset(new realtime_tools::RealtimePublisher<Keyboard>(node, "keyboard", 1000));
    mousePublisher_.reset(new realtime_tools::RealtimePublisher<Mouse>(node, "mouse", 1000));
    ROS_INFO("Robomaster RC Controller started, handle_name =  %s, publish_rate = %f", handleName.c_str(), publishRate_);
    return true;
}

void RMRCController::update(const ros::Time& time, const ros::Duration& period)
{
    lastPublishDuration_ += period;
    if (lastPublishDuration_.toSec() >= 1 / publishRate_) {
        lastPublishDuration_ = ros::Duration(0);
        std::vector<float> axes;
        std::vector<int> sw;
        auto rawAxes = handle_.getCH();
        auto rawSW   = handle_.getSW();
        for (size_t i = 0; i < 5; i++) {
            axes.push_back(rawAxes[i]);
        }
        for (size_t i = 0; i < 2; i++) {
            if (rawSW[i] == robot_interface::RemoteControllerHandle::Switch::DOWN) {
                sw.push_back(-1);
            } else if (rawSW[i] == robot_interface::RemoteControllerHandle::Switch::UP) {
                sw.push_back(1);
            } else {
                sw.push_back(0);
            }
        }
        /* 发布遥控器信息 */
        if (joyPublisher_ && joyPublisher_->trylock()) {
            joyPublisher_->msg_.header.seq++;
            joyPublisher_->msg_.header.stamp = time;
            joyPublisher_->msg_.axes         = axes;
            joyPublisher_->msg_.buttons      = sw;
            joyPublisher_->unlockAndPublish();
        }
        /* 发布键盘信息 */
        if (kbPublisher_ && kbPublisher_->trylock()) {
            kbPublisher_->msg_.header.seq++;
            kbPublisher_->msg_.header.stamp = time;
            kbPublisher_->msg_.W            = static_cast<int>(rawSW[2]);
            kbPublisher_->msg_.S            = static_cast<int>(rawSW[3]);
            kbPublisher_->msg_.A            = static_cast<int>(rawSW[4]);
            kbPublisher_->msg_.D            = static_cast<int>(rawSW[5]);
            kbPublisher_->msg_.SHIFT        = static_cast<int>(rawSW[6]);
            kbPublisher_->msg_.CTRL         = static_cast<int>(rawSW[7]);
            kbPublisher_->msg_.Q            = static_cast<int>(rawSW[8]);
            kbPublisher_->msg_.E            = static_cast<int>(rawSW[9]);
            kbPublisher_->msg_.R            = static_cast<int>(rawSW[10]);
            kbPublisher_->msg_.F            = static_cast<int>(rawSW[11]);
            kbPublisher_->msg_.G            = static_cast<int>(rawSW[12]);
            kbPublisher_->msg_.Z            = static_cast<int>(rawSW[13]);
            kbPublisher_->msg_.X            = static_cast<int>(rawSW[14]);
            kbPublisher_->msg_.C            = static_cast<int>(rawSW[15]);
            kbPublisher_->msg_.V            = static_cast<int>(rawSW[16]);
            kbPublisher_->msg_.B            = static_cast<int>(rawSW[17]);
            kbPublisher_->unlockAndPublish();
        }
        /* 发布鼠标信息 */
        if (mousePublisher_ && mousePublisher_->trylock()) {
            mousePublisher_->msg_.header.seq++;
            mousePublisher_->msg_.header.stamp = time;
            mousePublisher_->msg_.x            = rawAxes[5];
            mousePublisher_->msg_.y            = rawAxes[6];
            mousePublisher_->msg_.z            = rawAxes[7];
            mousePublisher_->msg_.leftButton   = static_cast<int>(rawSW[18]);
            mousePublisher_->msg_.rightButton  = static_cast<int>(rawSW[19]);
            mousePublisher_->unlockAndPublish();
        }
    }
}

void RMRCController::starting(const ros::Time& time)
{
}

void RMRCController::stopping(const ros::Time& time)
{
}

}  // namespace rm_rc_controller

PLUGINLIB_EXPORT_CLASS(rm_rc_controller::RMRCController, controller_interface::ControllerBase);