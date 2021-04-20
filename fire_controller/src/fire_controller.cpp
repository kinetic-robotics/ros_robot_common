#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

#include "fire_controller/fire_controller.h"

namespace fire_controller
{
FireController::FireController()
{
}

void FireController::shotTimeoutCallback()
{
    if (machineState_ == MachineState::SHOT_CONTINUOUS) {
        machineState_ = MachineState::IDLE;
    }
    isAlreadyShotOnce = false;
}

void FireController::shotCallback(const std_msgs::EmptyConstPtr& msg)
{
    /* 单/连发模式 */
    if (isAlreadyShotOnce) {
        machineState_ = MachineState::SHOT_CONTINUOUS;
    } else {
        if (machineState_ == MachineState::SHOT_ONCE) {
            targetPosition_ += +onceAngle_;
        } else {
            targetPosition_ = handle_.getPosition() + onceAngle_;
        }
        lastChangeTargetTime_ = ros::Time::now();
        machineState_     = MachineState::SHOT_ONCE;
        isAlreadyShotOnce = true;
    }
    shotTimeoutTimer_.stop();
    shotTimeoutTimer_.start();
}

bool FireController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& node)
{
    /* 读取配置 */
    std::string handleName;
    double shotTimeout;
    node.param<std::string>("handle_name", handleName, "base_to_fire_link");
    node.param<double>("shot_timeout", shotTimeout, 0.05);
    node.param<double>("continuous_speed", continuousSpeed_, 500);
    node.param<double>("once_angle", onceAngle_, 1.047);
    node.param<bool>("stuck/enable", isEnableStuckCheck_, false);
    node.param<double>("stuck/inverse_time", stuckInverseTime_, 1);
    node.param<double>("stuck/inverse_speed", stuckInverseSpeed_, 1000);
    node.param<double>("stuck/check_time", stuckInverseTime_, 1);
    node.param<double>("stuck/check_speed", stuckInverseSpeed_, 100);
    node.param<double>("position/check_time", positionCheckTime_, 0.2);
    node.param<double>("position/check_error", positionCheckError_, 0.02);
    handle_           = hw->getHandle(handleName);
    shotTimeoutTimer_ = node.createTimer(ros::Duration(shotTimeout), boost::bind(&FireController::shotTimeoutCallback, this), true, true);
    /* 初始化话题发布者并订阅话题 */
    speedStatePublisher_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(node, "speed/state", 1000));
    positionStatePublisher_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(node, "position/state", 1000));
    shotSubscriber_ = node.subscribe<std_msgs::Empty>("shot", 1000, &FireController::shotCallback, this);
    /* 初始化PID */
    if (!speedPID_.init(ros::NodeHandle(node, "speed/pid")) || !positionPID_.init(ros::NodeHandle(node, "position/pid"))) {
        ROS_FATAL("PID Controller inits failed!");
        return false;
    }
    ROS_INFO(
        "Fire controller started, handle_name =  %s, shot_timeout = %f, shot_once_angle = %f, shot_continuous_speed = %f,"
        "stuck_inverse_time = %f, stuck_inverse_speed = %f, stuck_check_time = %f, stuck_check_speed = %f.",
        handleName.c_str(), shotTimeout, onceAngle_, continuousSpeed_, stuckInverseTime_, stuckInverseSpeed_, stuckInverseTime_, stuckInverseSpeed_);
    return true;
}

void FireController::update(const ros::Time& time, const ros::Duration& period)
{
    ControlMode nowControlMode;
    double targetSpeed = 0; /* 目标速度,只在速度环下有效 */
    switch (machineState_) {
        case MachineState::SHOT_ONCE:
            nowControlMode = ControlMode::POSITION;
            /* 判断是否达到目标条件 */
            if ((!lastChangeTargetTime_.is_zero() && ros::Time::now() - lastChangeTargetTime_ > ros::Duration(positionCheckTime_)) || std::fabs(targetPosition_ - handle_.getPosition()) < positionCheckError_) {
                machineState_  = MachineState::IDLE;
            }
            break;
        case MachineState::SHOT_CONTINUOUS:
            targetSpeed    = continuousSpeed_;
            nowControlMode = ControlMode::SPEED;
            break;
        case MachineState::IDLE:
            nowControlMode = ControlMode::SPEED;
            targetSpeed    = 0;
            break;
    }
    /* 当位置环和速度环切换时要清空参数 */
    if (lastControlMode_ != nowControlMode) {
        if (nowControlMode == ControlMode::POSITION) {
            positionPID_.reset();
        } else {
            speedPID_.reset();
        }
    }
    lastControlMode_ = nowControlMode;
    /* 防卡弹处理 */
    auto now = ros::Time::now();
    if (isEnableStuckCheck_ && (nowControlMode == ControlMode::POSITION || (nowControlMode == ControlMode::SPEED && targetSpeed != 0))) {
        if (handle_.getVelocity() < stuckCheckSpeed_) {
            if (startStuckTime_.is_zero()) {
                startStuckTime_ = now;
            }
        } else {
            startStuckTime_ = startStuckTime_.fromSec(0);
        }
        if (!startStuckTime_.is_zero() && now - startStuckTime_ > ros::Duration(stuckCheckTime_)) {
            nowControlMode = ControlMode::SPEED;
            targetSpeed    = -stuckInverseSpeed_;
            if (now - startStuckTime_ > ros::Duration(stuckInverseTime_ + stuckCheckTime_)) {
                ROS_WARN("Detected stuck, has resolved.");
                startStuckTime_ = startStuckTime_.fromSec(0);
            }
        }
    }
    /* PID计算与发布话题 */
    double error, output;
    if (nowControlMode == ControlMode::POSITION) {
        error  = targetPosition_ - handle_.getPosition();
        output = positionPID_.computeCommand(error, period);
        handle_.setCommand(output);
        if (positionStatePublisher_ && positionStatePublisher_->trylock()) {
            double _;
            bool antiwindup;
            positionPID_.getGains(
                positionStatePublisher_->msg_.p, positionStatePublisher_->msg_.i, positionStatePublisher_->msg_.d,
                positionStatePublisher_->msg_.i_clamp, _, antiwindup);
            positionStatePublisher_->msg_.antiwindup    = static_cast<char>(antiwindup);
            positionStatePublisher_->msg_.set_point     = targetPosition_;
            positionStatePublisher_->msg_.process_value = handle_.getPosition();
            positionStatePublisher_->msg_.error         = error;
            positionStatePublisher_->msg_.time_step     = period.toSec();
            positionStatePublisher_->msg_.command       = output;
            positionStatePublisher_->msg_.header.stamp  = time;
            positionStatePublisher_->msg_.header.seq++;
            positionStatePublisher_->unlockAndPublish();
        }
    } else {
        error  = targetSpeed - handle_.getVelocity();
        output = speedPID_.computeCommand(error, period);
        handle_.setCommand(output);
        if (speedStatePublisher_ && speedStatePublisher_->trylock()) {
            double _;
            bool antiwindup;
            speedPID_.getGains(
                speedStatePublisher_->msg_.p, speedStatePublisher_->msg_.i, speedStatePublisher_->msg_.d,
                speedStatePublisher_->msg_.i_clamp, _, antiwindup);
            speedStatePublisher_->msg_.antiwindup    = static_cast<char>(antiwindup);
            speedStatePublisher_->msg_.set_point     = targetSpeed;
            speedStatePublisher_->msg_.process_value = handle_.getVelocity();
            speedStatePublisher_->msg_.error         = error;
            speedStatePublisher_->msg_.time_step     = period.toSec();
            speedStatePublisher_->msg_.command       = output;
            speedStatePublisher_->msg_.header.stamp  = time;
            speedStatePublisher_->msg_.header.seq++;
            speedStatePublisher_->unlockAndPublish();
        }
    }
}

void FireController::starting(const ros::Time& time)
{
}

void FireController::stopping(const ros::Time& time)
{
}

}  // namespace fire_controller

PLUGINLIB_EXPORT_CLASS(fire_controller::FireController, controller_interface::ControllerBase);