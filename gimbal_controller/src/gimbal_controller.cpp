#include <angles/angles.h>
#include <control_msgs/JointControllerState.h>
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <robot_toolbox/tool.h>
#include <sensor_msgs/Imu.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <urdf/model.h>

#include "gimbal_controller/gimbal_controller.h"

namespace gimbal_controller
{
GimbalController::GimbalController()
{
}

bool GimbalController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& node)
{
    /* 解析URDF */
    urdf::Model urdfModel;
    if (!urdfModel.initParamWithNodeHandle("robot_description", node)) {
        ROS_FATAL("URDF parsed failed!");
        return false;
    }
    std::array<std::string, 2> jointNames = {"yaw", "pitch"};
    for (size_t i = 0; i < jointNames.size(); i++) {
        std::string& jointName = jointNames[i];
        std::string realJointName;
        if (!node.getParam("joint/" + jointName + "/name", realJointName) || urdfModel.joints_.find(realJointName) == urdfModel.joints_.end()) {
            ROS_FATAL("URDF parsed failed! The joint name in parameters server didn't exist in URDF or the parameters does not exist!");
            return false;
        }
        ROS_INFO("Parsed from parameters, name = %s, joint = %s", jointName.c_str(), realJointName.c_str());
        joints_[jointName].handle         = hw->getHandle(realJointName);
        joints_[jointName].isContinuous   = urdfModel.joints_[realJointName]->type == urdf::Joint::CONTINUOUS;
        joints_[jointName].targetPosition = 0;
        /* 解析电机类型与限位 */
        if (!joints_[jointName].isContinuous) {
            joints_[jointName].limitUpper = urdfModel.joints_[realJointName]->limits->upper;
            joints_[jointName].limitLower = urdfModel.joints_[realJointName]->limits->lower;
        }
        if (!joints_[jointName].pid.init(ros::NodeHandle(node, "joint/" + jointName + "/pid"))) {
            ROS_FATAL("PID Controller init failed at joint: yaw.");
            return false;
        }
        joints_[jointName].statePublisher.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(node, "joint/" + jointName + "/state", 1000));
    }
    /* 解析参数 */
    std::string imuTopicName;
    node.param<bool>("enable_imu", isIMUEnable_, false);
    node.param<std::string>("imu_topic", imuTopicName, "/imu");
    node.param<std::string>("base_link", baseLink_, "base_link");
    node.param<std::string>("world_link", worldLink_, "odom");
    if (isIMUEnable_) {
        imuSubscriber_ = node.subscribe<sensor_msgs::Imu>(imuTopicName, 1000, &GimbalController::imuCallback, this);
    }
    /* 接收指令 */
    yawCommandSubscriber_   = node.subscribe<robot_msgs::Float64Stamped>("yaw/command", 1000, boost::bind(&GimbalController::targetPositionCallback, this, boost::ref(targetYawAngle_), _1));
    pitchCommandSubscriber_ = node.subscribe<robot_msgs::Float64Stamped>("pitch/command", 1000, boost::bind(&GimbalController::targetPositionCallback, this, boost::ref(targetPitchAngle_), _1));
    ROS_INFO("Gimbal Controller started, enable_imu = %d, imu_topic = %s, world link = %s, base link = %s.", isIMUEnable_, imuTopicName.c_str(), worldLink_.c_str(), baseLink_.c_str());
    return true;
}

void GimbalController::targetPositionCallback(double& var, const robot_msgs::Float64StampedConstPtr& msg)
{
    var = msg->result;
}

void GimbalController::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
    tf::Quaternion quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::StampedTransform imuToBaseLink;
    tfListener_.lookupTransform(baseLink_, msg->header.frame_id, ros::Time(0), imuToBaseLink);
    quaternion = imuToBaseLink * quaternion;
    tf::Matrix3x3 imuMatrix(quaternion);
    double _;
    imuMatrix.getRPY(_, nowBaseLinkPitchAngle_, nowBaseLinkYawAngle_);
}

void GimbalController::update(const ros::Time& time, const ros::Duration& period)
{
    if (!isIMUEnable_) {
        tf::StampedTransform worldToBaseLink;
        tfListener_.lookupTransform(baseLink_, worldLink_, ros::Time(0), worldToBaseLink);
        tf::Quaternion quaternion = worldToBaseLink.getRotation();
        tf::Matrix3x3 worldMatrix(quaternion);
        double _;
        worldMatrix.getRPY(_, nowBaseLinkPitchAngle_, nowBaseLinkYawAngle_);
    }
    /* 误差较小时保存IMU初始信息,0.01弧度约等于1度 */
    if (!isAlreadyCalibrate_ && joints_["yaw"].error < 0.01 && joints_["pitch"].error < 0.01) {
        initBaseLinkYawAngle_    = nowBaseLinkYawAngle_;
        initBaseLinkPitchAngle_  = nowBaseLinkPitchAngle_;
        isAlreadyCalibrate_ = true;
    }
    /* 根据坐标系偏差信息与指令计算目标电机位置 */
    joints_["yaw"].targetPosition   = initBaseLinkYawAngle_ - nowBaseLinkYawAngle_ + targetYawAngle_;
    joints_["pitch"].targetPosition = initBaseLinkPitchAngle_ - nowBaseLinkPitchAngle_ + targetPitchAngle_;
    /* 电机闭环控制 */
    for (auto iter = joints_.begin(); iter != joints_.end(); ++iter) {
        /* 计算偏差 */
        if (iter->second.isContinuous) {
            iter->second.error = angles::shortest_angular_distance(iter->second.handle.getPosition(), iter->second.targetPosition);
        } else {
            LIMIT(iter->second.targetPosition, iter->second.limitLower, iter->second.limitUpper);
            angles::shortest_angular_distance_with_large_limits(
                iter->second.handle.getPosition(),
                iter->second.targetPosition,
                iter->second.limitLower,
                iter->second.limitUpper,
                iter->second.error);
        }
        /* PID计算与发布话题 */
        double output = iter->second.pid.computeCommand(iter->second.error, period);
        iter->second.handle.setCommand(output);
        if (iter->second.statePublisher && iter->second.statePublisher->trylock()) {
            double _;
            bool antiwindup;
            iter->second.pid.getGains(
                iter->second.statePublisher->msg_.p, iter->second.statePublisher->msg_.i, iter->second.statePublisher->msg_.d,
                iter->second.statePublisher->msg_.i_clamp, _, antiwindup);
            iter->second.statePublisher->msg_.antiwindup    = static_cast<char>(antiwindup);
            iter->second.statePublisher->msg_.set_point     = iter->second.targetPosition;
            iter->second.statePublisher->msg_.process_value = iter->second.handle.getPosition();
            iter->second.statePublisher->msg_.error         = iter->second.error;
            iter->second.statePublisher->msg_.time_step     = period.toSec();
            iter->second.statePublisher->msg_.command       = output;
            iter->second.statePublisher->msg_.header.stamp  = time;
            iter->second.statePublisher->msg_.header.seq++;
            iter->second.statePublisher->unlockAndPublish();
        }
    }
}

void GimbalController::starting(const ros::Time& time)
{
}

void GimbalController::stopping(const ros::Time& time)
{
    for (auto iter = joints_.begin(); iter != joints_.end(); ++iter) {
        iter->second.handle.setCommand(0);
    }
}

}  // namespace gimbal_controller

PLUGINLIB_EXPORT_CLASS(gimbal_controller::GimbalController, controller_interface::ControllerBase);