#include <ros/ros.h>

#include <angles/angles.h>
#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <urdf/model.h>

#include "swerve_drive_controller/swerve_drive_controller.h"

namespace swerve_drive_controller
{
SwerveDriveController::SwerveDriveController()
{
}

bool SwerveDriveController::readConfig(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& node)
{
    node.param<double>("twist_timeout", twistTimeout_, 0.5);
    node.param<double>("publish_rate", publishRate_, 100);
    node.param<std::string>("base_frame_id", baseFrameID_, "base_link");
    node.param<std::string>("odom_frame_id", odomFrameID_, "odom");
    node.param<std::string>("twist_topic", twistTopic_, "/cmd_vel");
    node.param<std::string>("odom_topic", odomTopic_, "/odom");
    node.param<bool>("use_twist_stamped", isUseTwistStamped_, false);
    /* 解析URDF */
    urdf::Model urdfModel;
    if (!urdfModel.initParamWithNodeHandle("robot_description", node)) {
        ROS_FATAL("URDF parsed failed!");
        return false;
    }
    std::array<std::string, 4> jointNames = {"left_forward", "right_forward", "left_backward", "right_backward"};
    std::map<std::string, tf::Vector3> positionMap; /* 机器人关节-坐标数组 */
    for (size_t i = 0; i < jointNames.size(); i++) {
        std::string& jointName = jointNames[i];
        std::string realWheelJointName, realPivotJointName;
        if (!node.getParam("joint/" + jointName + "/wheel_name", realWheelJointName) || urdfModel.joints_.find(realWheelJointName) == urdfModel.joints_.end()) {
            ROS_FATAL("URDF parsed failed! The whell joint name in parameters server didn't exist in URDF or the parameters does not exist!");
            return false;
        }
        if (!node.getParam("joint/" + jointName + "/pivot_name", realPivotJointName) || urdfModel.joints_.find(realPivotJointName) == urdfModel.joints_.end()) {
            ROS_FATAL("URDF parsed failed! The whell joint name in parameters server didn't exist in URDF or the parameters does not exist!");
            return false;
        }
        joints_[jointName].wheelHandle           = hw->getHandle(realWheelJointName);
        joints_[jointName].pivotHandle           = hw->getHandle(realPivotJointName);
        urdf::JointConstSharedPtr wheelURDFJoint = urdfModel.getJoint(realWheelJointName);
        urdf::LinkConstSharedPtr jointLink       = urdfModel.getLink(wheelURDFJoint->child_link_name);
        if (jointLink->collision == nullptr || jointLink->collision->geometry == nullptr || jointLink->collision->geometry->type != urdf::Geometry::CYLINDER) {
            ROS_FATAL("URDF parsed failed! The collision of joint link isn't CYLINDER!");
            return false;
        }
        joints_[jointName].wheelTargetSpeed    = 0;
        joints_[jointName].pivotTargetPosition = 0;
        joints_[jointName].pivotTargetPositionShortest = 0;
        joints_[jointName].wheelMaxSpeed       = wheelURDFJoint->limits->velocity;
        joints_[jointName].wheelPerimeter      = (static_cast<urdf::Cylinder*>(jointLink->collision->geometry.get()))->radius * 2 * M_PI;
        ROS_INFO("Parsed from URDF, joint name = %s, wheel name = %s, pivot name = %s, wheel_perimeter = %f, max_speed = %f.", jointName.c_str(), realWheelJointName.c_str(), realPivotJointName.c_str(), joints_[jointName].wheelPerimeter, joints_[jointName].wheelMaxSpeed);
        /* 获取每个轮子相对base系的坐标,方便后面计算轮距和轴距 */
        tf::Transform transform(
            tf::Quaternion(wheelURDFJoint->parent_to_joint_origin_transform.rotation.x, wheelURDFJoint->parent_to_joint_origin_transform.rotation.y, wheelURDFJoint->parent_to_joint_origin_transform.rotation.z, wheelURDFJoint->parent_to_joint_origin_transform.rotation.w),
            tf::Vector3(wheelURDFJoint->parent_to_joint_origin_transform.position.x, wheelURDFJoint->parent_to_joint_origin_transform.position.y, wheelURDFJoint->parent_to_joint_origin_transform.position.z));
        urdf::JointConstSharedPtr parentJoint = wheelURDFJoint;
        while (parentJoint->parent_link_name != baseFrameID_) {
            urdf::LinkConstSharedPtr link_parent(urdfModel.getLink(parentJoint->parent_link_name));
            if (!link_parent || !link_parent->parent_joint) {
                ROS_ERROR("URDF parsed failed! Couldn't be retrieved from model description or his parent joint at %s.", parentJoint->parent_link_name.c_str());
                return false;
            }
            parentJoint = link_parent->parent_joint;
            transform *= tf::Transform(
                tf::Quaternion(parentJoint->parent_to_joint_origin_transform.rotation.x, parentJoint->parent_to_joint_origin_transform.rotation.y, parentJoint->parent_to_joint_origin_transform.rotation.z, parentJoint->parent_to_joint_origin_transform.rotation.w),
                tf::Vector3(parentJoint->parent_to_joint_origin_transform.position.x, parentJoint->parent_to_joint_origin_transform.position.y, parentJoint->parent_to_joint_origin_transform.position.z));
        }
        positionMap[jointName] = transform.getOrigin();
        /* 初始化PID控制器和状态发布 */
        if (!joints_[jointName].wheelPID.init(ros::NodeHandle(node, "joint/" + jointName + "/wheel_pid"))) {
            ROS_FATAL("Wheel PID Controller init failed at joint: %s.", jointName.c_str());
            return false;
        }
        joints_[jointName].wheelStatePublisher.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(node, "joint/" + jointName + "/wheel_state", 1000));
        if (!joints_[jointName].pivotPID.init(ros::NodeHandle(node, "joint/" + jointName + "/pivot_pid"))) {
            ROS_FATAL("Pivot PID Controller init failed at joint: %s.", jointName.c_str());
            return false;
        }
        joints_[jointName].pivotStatePublisher.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(node, "joint/" + jointName + "/pivot_state", 1000));
    }
    /* 判定是否是矩形 */
    if (tf::tfDistance(positionMap["left_forward"], positionMap["right_forward"]) != tf::tfDistance(positionMap["left_backward"], positionMap["right_backward"]) ||
        tf::tfDistance(positionMap["left_forward"], positionMap["left_backward"]) != tf::tfDistance(positionMap["right_forward"], positionMap["right_backward"]) ||
        tf::tfAngle(positionMap["left_forward"] - positionMap["left_backward"], positionMap["right_backward"] - positionMap["left_backward"]) - M_PI / 2 > 0.01) {
        ROS_FATAL("URDF parsed failed! Not all joints are in rectangular!");
        return false;
    }
    wheelTrack_ = tf::tfDistance(positionMap["left_forward"], positionMap["right_forward"]);
    wheelBase_  = tf::tfDistance(positionMap["left_forward"], positionMap["left_backward"]);
    ROS_INFO("Parsed from URDF, wheel_track = %f, wheel_base = %f.", wheelTrack_, wheelBase_);
    ROS_INFO("Pivot Drive Controller started!");
    return true;
}

bool SwerveDriveController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& node)
{
    if (!readConfig(hw, node)) return false;
    /* 初始化话题发布 */
    odomPublisher_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(node, odomTopic_, 1000));
    odomPublisher_->msg_.header.frame_id = odomFrameID_;
    odomPublisher_->msg_.child_frame_id  = baseFrameID_;
    tfPublisher_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(node, "/tf", 1000));
    tfPublisher_->msg_.transforms.resize(1);
    tfPublisher_->msg_.transforms[0].child_frame_id  = baseFrameID_;
    tfPublisher_->msg_.transforms[0].header.frame_id = odomFrameID_;
    /* 订阅话题 */
    if (isUseTwistStamped_) {
        twistSubscriber_ = node.subscribe<geometry_msgs::TwistStamped>(twistTopic_, 1000, &SwerveDriveController::twistStampedCallback, this);
    } else {
        twistSubscriber_ = node.subscribe<geometry_msgs::Twist>(twistTopic_, 1000, &SwerveDriveController::twistCallback, this);
    }
    twistTimeoutTimer_ = node.createTimer(ros::Duration(twistTimeout_), boost::bind(&SwerveDriveController::brake, this), true, true);
    return true;
}

void SwerveDriveController::inverseKinematics(const ros::Time& time, const ros::Duration& period)
{
    SpeedInfo* command = command_.readFromRT();
    tf::Vector3 commandVector(command->vx, command->vy, 0);
    double computeScale = 1;
    for (auto iter = joints_.begin(); iter != joints_.end(); ++iter) {
        tf::Vector3 wheelVector = commandVector;
        if (iter->first == "left_forward")
            wheelVector += tf::Vector3(command->vrz * -wheelTrack_, command->vrz * wheelBase_, 0);
        if (iter->first == "right_forward")
            wheelVector += tf::Vector3(command->vrz * wheelTrack_, command->vrz * wheelBase_, 0);
        if (iter->first == "left_backward")
            wheelVector += tf::Vector3(command->vrz * -wheelTrack_, command->vrz * -wheelBase_, 0);
        if (iter->first == "right_backward")
            wheelVector += tf::Vector3(command->vrz * wheelTrack_, command->vrz * -wheelBase_, 0);
        iter->second.wheelTargetSpeed    = wheelVector.length() / iter->second.wheelPerimeter * 2 * M_PI;
        iter->second.pivotTargetPosition = atan2(wheelVector.y(), wheelVector.x());
        double pivotError = angles::shortest_angular_distance(iter->second.pivotHandle.getPosition(), iter->second.pivotTargetPosition);
        double pivotErrorInverse = angles::shortest_angular_distance(iter->second.pivotHandle.getPosition(), iter->second.pivotTargetPosition + M_PI);
        iter->second.pivotTargetPositionShortest = std::abs(pivotError) < std::abs(pivotErrorInverse) ? iter->second.pivotTargetPosition : iter->second.pivotTargetPosition + M_PI;
        computeScale                     = std::min(iter->second.wheelMaxSpeed / abs(iter->second.wheelTargetSpeed), computeScale);
    }
    /* 万一某个轮子超过了最大速度,就整个底盘一起减速 */
    for (auto iter = joints_.begin(); iter != joints_.end(); ++iter) {
        iter->second.wheelTargetSpeed *= computeScale;
    }
}

void SwerveDriveController::forwardKinematics(const ros::Time& time, const ros::Duration& period)
{
    /* 运动解算 */
    odom_.x += odom_.vx * cos(odom_.rz) * period.toSec() + odom_.vy * sin(odom_.rz) * period.toSec();
    odom_.y += odom_.vx * sin(odom_.rz) * period.toSec() + odom_.vy * cos(odom_.rz) * period.toSec();
    odom_.rz += odom_.vrz * period.toSec();
    /* 换算成m/s */
    double vx = 0, vy = 0, vrz = 0;
    for (auto iter = joints_.begin(); iter != joints_.end(); ++iter) {
        vx += joints_[iter->first].wheelHandle.getVelocity() / (2 * M_PI) * joints_[iter->first].wheelPerimeter * std::cos(joints_[iter->first].pivotHandle.getPosition());
        vy += joints_[iter->first].wheelHandle.getVelocity() / (2 * M_PI) * joints_[iter->first].wheelPerimeter * std::sin(joints_[iter->first].pivotHandle.getPosition());
        vrz += joints_[iter->first].wheelHandle.getVelocity() / (2 * M_PI) * joints_[iter->first].wheelPerimeter * -std::cos(joints_[iter->first].pivotHandle.getPosition() + std::atan2(wheelBase_, wheelTrack_));
    }
    odom_.vrz = vx / 4;
    odom_.vy  = vy / 4;
    odom_.vx  = vrz / 4 * std::sqrt(std::pow(wheelBase_, 2) + std::pow(wheelTrack_, 2)) / 2;
}

void SwerveDriveController::update(const ros::Time& time, const ros::Duration& period)
{
    /* PID计算与发布话题 */
    for (auto iter = joints_.begin(); iter != joints_.end(); ++iter) {
        /* 计算轮子PID */
        double output;
        double shortestPivotError = angles::shortest_angular_distance(iter->second.pivotHandle.getPosition(), iter->second.pivotTargetPositionShortest);
        double wheelError  = iter->second.wheelTargetSpeed - iter->second.wheelHandle.getVelocity();
        wheelError *= std::cos(shortestPivotError);
        output = iter->second.wheelPID.computeCommand(wheelError, period);
        iter->second.wheelHandle.setCommand(output);
        if (iter->second.wheelStatePublisher && iter->second.wheelStatePublisher->trylock()) {
            double _;
            bool antiwindup;
            iter->second.wheelPID.getGains(
                iter->second.wheelStatePublisher->msg_.p, iter->second.wheelStatePublisher->msg_.i, iter->second.wheelStatePublisher->msg_.d,
                iter->second.wheelStatePublisher->msg_.i_clamp, _, antiwindup);
            iter->second.wheelStatePublisher->msg_.antiwindup    = static_cast<char>(antiwindup);
            iter->second.wheelStatePublisher->msg_.set_point     = iter->second.wheelTargetSpeed;
            iter->second.wheelStatePublisher->msg_.process_value = iter->second.wheelHandle.getVelocity();
            iter->second.wheelStatePublisher->msg_.error         = wheelError;
            iter->second.wheelStatePublisher->msg_.time_step     = period.toSec();
            iter->second.wheelStatePublisher->msg_.command       = output;
            iter->second.wheelStatePublisher->msg_.header.stamp  = time;
            iter->second.wheelStatePublisher->msg_.header.seq++;
            iter->second.wheelStatePublisher->unlockAndPublish();
        }
        /* 计算转向PID */
        output = iter->second.pivotPID.computeCommand(shortestPivotError, period);
        iter->second.pivotHandle.setCommand(output);
        if (iter->second.pivotStatePublisher && iter->second.pivotStatePublisher->trylock()) {
            double _;
            bool antiwindup;
            iter->second.pivotPID.getGains(
                iter->second.pivotStatePublisher->msg_.p, iter->second.pivotStatePublisher->msg_.i, iter->second.pivotStatePublisher->msg_.d,
                iter->second.pivotStatePublisher->msg_.i_clamp, _, antiwindup);
            iter->second.pivotStatePublisher->msg_.antiwindup    = static_cast<char>(antiwindup);
            iter->second.pivotStatePublisher->msg_.set_point     = iter->second.pivotTargetPositionShortest;
            iter->second.pivotStatePublisher->msg_.process_value = iter->second.pivotHandle.getPosition();
            iter->second.pivotStatePublisher->msg_.error         = shortestPivotError;
            iter->second.pivotStatePublisher->msg_.time_step     = period.toSec();
            iter->second.pivotStatePublisher->msg_.command       = output;
            iter->second.pivotStatePublisher->msg_.header.stamp  = time;
            iter->second.pivotStatePublisher->msg_.header.seq++;
            iter->second.pivotStatePublisher->unlockAndPublish();
        }
    }
    /* 发布话题 */
    lastPublishDuration_ += period;
    if (lastPublishDuration_.toSec() >= 1 / publishRate_) {
        lastPublishDuration_ = ros::Duration(0);
        tf::Transform position(tf::createQuaternionFromYaw(odom_.rz), tf::Vector3(odom_.x, odom_.y, 0));
        if (odomPublisher_ && odomPublisher_->trylock()) {
            odomPublisher_->msg_.header.stamp = time;
            odomPublisher_->msg_.header.seq++;
            tf::poseTFToMsg(position, odomPublisher_->msg_.pose.pose);
            tf::vector3TFToMsg(tf::Vector3(odom_.vx, odom_.vy, 0), odomPublisher_->msg_.twist.twist.linear);
            tf::vector3TFToMsg(tf::Vector3(0, 0, odom_.vrz), odomPublisher_->msg_.twist.twist.angular);
            odomPublisher_->unlockAndPublish();
        }
        if (tfPublisher_ && tfPublisher_->trylock()) {
            tfPublisher_->msg_.transforms[0].header.stamp = time;
            tfPublisher_->msg_.transforms[0].header.seq++;
            tf::transformTFToMsg(position, tfPublisher_->msg_.transforms[0].transform);
            tfPublisher_->unlockAndPublish();
        }
    }
}

void SwerveDriveController::twistCallback(const geometry_msgs::TwistConstPtr& msg)
{
    twistTimeoutTimer_.stop();
    twistTimeoutTimer_.start();
    command_.writeFromNonRT(SpeedInfo({msg->linear.x, msg->linear.y, msg->angular.z}));
}

void SwerveDriveController::twistStampedCallback(const geometry_msgs::TwistStampedConstPtr& msg)
{
    twistTimeoutTimer_.stop();
    twistTimeoutTimer_.start();
    command_.writeFromNonRT(SpeedInfo({msg->twist.linear.x, msg->twist.linear.y, msg->twist.angular.z}));
    if (ros::Time::now() - msg->header.stamp > ros::Duration(0.1)) {
        ROS_WARN("Twist stamped message arrives too late!");
    }
}

void SwerveDriveController::starting(const ros::Time& time)
{
}

void SwerveDriveController::stopping(const ros::Time& time)
{
    /* 停机 */
    for (auto iter = joints_.begin(); iter != joints_.end(); ++iter) {
        iter->second.wheelHandle.setCommand(0);
        iter->second.pivotHandle.setCommand(0);
    }
}

void SwerveDriveController::brake()
{
    ROS_WARN("Braking...");
    command_.writeFromNonRT(SpeedInfo({0, 0, 0}));
}

}  // namespace swerve_drive_controller

PLUGINLIB_EXPORT_CLASS(swerve_drive_controller::SwerveDriveController, controller_interface::ControllerBase);