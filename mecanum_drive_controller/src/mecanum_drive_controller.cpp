#include <ros/ros.h>

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

#include "mecanum_drive_controller/mecanum_drive_controller.h"

namespace mecanum_drive_controller
{
MecanumDriveController::MecanumDriveController()
{
}

bool MecanumDriveController::readConfig(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& node)
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
        std::string realJointName;
        if (!node.getParam("joint/" + jointName + "/name", realJointName) || urdfModel.joints_.find(realJointName) == urdfModel.joints_.end()) {
            ROS_FATAL("URDF parsed failed! The joint name in parameters server didn't exist in URDF or the parameters does not exist!");
            return false;
        }
        joints_[jointName].handle           = hw->getHandle(realJointName);
        urdf::JointConstSharedPtr urdfJoint = urdfModel.getJoint(realJointName);
        urdf::LinkConstSharedPtr jointLink  = urdfModel.getLink(urdfJoint->child_link_name);
        if (jointLink->collision == nullptr || jointLink->collision->geometry == nullptr || jointLink->collision->geometry->type != urdf::Geometry::CYLINDER) {
            ROS_FATAL("URDF parsed failed! The collision of joint link isn't CYLINDER!");
            return false;
        }
        joints_[jointName].targetSpeed    = 0;
        joints_[jointName].maxSpeed       = urdfJoint->limits->velocity;
        joints_[jointName].wheelPerimeter = (static_cast<urdf::Cylinder*>(jointLink->collision->geometry.get()))->radius * 2 * M_PI;
        ROS_INFO("Parsed from URDF, name = %s, joint_name = %s, wheel_perimeter = %f, max_speed = %f.", jointName.c_str(), realJointName.c_str(), joints_[jointName].wheelPerimeter, joints_[jointName].maxSpeed);
        tf::Transform transform(
            tf::Quaternion(urdfJoint->parent_to_joint_origin_transform.rotation.x, urdfJoint->parent_to_joint_origin_transform.rotation.y, urdfJoint->parent_to_joint_origin_transform.rotation.z, urdfJoint->parent_to_joint_origin_transform.rotation.w),
            tf::Vector3(urdfJoint->parent_to_joint_origin_transform.position.x, urdfJoint->parent_to_joint_origin_transform.position.y, urdfJoint->parent_to_joint_origin_transform.position.z));
        urdf::JointConstSharedPtr parentJoint = urdfJoint;
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
        if (!joints_[jointName].pid.init(ros::NodeHandle(node, "joint/" + jointName + "/pid"))) {
            ROS_FATAL("PID Controller init failed at joint: %s.", jointName.c_str());
            return false;
        }
        joints_[jointName].statePublisher.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(node, "joint/" + jointName + "/state", 1000));
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
    ROS_INFO("Mecanum Drive Controller started!");
    return true;
}

bool MecanumDriveController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& node)
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
        twistSubscriber_ = node.subscribe<geometry_msgs::TwistStamped>(twistTopic_, 1000, &MecanumDriveController::twistStampedCallback, this);
    } else {
        twistSubscriber_ = node.subscribe<geometry_msgs::Twist>(twistTopic_, 1000, &MecanumDriveController::twistCallback, this);
    }
    twistTimeoutTimer_ = node.createTimer(ros::Duration(twistTimeout_), boost::bind(&MecanumDriveController::brake, this), true, true);
    return true;
}

void MecanumDriveController::update(const ros::Time& time, const ros::Duration& period)
{
    /* 逆运动解算 */
    SpeedInfo* command                    = command_.readFromRT();
    joints_["left_forward"].targetSpeed   = ( command->vx - command->vy - command->vrz * (wheelTrack_ + wheelBase_) / 2) / joints_["left_forward"].wheelPerimeter  * 2 * M_PI;
    joints_["right_forward"].targetSpeed  = (-command->vx - command->vy - command->vrz * (wheelTrack_ + wheelBase_) / 2) / joints_["right_forward"].wheelPerimeter * 2 * M_PI;
    joints_["left_backward"].targetSpeed  = ( command->vx + command->vy - command->vrz * (wheelTrack_ + wheelBase_) / 2) / joints_["left_backward"].wheelPerimeter * 2 * M_PI;
    joints_["right_backward"].targetSpeed = (-command->vx + command->vy - command->vrz * (wheelTrack_ + wheelBase_) / 2) / joints_["right_backward"].wheelPerimeter* 2 * M_PI;
    /* 万一某个轮子超过了最大速度,就整个底盘一起减速 */
    double computeScale = 1;
    for (auto iter = joints_.begin(); iter != joints_.end(); iter++) {
        computeScale = std::min(iter->second.maxSpeed / abs(iter->second.targetSpeed), computeScale);
    }
    for (auto iter = joints_.begin(); iter != joints_.end(); iter++) {
        iter->second.targetSpeed *= computeScale;
    }
    /* 运动解算 */
    odom_.x += odom_.vx * cos(odom_.rz) * period.toSec() + odom_.vy * sin(odom_.rz) * period.toSec();
    odom_.y += odom_.vx * sin(odom_.rz) * period.toSec() + odom_.vy * cos(odom_.rz) * period.toSec();
    odom_.rz += odom_.vrz * period.toSec();
    /* 换算成m/s */
    double leftForwardJointSpeed   = joints_["left_forward"].handle.getVelocity()   / (2 * M_PI) * joints_["left_forward"].wheelPerimeter;
    double rightForwardJointSpeed  = joints_["right_forward"].handle.getVelocity()  / (2 * M_PI) * joints_["right_forward"].wheelPerimeter;
    double leftBackwardJointSpeed  = joints_["left_backward"].handle.getVelocity()  / (2 * M_PI) * joints_["left_backward"].wheelPerimeter;
    double rightBackwardJointSpeed = joints_["right_backward"].handle.getVelocity() / (2 * M_PI) * joints_["right_backward"].wheelPerimeter;
    odom_.vrz = (-leftForwardJointSpeed  - rightForwardJointSpeed  - leftBackwardJointSpeed - rightBackwardJointSpeed) / (4 * (wheelTrack_ + wheelBase_) / 2);
    odom_.vy  = ( leftBackwardJointSpeed + rightBackwardJointSpeed - leftForwardJointSpeed  - rightForwardJointSpeed) / 4;
    odom_.vx  = ( leftForwardJointSpeed  + leftBackwardJointSpeed  - rightForwardJointSpeed - rightBackwardJointSpeed) / 4;
    /* PID计算与发布话题 */
    for (auto iter = joints_.begin(); iter != joints_.end(); iter++) {
        double error, output;
        error  = iter->second.targetSpeed - iter->second.handle.getVelocity();
        output = iter->second.pid.computeCommand(error, period);
        iter->second.handle.setCommand(output);
        if (iter->second.statePublisher && iter->second.statePublisher->trylock()) {
            double _;
            bool antiwindup;
            iter->second.pid.getGains(
                iter->second.statePublisher->msg_.p, iter->second.statePublisher->msg_.i, iter->second.statePublisher->msg_.d,
                iter->second.statePublisher->msg_.i_clamp, _, antiwindup);
            iter->second.statePublisher->msg_.antiwindup    = static_cast<char>(antiwindup);
            iter->second.statePublisher->msg_.set_point     = iter->second.targetSpeed;
            iter->second.statePublisher->msg_.process_value = iter->second.handle.getVelocity();
            iter->second.statePublisher->msg_.error         = error;
            iter->second.statePublisher->msg_.time_step     = period.toSec();
            iter->second.statePublisher->msg_.command       = output;
            iter->second.statePublisher->msg_.header.stamp  = time;
            iter->second.statePublisher->msg_.header.seq++;
            iter->second.statePublisher->unlockAndPublish();
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

void MecanumDriveController::twistCallback(const geometry_msgs::TwistConstPtr& msg)
{
    twistTimeoutTimer_.stop();
    twistTimeoutTimer_.start();
    command_.writeFromNonRT(SpeedInfo({msg->linear.x, msg->linear.y, msg->angular.z}));
}

void MecanumDriveController::twistStampedCallback(const geometry_msgs::TwistStampedConstPtr& msg)
{
    twistTimeoutTimer_.stop();
    twistTimeoutTimer_.start();
    command_.writeFromNonRT(SpeedInfo({msg->twist.linear.x, msg->twist.linear.y, msg->twist.angular.z}));
    if (ros::Time::now() - msg->header.stamp > ros::Duration(0.1)) {
        ROS_WARN("Twist stamped message arrives too late!");
    }
}

void MecanumDriveController::starting(const ros::Time& time)
{
}

void MecanumDriveController::stopping(const ros::Time& time)
{
    /* 停机 */
    for (auto iter = joints_.begin(); iter != joints_.end(); iter++) {
        iter->second.handle.setCommand(0);
    }
}

void MecanumDriveController::brake()
{
    ROS_WARN("Braking...");
    command_.writeFromNonRT(SpeedInfo({0, 0, 0}));
}

}  // namespace mecanum_drive_controller

PLUGINLIB_EXPORT_CLASS(mecanum_drive_controller::MecanumDriveController, controller_interface::ControllerBase);