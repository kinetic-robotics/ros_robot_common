#ifndef GIMBAL_CONTROLLER_H_
#define GIMBAL_CONTROLLER_H_

#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/Imu.h>
#include <robot_msgs/Float64Stamped.h>

namespace gimbal_controller
{
class GimbalController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
  private:
    typedef struct {
        hardware_interface::JointHandle handle;                                                                /* 电机句柄 */
        control_toolbox::Pid pid;                                                                              /* PID */
        bool isContinuous;                                                                                     /* 是否可以连续旋转 */
        double limitUpper;                                                                                     /* 限位 */
        double limitLower;                                                                                     /* 限位 */
        double targetPosition;                                                                                 /* 目标位置 */
        double error;                                                                                          /* 偏差 */
        std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>> statePublisher; /* 信息发布 */
    } JointInfo;
    ros::Subscriber imuSubscriber_;           /* IMU话题订阅 */
    ros::Subscriber yawCommandSubscriber_;    /* YAW轴目标位置话题订阅 */
    ros::Subscriber pitchCommandSubscriber_;  /* PITCH轴目标位置话题订阅 */
    std::map<std::string, JointInfo> joints_; /* 电机信息 */
    bool isAlreadyCalibrate_  = false;        /* 是否已经完成云台校准过程 */
    double nowIMUYawAngle_    = 0;            /* 当前IMU yaw角度 */
    double nowIMUPitchAngle   = 0;            /* 当前IMU pitch */
    double initIMUYawAngle_   = 0;            /* 初始IMU yaw角度 */
    double initIMUPitchAngle_ = 0;            /* 初始IMU pitch */
    double targetYawAngle_    = 0;            /* 目标yaw角度 */
    double targetPitchAngle_  = 0;            /* 目标pitch角度 */

    /**
     * IMU话题回调
     * 
     * @param msg 消息
     */
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);

    /**
     * 目标位置话题回调
     * 
     * @param var 变量
     * @param msg 消息
     */
    void targetPositionCallback(double& var, const robot_msgs::Float64StampedConstPtr& msg);

  public:
    GimbalController();

    /**
     * ROS Control的Controller Init接口
     */
    bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& node);

    /**
     * ROS Control的Controller Update接口
     */
    void update(const ros::Time& time, const ros::Duration& period);

    /**
     * ROS Control的Controller Starting接口
     */
    void starting(const ros::Time& time);

    /**
     * ROS Control的Controller Stopping接口
     */
    void stopping(const ros::Time& time);
};
}  // namespace gimbal_controller

#endif