#ifndef MECANUM_CHASSIS_CONTROLLER_MECANUM_CHASSIS_CONTROLLER_H_
#define MECANUM_CHASSIS_CONTROLLER_MECANUM_CHASSIS_CONTROLLER_H_

#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_buffer.h>
#include <tf/tfMessage.h>

namespace swerve_drive_controller
{
class SwerveDriveController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
  private:
    typedef struct {
        hardware_interface::JointHandle wheelHandle;                                                                /* 轮子关节的Handle */
        hardware_interface::JointHandle pivotHandle;                                                                /* 转向关节的Handle */
        double wheelTargetSpeed;                                                                                    /* 轮子目标速度 */
        double wheelMaxSpeed;                                                                                       /* 轮子最大速度 */
        double wheelPerimeter;                                                                                      /* 轮子周长,单位米 */
        double pivotTargetPosition;                                                                                 /* 转向目标位置 */
        double pivotTargetPositionShortest;                                                                         /* 最小转向目标点目标位置 */
        control_toolbox::Pid wheelPID;                                                                              /* 轮子PID控制器 */
        control_toolbox::Pid pivotPID;                                                                              /* 转向PID控制器 */
        std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>> wheelStatePublisher; /* 轮子信息发布 */
        std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>> pivotStatePublisher; /* 转向信息发布 */
    } JointInfo;

    typedef struct {
        double vx;  /* X轴线性速度 */
        double vy;  /* X轴线性速度 */
        double vrz; /* Z轴旋转速度 */
    } SpeedInfo;

    typedef struct
    {
        double x;   /* X轴位置 */
        double y;   /* Y轴位置 */
        double rz;  /* Z轴旋转位置 */
        double vx;  /* X轴速度 */
        double vy;  /* Y轴速度 */
        double vrz; /* Z轴旋转速度 */
    } OdomInfo;

    double wheelTrack_;                                                                    /* 轮距,单位米 */
    double wheelBase_;                                                                     /* 轴距,单位米 */
    std::map<std::string, JointInfo> joints_;                                              /* 轮子信息 */
    double twistTimeout_;                                                                  /* 速度控制信息超时时间 */
    double publishRate_;                                                                   /* 发布里程计信息和TF变换的频率,PID信息发送不受影响 */
    std::string baseFrameID_;                                                              /* 机器人坐标系ID */
    std::string odomFrameID_;                                                              /* 里程计坐标系ID */
    std::string twistTopic_;                                                               /* twist话题路径 */
    std::string odomTopic_;                                                                /* 里程计话题路径 */
    bool isUseTwistStamped_;                                                               /* 是否使用TwistStamped信息替代Twist */
    realtime_tools::RealtimeBuffer<SpeedInfo> command_;                                    /* 控制信息 */
    OdomInfo odom_ = {0};                                                                  /* 机器人里程计信息 */
    std::unique_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odomPublisher_; /* 里程计信息发布 */
    std::unique_ptr<realtime_tools::RealtimePublisher<tf::tfMessage>> tfPublisher_;        /* TF信息发布 */
    ros::Subscriber twistSubscriber_;                                                      /* Twist或TwistStamped话题订阅 */
    ros::Timer twistTimeoutTimer_;                                                         /* Twist或TwistStamped信息超时计时器 */
    ros::Duration lastPublishDuration_;                                                    /* 上次发送里程计信息和TF变换信息的间隔 */

    /**
     * 收到速度话题回调
     * 
     * @param msg 消息
     */
    void twistCallback(const geometry_msgs::TwistConstPtr& msg);

    /**
     * 收到Stamped速度话题回调
     * 
     * @param msg 消息
     */
    void twistStampedCallback(const geometry_msgs::TwistStampedConstPtr& msg);

  public:
    SwerveDriveController();

    /**
     * 读取配置
     * 
     * @param hw   关节接口
     * @param node ROS节点
     */
    bool readConfig(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& node);

    /**
     * 运动学解算
     * 
     * @param time 当前时间
     * @param period 间隔
     */
    void forwardKinematics(const ros::Time& time, const ros::Duration& period);

    /**
     * 逆运动学解算
     * 
     * @param time 当前时间
     * @param period 间隔
     */
    void inverseKinematics(const ros::Time& time, const ros::Duration& period);

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

    /**
     * 刹车
     * 
     */
    void brake();
};

}  // namespace swerve_drive_controller

#endif