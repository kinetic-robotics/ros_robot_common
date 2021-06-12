#ifndef FIRE_CONTROLLER_FIRE_CONTROLLER_H_
#define FIRE_CONTROLLER_FIRE_CONTROLLER_H_

#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <robot_msgs/BoolStamped.h>
#include <robot_msgs/EmptyStamped.h>

#include "fire_controller/FireControllerConfig.h"

namespace fire_controller
{
class FireController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
  private:
    hardware_interface::JointHandle handle_;                                                                        /* 电机句柄 */
    control_toolbox::Pid speedPID_;                                                                                 /* 速度环PID控制器 */
    control_toolbox::Pid positionPID_;                                                                              /* 位置环PID控制器 */
    std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>> speedStatePublisher_;    /* 速度环信息发布 */
    std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>> positionStatePublisher_; /* 位置环信息发布 */
    ros::Subscriber shotOnceSubscriber_;                                                                            /* 单次射击话题订阅 */
    ros::Subscriber shotContinousStartSubscriber_;                                                                  /* 连续射击开始话题订阅 */
    ros::Subscriber shotContinousStopSubscriber_;                                                                   /* 连续射击停止话题订阅 */
    ros::Subscriber loadingKeySubscriber_;                                                                          /* 微动开关按钮话题订阅 */
    std::string loadingKeyTopic_;                                                                                   /* 微动开关按钮话题 */
    bool isEnableAutoLoading_;                                                                                      /* 是否开启自动上弹 */
    double autoLoadingSpeed_;                                                                                       /* 自动上弹时的速度 */
    bool isNowLoadingKeyPress_;                                                                                     /* 微动开关是否按下 */
    double continuousSpeed_;                                                                                        /* 连发模式下的转动速度 */
    double onceAngle_;                                                                                              /* 单发模式下转动的角度 */
    double stuckInverseSpeed_;                                                                                      /* 卡弹反转速度,单位弧度每秒 */
    double stuckInverseTime_;                                                                                       /* 卡弹反转时间,单位秒 */
    double stuckCheckTime_;                                                                                         /* 卡弹判断时间,单位秒 */
    double stuckCheckSpeed_;                                                                                        /* 卡弹判断时间 */
    double positionCheckTime_;                                                                                      /* 位置环到达目标时间,超过该时间时认为达到目标 */
    double positionCheckError_;                                                                                     /* 位置环到达目标允许误差,低于该误差时认为达到目标 */
    bool isEnableStuckCheck_;                                                                                       /* 是否启用卡弹检测 */
    double targetPosition_;                                                                                         /* 目标位置,只在位置环下有效 */
    ros::Time startStuckTime_;                                                                                      /* 开始卡弹的时间 */
    ros::Time lastChangeTargetTime_;                                                                                /* 上次位置环变更目标的时间 */
    dynamic_reconfigure::Server<FireControllerConfig> dynamicReconfigureServer_;                                    /* 动态配置服务器 */
    dynamic_reconfigure::Server<FireControllerConfig>::CallbackType dynamicReconfigureServerCallback_;              /* 动态配置服务器回调类型 */

    enum class ControlMode {
        POSITION,       /* 位置环 */
        SPEED           /* 速度环 */
    } lastControlMode_; /* 上一次控制模式 */

    enum class MachineState {
        IDLE            = 0,              /* 空闲 */
        SHOT_ONCE       = 1,              /* 单发 */
        SHOT_CONTINUOUS = 2               /* 连发 */
    } machineState_ = MachineState::IDLE; /* 主状态机 */

    /**
     * 收到微动开关信息话题回调
     * 
     * @param msg 消息
     */
    void loadingKeyStateCallback(const robot_msgs::BoolStampedConstPtr& msg);

    /**
     * 收到单发话题回调
     * 
     * @param msg 消息
     */
    void shotOnceCallback(const robot_msgs::EmptyStampedConstPtr& msg);

    /**
     * 收到连射开始话题回调
     * 
     * @param msg 消息
     */
    void shotContinousStartCallback(const robot_msgs::EmptyStampedConstPtr& msg);

    /**
     * 收到连射停止话题回调
     * 
     * @param msg 消息
     */
    void shotContinousStopCallback(const robot_msgs::EmptyStampedConstPtr& msg);

    /**
     * 动态配置回调
     * 
     * @param config 配置
     * @param level 级别
     */
    void dynamicReconfigureCallback(FireControllerConfig& config, uint32_t level);

  public:
    FireController();

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

}  // namespace fire_controller

#endif