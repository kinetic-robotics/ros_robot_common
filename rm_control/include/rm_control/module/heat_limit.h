#ifndef RM_CONTROL_MODULE_SHOT_H_
#define RM_CONTROL_MODULE_SHOT_H_

#include <ros/ros.h>

#include <rm_referee_controller/PowerHeat.h>
#include <rm_referee_controller/RobotStatus.h>
#include <robot_msgs/BoolStamped.h>

#include "rm_control/module/module.h"

namespace rm_control
{
class HeatLimitModule: public ModuleInterface
{
  private:
    ros::NodeHandle& node_;                  /* 节点 */
    ros::NodeHandle& nodeParam_;             /* 参数节点 */
    std::string heatTopic_;                  /* 裁判系统热量话题名称 */
    ros::Subscriber heatSubscriber_;         /* 裁判系统热量话题订阅 */
    std::string onlineTopic_;                /* 裁判系统是否在线话题名称 */
    ros::Subscriber onlineSubscriber_;       /* 裁判系统是否在线订阅者 */
    ros::Subscriber robotStatusSubscriber_;  /* 裁判系统机器人状态信息订阅者 */
    std::string robotStatusTopic_;           /* 裁判系统机器人状态信息话题 */
    bool isOnline_                  = false; /* 遥控器是否在线 */
    int nowHeat_                    = 0;     /* 当前热量 */
    int nowMaxHeat_                 = 0;     /* 当前最大热量 */
    int shotOnceThresholdHeat_      = 0;     /* 单发模式剩余热量阈值 */
    int shotContinousThresholdHeat_ = 0;     /* 连发模式剩余热量阈值 */
    bool isNowContinousMode_        = false; /* 当是否在连发模式 */
    enum class ShooterType {
        FIRST17MM  = 0,
        SECOND17MM = 1,
        FIRST42MM  = 2,
    } shooterType_ = ShooterType::FIRST17MM; /* 射击机构类型 */

    /**
     * 裁判系统热量接收回调
     * 
     * @param msg 消息
     */
    void heatCallback(const rm_referee_controller::PowerHeatConstPtr& msg);

    /**
     * 裁判系统是否在线接收回调
     * 
     * @param msg 消息
     */
    void onlineCallback(const robot_msgs::BoolStampedConstPtr& msg);

    /**
     * 裁判系统机器人信息接收回调
     * 
     * @param msg 消息
     */
    void robotStatusCallback(const rm_referee_controller::RobotStatusConstPtr& msg);

  public:
    /**
     * 构造函数
     * 
     * @param node 节点
     * @param nodeParam 参数节点
     */
    HeatLimitModule(ros::NodeHandle& node, ros::NodeHandle& nodeParam);

    /**
     * 初始化
     * @return 初始化是否成功
     */
    bool init();

    /**
     * 获取该通道的值
     * 
     * @param vx X轴线速度增量输出
     * @param vy Y轴线速度增量输出
     * @param vrz Z轴角速度增量输出增量输出
     * @param yawAngle Yaw轴目标角度增量输出
     * @param pitchAngle Pitch轴目标角度增量输出
     * @param shotStatus 射击状态
     * @param isEnable 是否启用该模块
     * @param period 时间间隔
     * @param enableModules 所有模块是否启用列表
     */
    void getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, ShotStatus& shotStatus, bool& isEnable, ros::Duration period, std::map<std::string, bool>& enableModules);
};

}  // namespace rm_control
#endif