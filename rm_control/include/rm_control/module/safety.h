#ifndef RM_CONTROL_MODULE_SAFETY_H_
#define RM_CONTROL_MODULE_SAFETY_H_

#include <ros/ros.h>

#include <robot_msgs/BoolStamped.h>

#include "rm_control/module/module.h"

namespace rm_control
{
class SafetyModule: public ModuleInterface
{
  private:
    ros::NodeHandle& node_;              /* 节点 */
    ros::NodeHandle& nodeParam_;         /* 参数节点 */
    std::string commandTopic_;           /* Command话题名称 */
    ros::Publisher commandPublisher_;    /* Command话题订阅 */
    std::string rcOnlineTopic_;          /* 遥控器是否在线话题名称 */
    ros::Subscriber rcOnlineSubscriber_; /* 遥控器是否在线订阅者 */
    bool isRCOnline_ = false;            /* 遥控器是否在线 */

    /**
     * 遥控器是否在线接收回调
     * 
     * @param msg 消息
     */
    void rcOnlineCallback(const robot_msgs::BoolStampedConstPtr& msg);

  public:
    /**
     * 构造函数
     * 
     * @param node 节点
     * @param nodeParam 参数节点
     */
    SafetyModule(ros::NodeHandle& node, ros::NodeHandle& nodeParam);

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
     */
    void getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, ShotStatus& shotStatus, bool& isEnable, ros::Duration period);
};

}  // namespace rm_control
#endif