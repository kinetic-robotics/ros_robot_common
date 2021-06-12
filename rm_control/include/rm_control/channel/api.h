#ifndef RM_CONTROL_CHANNEL_API_H_
#define RM_CONTROL_CHANNEL_API_H_

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <robot_msgs/Float64Stamped.h>
#include <robot_toolbox/function_tool.h>

#include "rm_control/channel/channel.h"

namespace rm_control
{
class ApiChannel: public ChannelInterface
{
  private:
    ros::NodeHandle& node_;            /* 节点 */
    ros::NodeHandle& nodeParam_;       /* 参数节点 */
    double vx_              = 0;       /* X轴线速度,单位m/s */
    double vy_              = 0;       /* Y轴线速度,单位m/s */
    double vrz_             = 0;       /* Z旋转轴速度,单位rad/s */
    double yawAngleDelta_   = 0;       /* Yaw轴目标变化角度,单位弧度 */
    double pitchAngleDelta_ = 0;       /* Pitch轴目标变化角度,单位弧度 */
    ros::Subscriber cmdVelSubscriber_; /* Cmd_vel话题订阅 */
    ros::Subscriber pitchSubscriber_;  /* Pitch角度话题订阅 */
    ros::Subscriber yawSubscriber_;    /* Yaw角度话题订阅 */

    /**
     * Cmd_vel信息回调
     * 
     * @param msg 
     */
    void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg);

    /**
     * Pitch角度信息回调
     * 
     * @param msg 
     */
    void pitchCallback(const robot_msgs::Float64StampedConstPtr& msg);

    /**
     * Yaw角度信息回调
     * 
     * @param msg 
     */
    void yawCallback(const robot_msgs::Float64StampedConstPtr& msg);

  public:
    /**
     * 构造函数
     * 
     * @param node 节点
     * @param nodeParam 参数节点
     */
    ApiChannel(ros::NodeHandle& node, ros::NodeHandle& nodeParam);

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
     * @param period 时间间隔
     * @param enableModules 所有模块列表,可以通过该map禁用或启用模块
     */
    void getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, ShotStatus& shotStatus, ros::Duration period, std::map<std::string, bool>& enableModules);
};

}  // namespace rm_control
#endif