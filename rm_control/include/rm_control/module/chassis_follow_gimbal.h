#ifndef RM_CONTROL_MODULE_CHASSIS_FOLLOW_GIMBAL_H_
#define RM_CONTROL_MODULE_CHASSIS_FOLLOW_GIMBAL_H_

#include <ros/ros.h>

#include <control_toolbox/pid.h>
#include <rm_control/module/module.h>
#include <robot_toolbox/function_tool.h>
#include <sensor_msgs/JointState.h>
#include <supercap_controller/supercap_state.h>

namespace rm_control
{
class ChassisFollowGimbalModule: public ModuleInterface
{
  private:
    ros::NodeHandle& node_;             /* 节点 */
    ros::NodeHandle& nodeParam_;        /* 参数节点 */
    std::string stateTopic_;            /* State话题名称 */
    ros::Subscriber stateSubscriber_;   /* State话题订阅 */
    std::vector<double> jointPosition_; /* 电机位置信息 */
    int yawNumber_;                     /* Yaw轴电机序号 */
    control_toolbox::Pid pid_;          /* 底盘跟随云台闭环 */

    /**
     * 电机角度信息回调
     * 
     * @param msg 
     */
    void stateCallback(const sensor_msgs::JointStateConstPtr& msg);

  public:
    /**
     * 构造函数
     * 
     * @param node 节点
     * @param nodeParam 参数节点
     */
    ChassisFollowGimbalModule(ros::NodeHandle& node, ros::NodeHandle& nodeParam);

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
     * @param isEnable 是否启用改模块
     * @param period 时间间隔
     */
    void getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, bool& isEnable, ros::Duration period);
};

}  // namespace rm_control
#endif