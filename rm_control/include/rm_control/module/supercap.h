#ifndef RM_CONTROL_MODULE_SUPERCAP_H_
#define RM_CONTROL_MODULE_SUPERCAP_H_

#include <ros/ros.h>

#include <rm_control/module/module.h>
#include <rm_referee_controller/RobotStatus.h>
#include <robot_toolbox/function_tool.h>
#include <supercap_controller/SupercapState.h>

namespace rm_control
{
class SupercapModule: public ModuleInterface
{
  private:
    ros::NodeHandle& node_;                                           /* 节点 */
    ros::NodeHandle& nodeParam_;                                      /* 参数节点 */
    std::unique_ptr<robot_toolbox::FunctionTool> limitNoVRZFunction_; /* 无小陀螺限速比例-超级电容剩余百分比函数 */
    std::unique_ptr<robot_toolbox::FunctionTool> limitVRZFunction_;   /* 小陀螺限速比例-超级电容剩余百分比函数 */
    std::unique_ptr<robot_toolbox::FunctionTool> powerFunction_;      /* 限速比例-当前底盘功率函数 */
    std::string stateTopic_;                                          /* State话题名称 */
    ros::Subscriber stateSubscriber_;                                 /* State话题订阅 */
    double percent_ = 0;                                              /* 超级电容剩余百分比 */
    ros::Subscriber robotStatusSubscriber_;                           /* 裁判系统机器人状态信息订阅者 */
    std::string robotStatusTopic_;                                    /* 裁判系统机器人状态信息话题 */
    std::string commandTopic_;                                        /* 超级电容目标功率话题 */
    ros::Publisher commandTopicPublisher_;                            /* 超级电容目标功率话题发布者 */
    double targetPower_ = 0;                                          /* 当前底盘最大功率 */

    /**
     * 裁判系统机器人信息接收回调
     * 
     * @param msg 消息
     */
    void robotStatusCallback(const rm_referee_controller::RobotStatusConstPtr& msg);

    /**
     * 超级电容信息回调
     * 
     * @param msg 
     */
    void stateCallback(const supercap_controller::SupercapStateConstPtr& msg);

  public:
    /**
     * 构造函数
     * 
     * @param node 节点
     * @param nodeParam 参数节点
     */
    SupercapModule(ros::NodeHandle& node, ros::NodeHandle& nodeParam);

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