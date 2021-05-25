#ifndef RM_CONTROL_MODULE_LASER_H_
#define RM_CONTROL_MODULE_LASER_H_

#include <ros/ros.h>

#include <rm_control/module/module.h>
#include <rm_referee_controller/GameStatus.h>
#include <robot_msgs/BoolStamped.h>

namespace rm_control
{
class LaserModule: public ModuleInterface
{
  private:
    ros::NodeHandle& node_;                 /* 节点 */
    ros::NodeHandle& nodeParam_;            /* 参数节点 */
    std::string commandTopic_;              /* 激光命令话题名称 */
    ros::Publisher commandPublisher_;       /* 激光命令话题发布者 */
    double targetcommand_ = 0;              /* 目标命令 */
    int commandHeaderSeq_ = 0;              /* 激光命令话题序号 */
    ros::Subscriber gameStatusSubscriber_;  /* 裁判系统比赛状态信息订阅者 */
    std::string gameStatusTopic_;           /* 裁判系统比赛状态信息话题 */
    bool isShouldStopOnce_         = true;  /* 是否需要停止一次激光, 这里为true,上电关激光 */
    bool isShouldStartOnce_        = false; /* 是否需要开启一次激光 */
    bool isEnableAutoControlLaser_ = false; /* 是否开启比赛中开关激光功能 */
    bool lastShouldStartLaser      = false; /* 上次接收到裁判系统时根据数据是否应该开启激光 */

    /**
     * 裁判系统比赛状态信息接收回调
     * 
     * @param msg 消息
     */
    void gameStatusCallback(const rm_referee_controller::GameStatusConstPtr& msg);

  public:
    /**
     * 构造函数
     * 
     * @param node 节点
     * @param nodeParam 参数节点
     */
    LaserModule(ros::NodeHandle& node, ros::NodeHandle& nodeParam);

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