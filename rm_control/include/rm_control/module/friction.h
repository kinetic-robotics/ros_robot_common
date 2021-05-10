#ifndef RM_CONTROL_MODULE_FRICTION_H_
#define RM_CONTROL_MODULE_FRICTION_H_

#include <ros/ros.h>

#include <rm_control/module/module.h>
#include <rm_referee_controller/RobotStatus.h>
#include <robot_msgs/Float64Stamped.h>

namespace rm_control
{
class FrictionModule: public ModuleInterface
{
  private:
    ros::NodeHandle& node_;                                      /* 节点 */
    ros::NodeHandle& nodeParam_;                                 /* 参数节点 */
    std::string speedTopic_;                                     /* 摩擦轮转速话题名称 */
    ros::Publisher speedPublisher_;                              /* 摩擦轮转速话题发布者 */
    double targetSpeed_ = 0;                                     /* 目标射速 */
    int speedHeaderSeq_ = 0;                                     /* 摩擦轮转速话题序号 */
    ros::Subscriber robotStatusSubscriber_;                      /* 裁判系统机器人状态信息订阅者 */
    std::string robotStatusTopic_;                               /* 裁判系统机器人状态信息话题 */
    bool isFirstLoop_ = true;                                    /* 是否第一次运行update函数 */
    enum class ShooterType {
        FIRST17MM  = 0,
        SECOND17MM = 1,
        FIRST42MM  = 2,
    } shooterType_ = ShooterType::FIRST17MM; /* 射击机构类型 */

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
    FrictionModule(ros::NodeHandle& node, ros::NodeHandle& nodeParam);

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