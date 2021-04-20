#ifndef RC_CONTROL_MODULE_MODULE_H_
#define RC_CONTROL_MODULE_MODULE_H_

#include <ros/ros.h>

namespace rc_control
{
class ModuleInterface
{
  public:
    /**
     * 构造函数
     * 
     * @param node 节点
     * @param nodeParam 参数节点
     */
    ModuleInterface(ros::NodeHandle& node, ros::NodeHandle& nodeParam){};

    /**
     * 初始化
     * @return 初始化是否成功
     */
    virtual bool init() = 0;

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
    virtual void getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, bool& isEnable, ros::Duration period) = 0;
};

}  // namespace rc_control
#endif