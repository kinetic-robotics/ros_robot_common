#ifndef RC_CONTROL_CHANNEL_CHANNEL_H_
#define RC_CONTROL_CHANNEL_CHANNEL_H_

#include <ros/ros.h>

namespace rc_control
{
class ChannelInterface
{
  public:
    /**
     * 构造函数
     * 
     * @param node 节点
     * @param nodeParam 参数节点
     */
    ChannelInterface(ros::NodeHandle& node, ros::NodeHandle& nodeParam){};

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
     * @param enableModules 所有模块列表,可以通过该map禁用或启用模块
     */
    virtual void getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, std::map<std::string, bool>& enableModules) = 0;
};

}  // namespace rc_control
#endif