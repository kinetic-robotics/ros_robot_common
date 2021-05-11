#ifndef RM_CONTROL_CHANNEL_CHANNEL_MANAGER_H_
#define RM_CONTROL_CHANNEL_CHANNEL_MANAGER_H_

#include <ros/ros.h>

#include "rm_control/channel/channel.h"

namespace rm_control
{
class ChannelManager
{
  private:
    ros::NodeHandle& node_;                                             /* 节点 */
    ros::NodeHandle& nodeParam_;                                        /* 参数节点 */
    std::map<std::string, std::shared_ptr<ChannelInterface>> channels_; /* 通道数组 */
    std::map<std::string, bool>& modulesStatus_;                        /* 模块的状态,主要允许通道动态修改模块是否启用 */
  public:
    /**
     * 构造函数
     * 
     * @param node 节点
     * @param nodeParam 参数节点
     * @param modulesStatus 模块的状态,主要允许通道动态修改模块是否启用
     */
    ChannelManager(ros::NodeHandle& node, ros::NodeHandle& nodeParam, std::map<std::string, bool>& modulesStatus);

    /**
     * 初始化
     * @return 初始化是否成功
     */
    bool init();

    /**
     * 发布相关话题
     * @param vx X轴线速度增量输出
     * @param vy Y轴线速度增量输出
     * @param vrz Z轴角速度增量输出增量输出
     * @param yawAngle Yaw轴目标角度增量输出
     * @param pitchAngle Pitch轴目标角度增量输出
     * @param period 时间间隔
     */
    void update(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, ros::Duration period);
};
}  // namespace rm_control

#endif