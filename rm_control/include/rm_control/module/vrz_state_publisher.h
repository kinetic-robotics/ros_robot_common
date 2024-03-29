#ifndef RM_CONTROL_MODULE_VRZ_STATE_PUBLISHER_H_
#define RM_CONTROL_MODULE_VRZ_STATE_PUBLISHER_H_

#include <ros/ros.h>

#include <rm_control/module/module.h>

namespace rm_control
{
class VRZStatePublisherModule: public ModuleInterface
{
  private:
    ros::NodeHandle& node_;             /* 节点 */
    ros::NodeHandle& nodeParam_;        /* 参数节点 */
    ros::Publisher vrzStatusPublisher_; /* 小陀螺状态话题发布者 */
    int vrzStatusPublisherSeq_ = 0;     /* 小陀螺是否启用信息发布序号 */
  public:
    /**
     * 构造函数
     * 
     * @param node 节点
     * @param nodeParam 参数节点
     */
    VRZStatePublisherModule(ros::NodeHandle& node, ros::NodeHandle& nodeParam);

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