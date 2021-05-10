#ifndef RM_CONTROL_MODULE_SUPERCAP_H_
#define RM_CONTROL_MODULE_SUPERCAP_H_

#include <ros/ros.h>

#include <rm_control/module/module.h>
#include <robot_toolbox/function_tool.h>
#include <supercap_controller/supercap_state.h>

namespace rm_control
{
class SupercapModule: public ModuleInterface
{
  private:
    ros::NodeHandle& node_;                                      /* 节点 */
    ros::NodeHandle& nodeParam_;                                 /* 参数节点 */
    std::unique_ptr<robot_toolbox::FunctionTool> limitFunction_; /* 限速比例-超级电容剩余百分比函数 */
    std::string stateTopic_;                                     /* State话题名称 */
    ros::Subscriber stateSubscriber_;                            /* State话题订阅 */
    double percent_ = 0;                                         /* 超级电容剩余百分比 */

    /**
     * 超级电容信息回调
     * 
     * @param msg 
     */
    void stateCallback(const supercap_controller::supercap_stateConstPtr& msg);

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
     * @param isEnable 是否启用改模块
     * @param period 时间间隔
     */
    void getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, bool& isEnable, ros::Duration period);
};

}  // namespace rm_control
#endif