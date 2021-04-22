#ifndef RC_CONTROL_CHANNEL_MOUSE_H_
#define RC_CONTROL_CHANNEL_MOUSE_H_

#include <ros/ros.h>

#include <rm_rc_controller/Mouse.h>
#include <robot_toolbox/function_tool.h>

#include "rc_control/channel/channel.h"

namespace rc_control
{
class MouseChannel: public ChannelInterface
{
  private:
    ros::NodeHandle& node_;                                           /* 节点 */
    ros::NodeHandle& nodeParam_;                                      /* 参数节点 */
    double yawAngle_   = 0;                                           /* Yaw轴目标角度 */
    double pitchAngle_ = 0;                                           /* Pitch轴目标角度 */
    std::unique_ptr<robot_toolbox::FunctionTool> yawAngleFunction_;   /* Yaw轴目标角度-鼠标移动量函数 */
    std::unique_ptr<robot_toolbox::FunctionTool> pitchAngleFunction_; /* Pitch轴目标角度-鼠标移动量函数 */
    std::string mouseTopic_;                                          /* Mouse话题名称 */
    ros::Subscriber mouseSubscriber_;                                 /* Mouse话题订阅 */

    /**
     * 键盘信息回调
     * 
     * @param msg 
     */
    void mouseCallback(const rm_rc_controller::MouseConstPtr& msg);

  public:
    /**
     * 构造函数
     * 
     * @param node 节点
     * @param nodeParam 参数节点
     */
    MouseChannel(ros::NodeHandle& node, ros::NodeHandle& nodeParam);

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
     * @param enableModules 所有模块列表,可以通过该map禁用或启用模块
     */
    void getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, std::map<std::string, bool>& enableModules);
};

}  // namespace rc_control
#endif