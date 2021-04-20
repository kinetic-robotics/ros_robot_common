#ifndef RC_CONTROL_CHANNEL_JOYSTICK_H_
#define RC_CONTROL_CHANNEL_JOYSTICK_H_

#include <ros/ros.h>

#include <robot_toolbox/function_tool.h>

#include "rc_control/channel/channel.h"
#include "sensor_msgs/Joy.h"

namespace rc_control
{
class JoystickChannel: public ChannelInterface
{
  private:
    ros::NodeHandle& node_;                                           /* 节点 */
    ros::NodeHandle& nodeParam_;                                      /* 参数节点 */
    double vx_;                                                       /* X轴线速度,单位m/s */
    double vy_;                                                       /* Y轴线速度,单位m/s */
    double yawAngle_   = 0;                                           /* Yaw轴目标角度,单位弧度 */
    double pitchAngle_ = 0;                                           /* Pitch轴目标角度,单位弧度 */
    int vxAxesNumber_;                                                /* 摇杆X轴速度通道编号 */
    int vyAxesNumber_;                                                /* 摇杆Y轴速度通道编号 */
    int yawAngleAxesNumber_;                                          /* 摇杆Z轴旋转速度通道编号 */
    int pitchAngleAxesNumber_;                                        /* 摇杆Pitch轴目标角度通道编号 */
    std::unique_ptr<robot_toolbox::FunctionTool> vxFunction_;         /* X轴速度-杆量函数 */
    std::unique_ptr<robot_toolbox::FunctionTool> vyFunction_;         /* Y轴速度-杆量函数 */
    std::unique_ptr<robot_toolbox::FunctionTool> yawAngleFunction_;   /* 摇杆Yaw轴目标角度-杆量函数 */
    std::unique_ptr<robot_toolbox::FunctionTool> pitchAngleFunction_; /* 摇杆Pitch轴目标角度-杆量函数 */
    ros::Subscriber joySubscriber_;                                   /* Joy话题订阅 */
    std::string joyTopic_;                                            /* Joy话题名称 */

    /**
     * 摇杆信息回调
     * 
     * @param msg 
     */
    void joyCallback(const sensor_msgs::JoyConstPtr& msg);

  public:
    /**
     * 构造函数
     * 
     * @param node 节点
     * @param nodeParam 参数节点
     */
    JoystickChannel(ros::NodeHandle& node, ros::NodeHandle& nodeParam);

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