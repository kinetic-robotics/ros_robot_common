#ifndef RM_CONTROL_CHANNEL_JOYSTICK_H_
#define RM_CONTROL_CHANNEL_JOYSTICK_H_

#include <ros/ros.h>

#include <robot_toolbox/function_tool.h>

#include "rm_control/channel/channel.h"
#include "sensor_msgs/Joy.h"

namespace rm_control
{
class JoystickChannel: public ChannelInterface
{
  private:
    ros::NodeHandle& node_;                                           /* 节点 */
    ros::NodeHandle& nodeParam_;                                      /* 参数节点 */
    double vx_              = 0;                                      /* X轴线速度,单位m/s */
    double vy_              = 0;                                      /* Y轴线速度,单位m/s */
    double vrz_             = 0;                                      /* Z旋转轴速度,单位rad/s */
    double yawAngleDelta_   = 0;                                      /* Yaw轴目标变化角度,单位弧度 */
    double pitchAngleDelta_ = 0;                                      /* Pitch轴目标变化角度,单位弧度 */
    int vxAxesNumber_;                                                /* 摇杆X轴速度通道编号 */
    int vyAxesNumber_;                                                /* 摇杆Y轴速度通道编号 */
    int vrzAxesNumber_;                                               /* 摇杆Z旋转速度通道编号 */
    int yawAngleAxesNumber_;                                          /* 摇杆Yaw轴目标角度通道编号 */
    int pitchAngleAxesNumber_;                                        /* 摇杆Pitch轴目标角度通道编号 */
    int frictionButtonNumber_;                                        /* 摩擦轮开关的拨杆编号 */
    int frictionToggleState_;                                         /* 摩擦轮开关时的摇杆状态 */
    int safetyButtonNumber_;                                          /* 禁用输出的拨杆编号 */
    int safetyToggleState_;                                           /* 禁用输出时的摇杆状态 */
    int disableJoystickButtonNumber_;                                 /* 禁用摇杆的拨杆编号 */
    int disableJoystickToggleState_;                                  /* 禁用摇杆时的摇杆状态 */
    std::unique_ptr<robot_toolbox::FunctionTool> vxFunction_;         /* X轴速度-杆量函数 */
    std::unique_ptr<robot_toolbox::FunctionTool> vyFunction_;         /* Y轴速度-杆量函数 */
    std::unique_ptr<robot_toolbox::FunctionTool> vrzFunction_;        /* 上面波拨轮也就是Z轴旋转速度-杆量函数 */
    std::unique_ptr<robot_toolbox::FunctionTool> yawAngleFunction_;   /* 摇杆Yaw轴目标角度-杆量函数 */
    std::unique_ptr<robot_toolbox::FunctionTool> pitchAngleFunction_; /* 摇杆Pitch轴目标角度-杆量函数 */
    ros::Subscriber joySubscriber_;                                   /* Joy话题订阅 */
    std::string joyTopic_;                                            /* Joy话题名称 */
    int lastFrictionButtonState_ = 0;                                 /* 上一次循环时左拨杆状态 */
    bool isShouldToggleFriction_ = false;                             /* 是否切换摩擦轮状态 */
    bool isSafetyEnable_         = false;                             /* 是否禁用输出 */

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
     * @param shotStatus 射击状态
     * @param period 时间间隔
     * @param enableModules 所有模块列表,可以通过该map禁用或启用模块
     */
    void getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, ShotStatus& shotStatus, ros::Duration period, std::map<std::string, bool>& enableModules);
};

}  // namespace rm_control
#endif