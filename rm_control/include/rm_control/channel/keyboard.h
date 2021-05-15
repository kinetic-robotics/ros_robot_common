#ifndef RM_CONTROL_CHANNEL_KEYBOARD_H_
#define RM_CONTROL_CHANNEL_KEYBOARD_H_

#include <ros/ros.h>

#include <rm_rc_controller/Keyboard.h>
#include <robot_toolbox/function_tool.h>

#include "rm_control/channel/channel.h"

namespace rm_control
{
class KeyboardChannel: public ChannelInterface
{
  private:
    enum class KeyState {
        FORWARD,
        BACKWARD,
        NONE
    };
    ros::NodeHandle& node_;                                    /* 节点 */
    ros::NodeHandle& nodeParam_;                               /* 参数节点 */
    double vx_ = 0;                                            /* X轴线速度,单位m/s */
    double vy_ = 0;                                            /* Y轴线速度,单位m/s */
    std::string vxForwardKey_;                                 /* X轴前进按键 */
    std::string vxBackwardKey_;                                /* X轴后退按键 */
    std::string vyForwardKey_;                                 /* Y轴前进按键 */
    std::string vyBackwardKey_;                                /* Y轴后退按键 */
    std::string speedUpKey_;                                   /* 加速按钮,按下该按钮将解除电容限制 */
    std::string vrzKey_;                                       /* 小陀螺切换按钮 */
    bool lastVrzKeyState_ = false;                             /* 上一次小陀螺切换按钮状态 */
    std::unique_ptr<robot_toolbox::FunctionTool> vrzFunction_; /* 小陀螺模式时间-速度函数 */
    bool isVrzEnable_ = false;                                 /* 当前小陀螺模式是否开启 */
    ros::Time pressVrzButtonTime_;                             /* 开启小陀螺模式时的时间 */
    ros::Time pressVxButtonTime_;                              /* 按下X轴前进或后退按键时的时间 */
    ros::Time pressVyButtonTime_;                              /* 按下Y轴前进或后退按键时的时间 */
    KeyState vxKeyState_ = KeyState::NONE;                     /* X轴上一次按下的按钮 */
    KeyState vyKeyState_ = KeyState::NONE;                     /* Y轴上一次按下的按钮 */
    std::unique_ptr<robot_toolbox::FunctionTool> vxFunction_;  /* X轴时间-速度函数 */
    std::unique_ptr<robot_toolbox::FunctionTool> vyFunction_;  /* Y轴时间-速度函数 */
    ros::Subscriber keyboardSubscriber_;                       /* Keyboard话题订阅 */
    std::string keyboardTopic_;                                /* Keyboard话题名称 */
    bool isSpeedUp_ = false;                                   /* 是否开启加速模式 */

    /**
     * 通过按键名称获取按键是否按下
     * 
     * @param key 按键名称
     * @param keyboard 键盘信息
     * @return 是否按下
     */
    bool getValueByKeyName(std::string& key, const rm_rc_controller::KeyboardConstPtr& keyboard);

    /**
     * 键盘信息回调
     * 
     * @param msg 
     */
    void keyboardCallback(const rm_rc_controller::KeyboardConstPtr& msg);

    /**
     * 检查按键是否在可以监听的列表内
     * 
     * @param key 按键,不区分大小写
     * @return 是否可以监听
     */
    bool checkKey(std::string& key);

  public:
    /**
     * 构造函数
     * 
     * @param node 节点
     * @param nodeParam 参数节点
     */
    KeyboardChannel(ros::NodeHandle& node, ros::NodeHandle& nodeParam);

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
     * @param period 时间间隔
     * @param enableModules 所有模块列表,可以通过该map禁用或启用模块
     */
    void getValue(double& vx, double& vy, double& vrz, double& yawAngle, double& pitchAngle, ros::Duration period, std::map<std::string, bool>& enableModules);
};

}  // namespace rm_control
#endif