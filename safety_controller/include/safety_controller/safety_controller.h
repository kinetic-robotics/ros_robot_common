#ifndef SAFETY_CONTROLLER_KEY_CONTROLLER_H_
#define SAFETY_CONTROLLER_KEY_CONTROLLER_H_

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <robot_interface/safety.h>
#include <robot_msgs/BoolStamped.h>

namespace safety_controller
{
class SafetyController: public controller_interface::Controller<robot_interface::SafetyInterface>
{
  private:
    double checkTime_;                                                                  /* 超时时间,超时后会自动动作关闭输出 */
    robot_interface::SafetyHandle handle_;                                              /* 句柄 */
    double timeout_;                                                                    /* 命令超时时间 */
    ros::Subscriber commandSubscriber_;                                                 /* 命令订阅 */
    ros::Timer timeoutTimer_;                                                           /* 命令超时计时器 */
    std::unique_ptr<realtime_tools::RealtimePublisher<robot_msgs::BoolStamped>> statePublisher_; /* 状态信息发布 */
    ros::Duration lastPublishDuration_;                                                 /* 上次发布按键信息的间隔 */
    double publishRate_;                                                                /* 发布安全状态信息的频率 */

    /**
     * 命令超时回调
     * 
     */
    void timeoutCallback();

    /**
     * 安全命令话题回调
     * 
     * @param msg 消息
     */
    void commandCallback(const robot_msgs::BoolStampedConstPtr &msg);

  public:
    SafetyController();

    /**
     * ROS Control的Controller Init接口
     */
    bool init(robot_interface::SafetyInterface *hw, ros::NodeHandle &node);

    /**
     * ROS Control的Controller Update接口
     */
    void update(const ros::Time &time, const ros::Duration &period);

    /**
     * ROS Control的Controller Starting接口
     */
    void starting(const ros::Time &time);

    /**
     * ROS Control的Controller Stopping接口
     */
    void stopping(const ros::Time &time);
};

}  // namespace safety_controller

#endif