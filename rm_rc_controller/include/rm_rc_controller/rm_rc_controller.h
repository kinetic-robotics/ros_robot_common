#ifndef RM_RC_CONTROLLER_RM_RC_CONTROLLER_H_
#define RM_RC_CONTROLLER_RM_RC_CONTROLLER_H_

#include <controller_interface/controller.h>
#include <geometry_msgs/TwistStamped.h>
#include <realtime_tools/realtime_publisher.h>
#include <robot_interface/remote_controller.h>
#include <robot_msgs/BoolStamped.h>
#include <sensor_msgs/Joy.h>

#include "rm_rc_controller/Keyboard.h"
#include "rm_rc_controller/Mouse.h"

namespace rm_rc_controller
{
class RMRCController: public controller_interface::Controller<robot_interface::RemoteControllerInterface>
{
  private:
    std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Joy>> joyPublisher_;             /* Joy信息发布 */
    std::unique_ptr<realtime_tools::RealtimePublisher<Keyboard>> kbPublisher_;                      /* 键盘信息发布 */
    std::unique_ptr<realtime_tools::RealtimePublisher<Mouse>> mousePublisher_;                      /* 鼠标信息发布 */
    std::unique_ptr<realtime_tools::RealtimePublisher<robot_msgs::BoolStamped>> isOnlinePublisher_; /* 遥控器是否在线信息发布 */
    robot_interface::RemoteControllerHandle handle_;                                                /* 遥控器句柄 */
    ros::Duration lastPublishDuration_;                                                             /* 上次发布遥控器信息的间隔 */
    double publishRate_;                                                                            /* 发布遥控器信息的频率 */

  public:
    RMRCController();

    /**
     * ROS Control的Controller Init接口
     */
    bool init(robot_interface::RemoteControllerInterface *hw, ros::NodeHandle &node);

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

}  // namespace rm_rc_controller

#endif