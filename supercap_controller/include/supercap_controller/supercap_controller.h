#ifndef SUPERCAP_CONTROLLER_SUPERCAP_CONTROLLER_H_
#define SUPERCAP_CONTROLLER_SUPERCAP_CONTROLLER_H_

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <robot_interface/supercap.h>
#include <std_msgs/Float64.h>

#include "supercap_controller/SupercapState.h"

namespace supercap_controller
{
class SupercapController: public controller_interface::Controller<robot_interface::SupercapInterface>
{
  private:
    std::unique_ptr<realtime_tools::RealtimePublisher<SupercapState>> statePublisher_; /* 状态信息发布 */
    robot_interface::SupercapHandle handle_;                                            /* 超级电容句柄 */
    ros::Subscriber targetPowerSubscriber_;                                             /* 目标功率订阅 */
    ros::Duration lastPublishDuration_;                                                 /* 上次发布超级电容信息的间隔 */
    double publishRate_;                                                                /* 发布超级电容信息的频率 */

    /**
     * 目标功率回调
     * 
     * @param msg 消息
     */
    void commandCallback(const std_msgs::Float64ConstPtr &msg);

  public:
    SupercapController();

    /**
     * ROS Control的Controller Init接口
     */
    bool init(robot_interface::SupercapInterface *hw, ros::NodeHandle &node);

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

}  // namespace supercap_controller

#endif