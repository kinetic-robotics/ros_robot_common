#ifndef LASER_CONTROLLER_LASER_CONTROLLER_H_
#define LASER_CONTROLLER_LASER_CONTROLLER_H_

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <robot_interface/io.h>
#include <robot_msgs/BoolStamped.h>

namespace laser_controller
{
class LaserController: public controller_interface::Controller<robot_interface::IOInterface>
{
  private:
    robot_interface::IOHandle handle_;                                                           /* 电机句柄 */
    std::unique_ptr<realtime_tools::RealtimePublisher<robot_msgs::BoolStamped>> statePublisher_; /* 状态发布者 */
    ros::Subscriber switchSubscriber_;                                                           /* 开关订阅者 */
    bool isOn_ = false;                                                                          /* 是否打开激光 */
    ros::Duration lastPublishDuration_;                                                          /* 上次发布激光信息的间隔 */
    double publishRate_;                                                                         /* 发布激光信息的频率 */

    /**
     * 开关话题回调
     * 
     * @param msg 消息
     */
    void switchCallback(const robot_msgs::BoolStampedConstPtr& msg);

  public:
    LaserController();

    /**
     * ROS Control的Controller Init接口
     */
    bool init(robot_interface::IOInterface* hw, ros::NodeHandle& node);

    /**
     * ROS Control的Controller Update接口
     */
    void update(const ros::Time& time, const ros::Duration& period);

    /**
     * ROS Control的Controller Starting接口
     */
    void starting(const ros::Time& time);

    /**
     * ROS Control的Controller Stopping接口
     */
    void stopping(const ros::Time& time);
};

}  // namespace laser_controller

#endif