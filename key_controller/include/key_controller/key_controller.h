#ifndef KEY_CONTROLLER_KEY_CONTROLLER_H_
#define KEY_CONTROLLER_KEY_CONTROLLER_H_

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <robot_interface/io.h>
#include <std_msgs/Bool.h>

namespace key_controller
{
class KeyController: public controller_interface::Controller<robot_interface::IOInterface>
{
  private:
    typedef struct {
        bool isActiveLow;                                                                  /* 是否低电平有效 */
        double antiShakeTime;                                                              /* 防抖时间 */
        robot_interface::IOHandle handle;                                                  /* 句柄 */
        std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Bool>> statePublisher; /* 状态信息发布 */
        ros::Time lastActiveTime;                                                          /* 收到激活电平时的时间,用于防抖 */
    } KeyInfo;
    ros::Duration lastPublishDuration_;   /* 上次发布超级电容信息的间隔 */
    double publishRate_;                  /* 发布超级电容信息的频率 */
    std::map<std::string, KeyInfo> keys_; /* 按键信息数组 */

  public:
    KeyController();

    /**
     * ROS Control的Controller Init接口
     */
    bool init(robot_interface::IOInterface *hw, ros::NodeHandle &node);

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

}  // namespace key_controller

#endif