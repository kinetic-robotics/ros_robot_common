#ifndef BULLET_COVER_CONTROLLER_BULLET_COVER_CONTROLLER_H_
#define BULLET_COVER_CONTROLLER_BULLET_COVER_CONTROLLER_H_

#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <robot_msgs/BoolStamped.h>
#include <robot_toolbox/function_tool.h>

namespace bullet_cover_controller
{
class BulletCoverController: public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
  private:
    hardware_interface::JointHandle handle_;                                                                /* 电机句柄 */
    std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>> statePublisher_; /* 状态发布者 */
    double targetOnPosition_  = 0;                                                                          /* 弹舱盖打开时的位置 */
    double targetOffPosition_ = 0;                                                                          /* 弹舱盖关闭时的位置 */
    ros::Subscriber switchSubscriber_;                                                                      /* 开关订阅者 */
    bool isOn_ = false;                                                                                     /* 是否打开弹舱盖 */

    /**
     * 开关话题回调
     * 
     * @param msg 消息
     */
    void switchCallback(const robot_msgs::BoolStampedConstPtr& msg);

  public:
    BulletCoverController();

    /**
     * ROS Control的Controller Init接口
     */
    bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& node);

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

}  // namespace bullet_cover_controller

#endif