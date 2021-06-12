#ifndef ROBOT_INTERFACE_REMOTE_CONTROLLER_H_
#define ROBOT_INTERFACE_REMOTE_CONTROLLER_H_

#include <ros/ros.h>

#include <hardware_interface/internal/hardware_resource_manager.h>

namespace robot_interface
{
class RemoteControllerHandle
{
  public:
    /* 拉杆状态 */
    enum class Switch {
        UP   = 1,
        MID  = 0,
        DOWN = -1
    };

    RemoteControllerHandle(){};

    /**
     * 构造函数
     * 
     * @param name 遥控器名称
     * @param isOnline 遥控器是否在线
     * @param ch 通道们, 范围-1-1
     * @param sw 开关们
     */
    RemoteControllerHandle(
        const std::string& name, bool* isOnline, std::vector<double>* ch, std::vector<Switch>* sw)
        : name_(name), ch_(ch), sw_(sw), isOnline_(isOnline){};

    std::string getName() const { return name_; }
    bool getIsOnline() const { ROS_ASSERT(isOnline_); return *isOnline_; }
    std::vector<double> getCH() const { ROS_ASSERT(ch_); return *ch_; }
    std::vector<Switch> getSW() const { ROS_ASSERT(sw_); return *sw_; }

    const bool* getIsOnlinePtr() const { ROS_ASSERT(isOnline_); return isOnline_; }
    const std::vector<double>* getCHPtr() const { ROS_ASSERT(ch_); return ch_; }
    const std::vector<Switch>* getSWPtr() const { ROS_ASSERT(sw_); return sw_; }

  private:
    std::string name_;
    const std::vector<double>* ch_      = {nullptr};
    const std::vector<Switch>* sw_      = {nullptr};
    const bool* isOnline_               = {nullptr};
};

class RemoteControllerInterface: public hardware_interface::HardwareResourceManager<RemoteControllerHandle> {};

}  // namespace robot_interface

#endif