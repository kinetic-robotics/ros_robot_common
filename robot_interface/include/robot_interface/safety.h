#ifndef ROBOT_INTERFACE_SAFETY_H_
#define ROBOT_INTERFACE_SAFETY_H_

#include <ros/ros.h>

#include <hardware_interface/internal/hardware_resource_manager.h>

namespace robot_interface
{
class SafetyHandle
{
  private:
    std::string name_;
    const bool* currentStatus_ = {nullptr}; /* 当前是否切断所有输出 */
    bool* setStatus_           = {nullptr}; /* 设定是否切断所有输出 */

  public:
    SafetyHandle(){};

    /**
     * 构造函数
     * 
     * @param name 安全接口名称
     * @param currentStatus 是否禁用输出状态,true表示高电平,false表示低电平
     * @param setStatus 设定是否禁用输出,true表示高电平,false表示低电平
     */
    SafetyHandle(
        const std::string& name, bool* currentStatus, bool* setStatus)
        : name_(name), currentStatus_(currentStatus), setStatus_(setStatus){};

    std::string getName() const { return name_; }
    bool getCurrentStatus() const { ROS_ASSERT(currentStatus_); return *currentStatus_; }
    bool getSetStatus() const { ROS_ASSERT(setStatus_); return *setStatus_; }

    const bool* getCurrentStatusPtr() const { ROS_ASSERT(currentStatus_); return currentStatus_; }
    bool* getSetStatusPtr() const { ROS_ASSERT(setStatus_); return setStatus_; }

    void setStatus(bool command) const { ROS_ASSERT(setStatus_); *setStatus_ = command; }
};

class SafetyInterface: public hardware_interface::HardwareResourceManager<SafetyHandle> {};

}  // namespace robot_interface

#endif