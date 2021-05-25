#ifndef ROBOT_INTERFACE_IO_H_
#define ROBOT_INTERFACE_IO_H_

#include <ros/ros.h>

#include <hardware_interface/internal/hardware_resource_manager.h>

namespace robot_interface
{
class IOHandle
{
  private:
    std::string name_;
    const bool* currentLevel_ = {nullptr}; /* 当前电平状态,true表示高电平,false表示低电平 */
    bool* setLevel_           = {nullptr}; /* 设定目标电平,true表示高电平,false表示低电平 */

  public:
    IOHandle(){};

    /**
     * 构造函数
     * 
     * @param name 超级电容名称
     * @param currentLevel 当前电平状态,true表示高电平,false表示低电平
     * @param setLevel 设定目标电平,true表示高电平,false表示低电平
     */
    IOHandle(
        const std::string& name, bool* currentLevel, bool* setLevel)
        : name_(name), currentLevel_(currentLevel), setLevel_(setLevel){};

    std::string getName() const { return name_; }
    bool getCurrentLevel() const { ROS_ASSERT(currentLevel_); return *currentLevel_; }
    bool getSetLevel() const { ROS_ASSERT(setLevel_); return *setLevel_; }

    const bool* getCurrentLevelPtr() const { ROS_ASSERT(currentLevel_); return currentLevel_; }
    bool* getSetLevelPtr() const { ROS_ASSERT(setLevel_); return setLevel_; }

    void setLevel(bool command) const { ROS_ASSERT(setLevel_); *setLevel_ = command; }
};

class IOInterface: public hardware_interface::HardwareResourceManager<IOHandle> {};

}  // namespace robot_interface

#endif