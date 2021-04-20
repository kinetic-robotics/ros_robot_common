#ifndef ROBOT_INTERFACE_SUPERCAP_H_
#define ROBOT_INTERFACE_SUPERCAP_H_

#include <ros/ros.h>

#include <hardware_interface/internal/hardware_resource_manager.h>

namespace robot_interface
{
class SupercapHandle
{
  private:
    std::string name_;
    const double* percent_        = {nullptr}; /* 剩余电量百分比 */
    const double* inputVoltage_   = {nullptr}; /* 输入电压 */
    const double* inputCurrent_   = {nullptr}; /* 输入电流 */
    const double* capVoltage_     = {nullptr}; /* 电容电压 */
    const double* nowTargetPower_ = {nullptr}; /* 实际目标功率 */
    double* cmdTargetPower_       = {nullptr}; /* 设定目标功率 */

  public:
    SupercapHandle(){};

    /**
     * 构造函数
     * 
     * @param name 超级电容名称
     * @param percent 剩余百分比
     * @param inputVoltage 输入电压
     * @param inputCurrent 输入电流
     * @param capVoltage 电容电压
     * @param nowTargetPower 当前目标功率
     * @param cmdTargetPower 设定目标功率
     */
    SupercapHandle(
        const std::string& name, double* percent, double* inputVoltage, double* inputCurrent, double* capVoltage, double* nowTargetPower, double* cmdTargetPower)
        : name_(name), percent_(percent), inputVoltage_(inputVoltage), inputCurrent_(inputCurrent), capVoltage_(capVoltage), nowTargetPower_(nowTargetPower), cmdTargetPower_(cmdTargetPower) {};

    std::string getName() const { return name_; }
    double getPercent() const { ROS_ASSERT(percent_); return *percent_; }
    double getInputVoltage() const { ROS_ASSERT(inputVoltage_); return *inputVoltage_; }
    double getInputCurrent() const { ROS_ASSERT(inputCurrent_); return *inputCurrent_; }
    double getCapVoltage() const { ROS_ASSERT(capVoltage_); return *capVoltage_; }
    double getNowTargetPower() const { ROS_ASSERT(nowTargetPower_); return *nowTargetPower_; }
    double getCMDTargetPower() const { ROS_ASSERT(cmdTargetPower_); return *cmdTargetPower_; }

    const double* getPercentPtr() const { ROS_ASSERT(percent_); return percent_; }
    const double* getInputVoltagePtr() const { ROS_ASSERT(inputVoltage_); return inputVoltage_; }
    const double* getInputCurrentPtr() const { ROS_ASSERT(inputCurrent_); return inputCurrent_; }
    const double* getCapVoltagePtr() const { ROS_ASSERT(capVoltage_); return capVoltage_; }
    const double* getNowTargetPowerPtr() const { ROS_ASSERT(nowTargetPower_); return nowTargetPower_; }
    double* getCMDTargetPowerPtr() const { ROS_ASSERT(cmdTargetPower_); return cmdTargetPower_; }

    void setCMDTargetPower(double command) const { ROS_ASSERT(cmdTargetPower_); *cmdTargetPower_ = command; }
};

class SupercapInterface: public hardware_interface::HardwareResourceManager<SupercapHandle> {};

}  // namespace robot_interface

#endif