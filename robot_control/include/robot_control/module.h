#ifndef ROBOT_CONTROL_MODULE_H_
#define ROBOT_CONTROL_MODULE_H_

#include <ros/ros.h>

#include <hardware_interface/robot_hw.h>
#include <urdf/model.h>

#include "robot_control/communication/communication.h"

namespace robot_control
{
class ModuleInterface
{
  public:
    /**
     * 构造函数
     * 
     * @param node 节点
     * @param nodeParam 参数节点
     * @param urdf URDF文件
     * @param driver 驱动
     * @param robotHW RobotHW层
     * @param isDisableOutput 是否禁用输出
     */
    ModuleInterface(ros::NodeHandle& node, ros::NodeHandle& nodeParam, std::string urdf, CommunicationDriver& driver, hardware_interface::RobotHW& robotHW, bool& isDisableOutput){};

    /**
     * 初始化
     * @return 初始化是否成功
     */
    virtual bool init() = 0;

    /**
     * 退出
     */
    virtual void shutdown() = 0;

    /**
     * ROS Control 需要的读取函数
     */
    virtual void read(const ros::Time& time, const ros::Duration& period) = 0;

    /**
     * ROS Control 需要的写入函数
     */
    virtual void write(const ros::Time& time, const ros::Duration& period) = 0;
};

}  // namespace robot_control
#endif