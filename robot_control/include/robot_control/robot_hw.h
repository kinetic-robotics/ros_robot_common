#ifndef ROBOT_CONTROL_ROBOT_HW_H_
#define ROBOT_CONTROL_ROBOT_HW_H_

#include <hardware_interface/robot_hw.h>
#include <robot_interface/safety.h>

#include "robot_control/communication/communication.h"
#include "robot_control/motor.h"

namespace robot_control
{
class RobotHW: public hardware_interface::RobotHW
{
  private:
    ros::NodeHandle& node_;                                           /* 节点 */
    ros::NodeHandle& nodeParam_;                                      /* 参数节点 */
    std::shared_ptr<CommunicationDriver> communicationDriver_;        /* Communication驱动实例 */
    std::map<std::string, std::shared_ptr<ModuleInterface>> modules_; /* 支持的模块数组 */
    bool isDisableOutput_ = false;                                    /* 是否禁用输出 */
    robot_interface::SafetyInterface safetyInterface_;                /* 安全接口 */
  public:
    /**
     * 构造函数
     * 
     * @param node 节点
     * @param nodeParam 参数节点
     */
    RobotHW(ros::NodeHandle& node, ros::NodeHandle& nodeParam);

    /**
     * 初始化RobotHW层
     * @return 初始化是否成功
     */
    bool init();

    /**
     * 退出
     */
    void shutdown();

    /**
     * ROS Control 需要的读取函数
     */
    void read(const ros::Time& time, const ros::Duration& period);

    /**
     * ROS Control 需要的写入函数
     */
    void write(const ros::Time& time, const ros::Duration& period);
};

}  // namespace robot_control
#endif