#ifndef ROBOT_CONTROL_COMMUNICATION_H_
#define ROBOT_CONTROL_COMMUNICATION_H_

#include <ros/ros.h>

#include "robot_control/communication/serial.h"
#include "robot_control/communication/can.h"
#include "robot_control/communication/usb_io_control.h"

namespace robot_control
{

class CommunicationDriver
{
  public:
    /**
     * 构造函数
     * 
     * @param node 节点
     * @param nodeParam 参数节点
     */
    CommunicationDriver(ros::NodeHandle& node, ros::NodeHandle& nodeParam);

    /**
     * 初始化所有通信
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

    std::shared_ptr<USBIOControl> usb;
    std::unique_ptr<SerialDriver> serial;
    std::unique_ptr<CANDriver> can;

  private:
    ros::NodeHandle& node_;      /* 节点 */
    ros::NodeHandle& nodeParam_; /* 参数节点 */

    /**
     * USB出错回调
     * 
     * @param errMsg 错误信息
     */
    void usbErrorCallback(std::string& errMsg);
};

}  // namespace robot_control
#endif