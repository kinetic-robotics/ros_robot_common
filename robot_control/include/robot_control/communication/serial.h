#ifndef ROBOT_CONTROL_COMMUNICATION_SERIAL_H_
#define ROBOT_CONTROL_COMMUNICATION_SERIAL_H_

#include <ros/ros.h>

#include "robot_control/communication/usb_io_control.h"

namespace robot_control
{
class SerialDriver
{
  public:
    /**
     * 构造函数
     * 
     * @param node 节点
     * @param nodeParam 参数节点
     * @param driver    通信驱动
     */
    SerialDriver(ros::NodeHandle& node, ros::NodeHandle& nodeParam, std::shared_ptr<USBIOControl> driver);

    /**
     * 初始化通信
     */
    bool init();

    /**
     * 退出
     */
    void shutdown();

    /**
     * 串口发送数据
     * 
     * @param serialNum 串口ID
     * @param data 数据
     */
    void send(unsigned int serialNum, std::vector<uint8_t>& data);

    /**
     * 串口收到数据回调函数
     * @param serialNum 串口ID
     * @param data 数据
     */
    using RxCallback = std::function<void(unsigned int, std::vector<uint8_t>&)>;

    /**
     * 串口出错回调函数
     * @param serialNum 串口ID
     * @param msg 错误内容
     */
    using ErrorCallback = std::function<void(unsigned int, std::string&)>;

    /**
     * 注册串口收到数据回调
     * 
     * @param callback 回调
     */
    void registerRxCallback(RxCallback callback);

    /**
     * 注册串口出错回调
     * 
     * @param callback 回调
     */
    void registerErrorCallback(ErrorCallback callback);

  private:
    ros::NodeHandle& node_;                     /* 节点 */
    ros::NodeHandle& nodeParam_;                /* 参数节点 */
    std::vector<RxCallback> rxCallbacks_;       /* 收到串口信息回调 */
    std::vector<ErrorCallback> errorCallbacks_; /* 串口出错回调 */
    std::shared_ptr<USBIOControl> driver_;      /* USB驱动 */

    /**
     * USB回调
     * 
     * @param serialNum 串口ID
     * @param data      数据
     */
    void usbCallback(unsigned int cmdID, std::vector<uint8_t>&);
};

}  // namespace robot_control

#endif