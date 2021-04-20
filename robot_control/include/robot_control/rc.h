#ifndef ROBOT_CONTROL_RC_H_
#define ROBOT_CONTROL_RC_H_

#include <ros/ros.h>

#include <robot_interface/remote_controller.h>

#include "robot_control/module.h"

namespace robot_control
{
class RCDriver: public ModuleInterface
{
  private:
    ros::NodeHandle& node_;                                           /* 节点 */
    ros::NodeHandle& nodeParam_;                                      /* 参数节点 */
    std::string urdf_;                                                /* URDF文件 */
    CommunicationDriver& driver_;                                     /* 通信驱动 */
    hardware_interface::RobotHW& robotHW_;                            /* RobotHW层 */
    robot_interface::RemoteControllerInterface interface_;            /* 遥控器接口 */
    ros::Timer timeoutTimer_;                                         /* 遥控器信息超时计时器 */
    std::vector<double> ch_;                                          /* 通道信息 */
    std::vector<robot_interface::RemoteControllerHandle::Switch> sw_; /* 开关信息 */
    int rcSerialNum_;                                                 /* 遥控器串口编号 */
    double rcTimeout_;                                                /* 遥控器超时时间 */
    std::string handleName_;                                          /* 遥控器Handle名称 */
    std::vector<uint8_t> serialRecvData_;                             /* 遥控器收到的数据 */
    ros::Time lastRecvByteTime_;                                      /* 上次收到遥控器数据的时间,由于串口数据并不总是每帧18字节,所以需要使用这个判断空闲中断 */
    double isOneline_ = false;                                        /* 遥控器是否在线 */

    /**
     * 串口接收回调
     * 
     * @param serialNum 
     * @param data 
     */
    void serialRXCallback(unsigned int serialNum, std::vector<uint8_t>& data);

    /**
     * 遥控器信息超时回调
     * 
     */
    void timeoutCallback();

    /**
     * 解析遥控器数据
     * 
     * @param data 遥控器数据帧
     */
    void parsedData(std::vector<uint8_t>& data);

  public:
    /**
     * 构造函数
     * 
     * @param node 节点
     * @param nodeParam 参数节点
     * @param urdf URDF文件
     * @param driver 驱动
     * @param robotHW RobotHW层
     */
    RCDriver(ros::NodeHandle& node, ros::NodeHandle& nodeParam, std::string urdf, CommunicationDriver& driver, hardware_interface::RobotHW& robotHW);

    /**
     * 初始化
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