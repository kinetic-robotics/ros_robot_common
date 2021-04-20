#ifndef ROBOT_CONTROL_COMMUNICATION_CAN_H_
#define ROBOT_CONTROL_COMMUNICATION_CAN_H_

#include <ros/ros.h>

#include "robot_control/communication/usb_io_control.h"

namespace robot_control
{
class CANDriver
{
  public:
    typedef struct {
        uint32_t id; /* CAN ID */
        enum class Type{
            STD = 0,               /* 标准帧 */
            EXT = 1                /* 扩展帧 */
        } type;                    /* CAN帧类型 */
        std::vector<uint8_t> data; /* 数据 */
        bool isRemoteTransmission; /* 是否为远程帧 */
    } Frame;

    /**
     * 构造函数
     * 
     * @param node 节点
     * @param nodeParam 参数节点
     * @param driver    通信驱动
     */
    CANDriver(ros::NodeHandle& node, ros::NodeHandle& nodeParam, std::shared_ptr<USBIOControl> driver);

    /**
     * 初始化通信
     */
    bool init();

    /**
     * 退出
     */
    void shutdown();

    /**
     * CAN接收回调
     * 
     * @param canNum   CAN编号
     * @param frame    CAN帧
     */
    using RxCallback = boost::function<void(unsigned int, Frame&)>;

    /**
     * CAN错误回调
     * 
     * @param canNum   CAN编号
     * @param errorMsg 总线错误信息
     */
    using ErrorCallback = boost::function<void(unsigned int, std::string&)>;

    /**
     * 注册接收回调
     * 
     * @param callback 回调
     */
    void registerRxCallback(RxCallback callback);

    /**
     * 注册错误回调
     * 
     * @param callback 回调
     */
    void registerErrorCallback(ErrorCallback callback);

    /**
     * 发送CAN帧
     * 
     * @param canNum CAN编号
     * @param frame 帧
     */
    void sendFrame(unsigned int canNum, Frame& frame);

  private:
    ros::NodeHandle& node_;                     /* 节点 */
    ros::NodeHandle& nodeParam_;                /* 参数节点 */
    std::vector<RxCallback> rxCallbacks_;       /* 接收回调 */
    std::vector<ErrorCallback> errorCallbacks_; /* 状态变更回调 */
    std::shared_ptr<USBIOControl> driver_;      /* USB驱动 */

    typedef struct {
        int fps;            /* CAN帧率 */
        ros::Time lastTime; /* 上次统计CAN帧率的时间 */
        int lastFPS;        /* 上次的CAN帧率 */
    } CANFPSStatistics;
    std::array<CANFPSStatistics, 2> cansFPS_ = {0}; /* 统计CAN帧率 */

    /**
     * USB回调
     * 
     * @param serialNum 串口ID
     * @param data      数据
     */
    void usbCallback(unsigned int cmdID, std::vector<uint8_t>& data);
};

}  // namespace robot_control

#endif