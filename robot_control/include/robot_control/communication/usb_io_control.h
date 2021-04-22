#ifndef ROBOT_CONTROL_COMMUNICATION_USB_IO_CONTROL_H_
#define ROBOT_CONTROL_COMMUNICATION_USB_IO_CONTROL_H_

#include <boost/thread.hpp>

#include <ros/ros.h>

#include <libusb-1.0/libusb.h>

namespace robot_control
{
class USBIOControl
{
  public:
    USBIOControl(ros::NodeHandle& node, ros::NodeHandle& nodeParam);

    /**
     * 初始化通信
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

    /**
     * USB接收回调
     * 
     * @param cmdID 命令编号
     * @param data  数据
     */
    using RxCallback = boost::function<void(unsigned int, std::vector<uint8_t>&)>;

    /**
     * USB错误回调
     * 
     * @param errorMsg 总线错误信息
     */
    using ErrorCallback = boost::function<void(std::string&)>;

    /**
     * 注册收到数据回调
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
     * 发送通信帧
     * 
     * @param cmdID 指令编号
     * @param data  数据
     */
    void sendFrame(unsigned int cmdID, std::vector<uint8_t>& data);

  private:
    enum class ReceiveMachineState {
        SOF    = 0,                                    /* 帧头 */
        CMD_ID = 1,                                    /* 指令ID */
        LENGTH = 2,                                    /* 长度 */
        XOR    = 3,                                    /* XOR校验 */
        DATA   = 4                                     /* 数据 */
    } rxMachineState = ReceiveMachineState::SOF;       /* USB接收状态机 */
    ros::NodeHandle& node_;                            /* 节点 */
    ros::NodeHandle& nodeParam_;                       /* 参数节点 */
    libusb_device* device_        = 0;                 /* USB设备 */
    libusb_device_handle* handle_ = 0;                 /* 已经打开的USB设备 */
    libusb_hotplug_callback_handle usbCallbackHandle_; /* USB回调句柄 */
    std::vector<RxCallback> rxCallbacks_;              /* 接收回调 */
    std::vector<ErrorCallback> errorCallbacks_;        /* 错误回调 */
    int txBufferLength_;                               /* USB发送缓冲区大小 */
    int rxBufferLength_;                               /* USB接收缓冲区大小 */
    int txEndpoint_;                                   /* USB发送端点 */
    int rxEndpoint_;                                   /* USB接收端点 */
    int interfaceNumber_;                              /* USB设备InterfaceNumber */
    int vendorID_;                                     /* USB设备VendorID */
    int productID_;                                    /* USB设备ProductID */
    double txTimeout_;                                 /* USB一次发送的超时时间,单位为秒 */
    double rxTimeout_;                                 /* USB一次接收的超时时间,单位为秒 */
    std::vector<uint8_t> rxBuff_;                      /* USB接收数据缓冲区 */
    uint8_t rxCMDID_;                                  /* USB接收数据指令ID */
    int rxTargetDataLength = 0;                        /* USB接收数据目标数据长度 */
    bool isLocalMode_;                                 /* 是否启用本地模式,在该模式下将无需访问USB串口 */
    bool isNeedAttachDrivers_ = false;                 /* 判断是否为刚刚成功打开设备,当热拔插回调成功打开设备后,需要交由read方法附加驱动和初始化interface,如果直接在里面初始化,则会引发报错,详见libusb文档 */
    static USBIOControl* that_;                        /* 第一个实例,用于确保只有一个实例存在,也用于libusb中c style function的转换 */
    std::vector<uint8_t> txBuffer_;                    /* USB发送缓冲区 */
    std::vector<uint8_t> rxRawBuffer_;                 /* USB原始接收缓冲区 */
    std::vector<std::string> errorBuffer_;             /* USB错误缓冲区 */
    boost::mutex mutex_;                               /* USB线程锁 */
    boost::thread usbThread_;                          /* USB线程 */

    /**
     * 收发线程
     * 
     */
    void thread();

    /**
     * USB热拔插事件回调
     * 
     * @param ctx 上下文
     * @param dev 设备
     * @param event 事件类型
     * @param userData 
     * @return 0 保持该回调注册
     * @return 1 取消该回调注册
     */
    static int usbHotplugCallback(libusb_context* ctx, libusb_device* dev, libusb_hotplug_event event, void* userData);

    /**
     * 调用错误回调
     * 
     * @param errMsg 错误信息
     */
    void callErrorCallback(std::string errMsg);
};

}  // namespace robot_control

#endif