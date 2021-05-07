#include <ros/ros.h>

#include <libusb-1.0/libusb.h>
#include <robot_toolbox/tool.h>

#include "robot_control/communication/config.h"
#include "robot_control/communication/usb_io_control.h"

namespace robot_control
{

USBIOControl::USBIOControl(ros::NodeHandle& node, ros::NodeHandle& nodeParam)
    : node_(node), nodeParam_(nodeParam)
{
    /* 检测本类是否为第一次实例化 */
    if (that_ != NULL) {
        ROS_FATAL("robot_control::USBIOControl has been load more than once!");
    }
    that_ = this;
}

int USBIOControl::usbHotplugCallback(libusb_context* ctx, libusb_device* dev, libusb_hotplug_event event, void* userData)
{
    if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED) {
        ROS_INFO("USB device plug in.");
        if (libusb_open(dev, &that_->handle_) != LIBUSB_SUCCESS) {
            that_->callErrorCallback("Device cannot open.");
        } else {
            ROS_INFO("Open USB device success.");
            that_->isNeedAttachDrivers_ = true;
        }
    } else if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT) {
        that_->callErrorCallback("Device plug out.");
        if (that_->handle_) {
            libusb_close(that_->handle_);
            that_->handle_ = NULL;
        }
    }
    return 0;
}

bool USBIOControl::init()
{
    /* 读取配置 */
    CONFIG_ASSERT("communication/usb/local_mode", nodeParam_.getParam("communication/usb/local_mode",isLocalMode_));
    CONFIG_ASSERT("communication/usb/vendor_id", nodeParam_.getParam("communication/usb/vendor_id", vendorID_) && vendorID_ >= 0);
    CONFIG_ASSERT("communication/usb/product_id", nodeParam_.getParam("communication/usb/product_id", productID_) && productID_ >= 0);
    CONFIG_ASSERT("communication/usb/interface_number", nodeParam_.getParam("communication/usb/interface_number", interfaceNumber_) && interfaceNumber_ >= 0);
    CONFIG_ASSERT("communication/usb/tx/buffer_length", nodeParam_.getParam("communication/usb/tx/buffer_length",txBufferLength_) && txBufferLength_ > 0);
    CONFIG_ASSERT("communication/usb/tx/endpoint", nodeParam_.getParam("communication/usb/tx/endpoint",txEndpoint_) && (txEndpoint_ & 0x80)  == 0 && txEndpoint_ > 0);
    CONFIG_ASSERT("communication/usb/tx/timeout", nodeParam_.getParam("communication/usb/tx/timeout",txTimeout_) && txTimeout_ >= 0);
    CONFIG_ASSERT("communication/usb/rx/buffer_length", nodeParam_.getParam("communication/usb/rx/buffer_length", rxBufferLength_) && rxBufferLength_ > 0);
    CONFIG_ASSERT("communication/usb/rx/endpoint", nodeParam_.getParam("communication/usb/rx/endpoint", rxEndpoint_) && (rxEndpoint_ & 0x80) && rxEndpoint_ > 0);
    CONFIG_ASSERT("communication/usb/rx/timeout", nodeParam_.getParam("communication/usb/rx/timeout", rxTimeout_) && rxTimeout_ >= 0);
    /* 初始化USB */
    if (isLocalMode_) return true;
    if (libusb_init(NULL) != LIBUSB_SUCCESS) {
        ROS_FATAL("Init LibUSB failed!");
        return false;
    }
    if (libusb_hotplug_register_callback(
            NULL,
            (libusb_hotplug_event)(LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT),
            LIBUSB_HOTPLUG_ENUMERATE,
            vendorID_,
            productID_,
            LIBUSB_HOTPLUG_MATCH_ANY,
            &USBIOControl::usbHotplugCallback,
            NULL,
            &usbCallbackHandle_) != LIBUSB_SUCCESS) {
        ROS_FATAL("Init USB hotplug callback failed!");
        return false;
    }
    /* 运行10次循环以打开USB */
    struct timeval time = {0, 0};
    for (size_t i = 0; i < 10; i++) {
        libusb_handle_events_timeout_completed(NULL, &time, NULL);
    }
    /* 初始化线程 */
    usbThread_ = boost::thread(boost::bind(&USBIOControl::thread, this));
    return true;
}

void USBIOControl::shutdown()
{
    if (isLocalMode_) return;
    usbThread_.interrupt();
    usbThread_.join();
    libusb_hotplug_deregister_callback(NULL, usbCallbackHandle_);
    libusb_exit(NULL);
}

void USBIOControl::registerRxCallback(RxCallback callback)
{
    rxCallbacks_.push_back(callback);
}

void USBIOControl::registerErrorCallback(ErrorCallback callback)
{
    errorCallbacks_.push_back(callback);
}

void USBIOControl::sendFrame(unsigned int cmdID, std::vector<uint8_t>& data)
{
    if (isLocalMode_) return;
    uint8_t outputData[data.size() + 4] = {0};
    outputData[0]                       = PKG_SOF;
    outputData[1]                       = cmdID;
    outputData[2]                       = data.size();
    outputData[3]                       = PKG_SOF ^ cmdID ^ data.size();
    std::copy(data.begin(), data.end(), outputData + 4);
    if (!handle_) return;
    /* 防止看错括号,请注意这个括号不是if的 */
    {
        boost::mutex::scoped_lock guard(mutex_);
        txBuffer_.insert(txBuffer_.begin(), outputData, outputData + sizeof(outputData));
    }
}

void USBIOControl::thread()
{
    if (isLocalMode_) return;
    while (true) {
        boost::this_thread::sleep_for(boost::chrono::microseconds(100));
        /* 调用libusb事件处理函数 */
        struct timeval nonBlockingTime = {0, 0};
        libusb_handle_events_timeout_completed(NULL, &nonBlockingTime, NULL);
        /* 尝试附加驱动和打开interface */
        if (isNeedAttachDrivers_) {
            if (libusb_set_auto_detach_kernel_driver(handle_, 1) != LIBUSB_SUCCESS || libusb_claim_interface(handle_, interfaceNumber_) != LIBUSB_SUCCESS) {
                callErrorCallback("Detach drivers or claim interface failed");
            } else {
                isNeedAttachDrivers_ = false;
            }
        }
        if (!handle_ || isNeedAttachDrivers_) continue;
        /* 发送数据 */
        if (txBuffer_.size() != 0) {
            uint8_t data[txBuffer_.size()] = {0};
            {
                boost::mutex::scoped_lock guard(mutex_);
                std::copy(txBuffer_.begin(), txBuffer_.begin() + sizeof(data), data);
                txBuffer_.erase(txBuffer_.begin(), txBuffer_.begin() + sizeof(data));
            }
            int actualSendBytes = 0;
            int rc = libusb_bulk_transfer(handle_, txEndpoint_, data, sizeof(data), &actualSendBytes, txTimeout_ * 1000);
            std::string errMsg;
            /* 错误处理 */
            if (rc != LIBUSB_SUCCESS) {
                if (rc == LIBUSB_ERROR_TIMEOUT) errMsg += "Tx timeout.";
                else if (rc == LIBUSB_ERROR_PIPE) errMsg += "Tx endpoint halted.";
                else if (rc == LIBUSB_ERROR_OVERFLOW) errMsg += "Tx data over flow.";
                else if (rc == LIBUSB_ERROR_NO_DEVICE) errMsg += "Tx device disconnected.";
                else if (rc == LIBUSB_ERROR_BUSY) errMsg += "Tx event handle busy.";
                else if (rc == LIBUSB_ERROR_INVALID_PARAM) errMsg += "Tx invalid param.";
                else errMsg += "Tx Unknown error.";
                callErrorCallback(errMsg);
            } else if (actualSendBytes != sizeof(data)) {
                errMsg += "Tx send timeout.";
                callErrorCallback(errMsg);
            }
        }
        /* 接收数据 */
        uint8_t data[rxBufferLength_] = {0};
        int actualSendBytes = 0;
        int rc = libusb_bulk_transfer(handle_, rxEndpoint_, data, sizeof(data), &actualSendBytes, rxTimeout_ * 1000);
        std::string errMsg;
        /* 错误处理 */
        if (rc != LIBUSB_SUCCESS && rc != LIBUSB_ERROR_TIMEOUT) {
            if (rc == LIBUSB_ERROR_PIPE) errMsg += "Rx endpoint halted.";
            else if (rc == LIBUSB_ERROR_OVERFLOW) errMsg += "Rx data over flow.";
            else if (rc == LIBUSB_ERROR_NO_DEVICE) errMsg += "Rx device disconnected.";
            else if (rc == LIBUSB_ERROR_BUSY) errMsg += "Rx event handle busy.";
            else if (rc == LIBUSB_ERROR_INVALID_PARAM) errMsg += "Rx invalid param.";
            else errMsg += "Rx Unknown error.";
            callErrorCallback(errMsg);
        } else {
            boost::mutex::scoped_lock guard(mutex_);
            std::copy(data, data + actualSendBytes, std::back_inserter(rxRawBuffer_));
        }
    }
}

void USBIOControl::callErrorCallback(std::string errMsg)
{

    errorBuffer_.push_back(errMsg);
}

void USBIOControl::read(const ros::Time& time, const ros::Duration& period)
{
    if (isLocalMode_) return;
    std::vector<uint8_t> datas;
    {
        boost::mutex::scoped_lock guard(mutex_);
        datas.insert(datas.end(), rxRawBuffer_.begin(), rxRawBuffer_.end());
        rxRawBuffer_.erase(rxRawBuffer_.begin(), rxRawBuffer_.end());
    }
    for (auto iter = datas.begin(); iter != datas.end(); iter++) {
        uint8_t data = *iter;
        /* 解析数据的状态机 */
        switch (rxMachineState) {
            case ReceiveMachineState::SOF:
                if (data == PKG_SOF) rxMachineState = ReceiveMachineState::CMD_ID;
                break;
            case ReceiveMachineState::CMD_ID:
                rxCMDID_       = data;
                rxMachineState = ReceiveMachineState::LENGTH;
                break;
            case ReceiveMachineState::LENGTH:
                rxTargetDataLength = data;
                rxBuff_.resize(0);
                rxMachineState     = ReceiveMachineState::XOR;
                break;
            case ReceiveMachineState::XOR:
                if (PKG_SOF ^ rxCMDID_ ^ rxTargetDataLength == data) {
                    rxMachineState = ReceiveMachineState::DATA;
                } else {
                    rxMachineState = ReceiveMachineState::SOF;
                }
                break;
            case ReceiveMachineState::DATA:
                rxBuff_.push_back(data);
                if (rxBuff_.size() == rxTargetDataLength || rxTargetDataLength == 0) {
                    /* 加入帧信息 */
                    rxMachineState = ReceiveMachineState::SOF;
                    for (size_t i = 0; i < rxCallbacks_.size(); i++) {
                        rxCallbacks_[i](rxCMDID_, rxBuff_);
                    }
                }
                break;
        }
    }
    /* 调用错误处理函数 */
    std::vector<std::string> errors;
    {
        boost::mutex::scoped_lock guard(mutex_);
        errors.insert(errors.begin(), errorBuffer_.begin(), errorBuffer_.end());
        errorBuffer_.erase(errorBuffer_.begin(), errorBuffer_.end());
    }
    for (auto iter = errors.begin(); iter != errors.end(); iter++) {
        for (size_t i = 0; i < errorCallbacks_.size(); i++) {
            errorCallbacks_[i](*iter);
        }
    }
}

void USBIOControl::write(const ros::Time& time, const ros::Duration& period)
{
    if (isLocalMode_) return;
}

USBIOControl* USBIOControl::that_ = NULL;

}  // namespace robot_control