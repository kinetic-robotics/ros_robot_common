#include <boost/bind.hpp>

#include <ros/ros.h>

#include "robot_control/communication/config.h"
#include "robot_control/communication/serial.h"
#include "robot_control/communication/usb_io_control.h"

namespace robot_control
{
SerialDriver::SerialDriver(ros::NodeHandle& node, ros::NodeHandle& nodeParam, std::shared_ptr<USBIOControl> driver)
    : node_(node), nodeParam_(nodeParam), driver_(driver)
{
}

bool SerialDriver::init()
{
    driver_->registerRxCallback(boost::bind(&SerialDriver::usbCallback, this, _1, _2));
    return true;
}

void SerialDriver::registerRxCallback(RxCallback callback)
{
    rxCallbacks_.push_back(callback);
}

void SerialDriver::registerErrorCallback(ErrorCallback callback)
{
    errorCallbacks_.push_back(callback);
}

void SerialDriver::send(unsigned int serialNum, std::vector<uint8_t>& data)
{
    ROS_ASSERT(serialNum < SERIAL_NUMBER);
    driver_->sendFrame(CMD_UART1_TX + serialNum, data);
}

void SerialDriver::usbCallback(unsigned int cmdID, std::vector<uint8_t>& data)
{
    /* 收到数据 */
    if (cmdID == CMD_UART1_RX || cmdID == CMD_UART2_RX || cmdID == CMD_UART6_RX || cmdID == CMD_UART7_RX || cmdID == CMD_UART8_RX) {
        unsigned int serialNum = cmdID - CMD_UART1_RX;
        for (size_t i = 0; i < rxCallbacks_.size(); i++) {
            rxCallbacks_[i](serialNum, data);
        }
    }
    /* 发生错误 */
    if (cmdID == CMD_ERROR && data.size() == 5 && (data[0] == ERROR_UART1 || data[0] == ERROR_UART2 || data[0] == ERROR_UART6 || data[0] == ERROR_UART7 || data[0] == ERROR_UART8)) {
        unsigned int serialNum = data[0] - ERROR_UART1;
        uint32_t error         = data[1] << 24 | data[2] << 16 | (data[3] << 8) | data[4];
        std::string errMsg;
        if (error & UART_ERROR_PE) errMsg += "Parity error.";
        if (error & UART_ERROR_NE) errMsg += "Noise error.";
        if (error & UART_ERROR_FE) errMsg += "Frame error.";
        if (error & UART_ERROR_ORE) errMsg += "Overrun error.";
        if (error & UART_ERROR_DMA) errMsg += "DMA transfer error.";
        //ROS_ERROR("Serial Error occured! Serial Num: %u, Driver message: %s", serialNum, errMsg.c_str());
        for (size_t i = 0; i < errorCallbacks_.size(); i++) {
            errorCallbacks_[i](serialNum, errMsg);
        }
    }
}

void SerialDriver::shutdown()
{
}

}  // namespace robot_control