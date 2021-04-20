#include <boost/bind.hpp>

#include <ros/ros.h>

#include "robot_control/communication/can.h"
#include "robot_control/communication/config.h"
#include "robot_control/communication/usb_io_control.h"

namespace robot_control
{
CANDriver::CANDriver(ros::NodeHandle& node, ros::NodeHandle& nodeParam, std::shared_ptr<USBIOControl> driver)
    : node_(node), nodeParam_(nodeParam), driver_(driver)
{
}

bool CANDriver::init()
{
    driver_->registerRxCallback(boost::bind(&CANDriver::usbCallback, this, _1, _2));
    return true;
}

void CANDriver::registerRxCallback(RxCallback callback)
{
    rxCallbacks_.push_back(callback);
}

void CANDriver::registerErrorCallback(ErrorCallback callback)
{
    errorCallbacks_.push_back(callback);
}

void CANDriver::sendFrame(unsigned int canNum, Frame& frame)
{
    ROS_ASSERT(canNum < CAN_NUMBER);
    uint8_t sendID = CMD_CAN1_TX + canNum;
    std::vector<uint8_t> data;
    data.resize(4);
    uint32_t id = 0;
    id |= static_cast<int>(frame.type) << 31;
    id |= frame.isRemoteTransmission << 30;
    id |= frame.id << 1;
    data[0] = (id >> 24) & 0xFF;
    data[1] = (id >> 16) & 0xFF;
    data[2] = (id >> 8) & 0xFF;
    data[3] = (id)&0xFF;
    data.insert(data.end(), frame.data.begin(), frame.data.end());
    driver_->sendFrame(sendID, data);
}

void CANDriver::usbCallback(unsigned int cmdID, std::vector<uint8_t>& data)
{
    /* 收到数据 */
    if (cmdID == CMD_CAN1_RX || cmdID == CMD_CAN2_RX) {
        if (data.size() < 4) {
            ROS_ERROR("Receive CAN rx packet, but data length is wrong!");
            return;
        }
        unsigned int canNum = cmdID - CMD_CAN1_RX;
        /* CAN FPS统计 */
        auto now = ros::Time::now();
        cansFPS_[canNum].fps++;
        if (now - cansFPS_[canNum].lastTime > ros::Duration(1)) {
            if (cansFPS_[canNum].lastFPS - cansFPS_[canNum].fps > 10) {
                ROS_WARN("Detected CAN FPS is lower than previous! canNum = %d, fps = %d, last_fps = %d.", canNum, cansFPS_[canNum].fps, cansFPS_[canNum].lastFPS);
            }
            cansFPS_[canNum].lastFPS  = cansFPS_[canNum].fps;
            cansFPS_[canNum].fps      = 0;
            cansFPS_[canNum].lastTime = now;
        }
        Frame frame;
        uint32_t id                = data[0] << 24 | data[1] << 16 | (data[2] << 8) | data[3] & 0xFF;
        frame.type                 = id >> 31 == 0 ? Frame::Type::STD : Frame::Type::EXT;
        frame.isRemoteTransmission = (id >> 30) & 0xFF;
        frame.id                   = (id >> 1) & 0x1FFFFFFF;
        frame.data.insert(frame.data.end(), data.begin() + 4, data.end());
        for (size_t i = 0; i < rxCallbacks_.size(); i++) {
            rxCallbacks_[i](canNum, frame);
        }
    }
    /* 发生错误 */
    if (cmdID == CMD_ERROR && data.size() == 5 && (data[0] == ERROR_CAN1 || data[0] == ERROR_CAN2)) {
        unsigned int canNum = data[0] - ERROR_CAN1;
        uint32_t error      = data[1] << 24 | data[2] << 16 | (data[3] << 8) | data[4];
        std::string errMsg;
        if (error & CAN_ERROR_EWG) errMsg += "Protocol Error Warning.";
        if (error & CAN_ERROR_EPV) errMsg += "Error Passive.";
        if (error & CAN_ERROR_BOF) errMsg += "Bus-off error.";
        if (error & CAN_ERROR_STF) errMsg += "Stuff error.";
        if (error & CAN_ERROR_FOR) errMsg += "Form error.";
        if (error & CAN_ERROR_ACK) errMsg += "Acknowledgment error.";
        if (error & CAN_ERROR_BR) errMsg += "Bit recessive error.";
        if (error & CAN_ERROR_BD) errMsg += "Bit dominant error.";
        if (error & CAN_ERROR_CRC) errMsg += "CRC error.";
        if (error & CAN_ERROR_RX_FOV0) errMsg += "Rx FIFO0 overrun error.";
        if (error & CAN_ERROR_RX_FOV1) errMsg += "Rx FIFO1 overrun error.";
        if (error & CAN_ERROR_TX_ALST0) errMsg += "TxMailbox 0 transmit failure due to arbitration lost.";
        if (error & CAN_ERROR_TX_TERR0) errMsg += "TxMailbox 0 transmit failure due to transmit error.";
        if (error & CAN_ERROR_TX_ALST1) errMsg += "TxMailbox 1 transmit failure due to arbitration lost.";
        if (error & CAN_ERROR_TX_TERR1) errMsg += "TxMailbox 1 transmit failure due to transmit error.";
        if (error & CAN_ERROR_TX_ALST2) errMsg += "TxMailbox 2 transmit failure due to arbitration lost.";
        if (error & CAN_ERROR_TX_TERR2) errMsg += "TxMailbox 2 transmit failure due to transmit error.";
        if (error & CAN_ERROR_TIMEOUT) errMsg += "Timeout error.";
        if (error & CAN_ERROR_NOT_INITIALIZED) errMsg += "Peripheral not initialized.";
        if (error & CAN_ERROR_NOT_READY) errMsg += "Peripheral not ready.";
        if (error & CAN_ERROR_NOT_STARTED) errMsg += "Peripheral not started.";
        if (error & CAN_ERROR_PARAM) errMsg += "Parameter error.";
        if (error & CAN_ERROR_INTERNAL) errMsg += "Internal error.";
        if (error & CAN_ERROR_SEND_TIMEOUT) errMsg += "Send timeout error.";
        if (error & CAN_ERROR_COMMUNICATE_TIMEOUT) errMsg += "Forwarding timeout error.";
        ROS_ERROR("CAN Error occured! CAN Num: %u, Driver message: %s.", canNum, errMsg.c_str());
        for (size_t i = 0; i < errorCallbacks_.size(); i++) {
            errorCallbacks_[i](canNum, errMsg);
        }
    }
}

void CANDriver::shutdown()
{
}

}  // namespace robot_control