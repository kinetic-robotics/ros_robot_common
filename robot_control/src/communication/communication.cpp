#include <boost/bind.hpp>

#include "robot_control/communication/can.h"
#include "robot_control/communication/communication.h"
#include "robot_control/communication/serial.h"
#include "robot_control/communication/usb_io_control.h"

namespace robot_control
{
CommunicationDriver::CommunicationDriver(ros::NodeHandle& node, ros::NodeHandle& nodeParam)
    : node_(node), nodeParam_(nodeParam)
{
    usb    = std::make_shared<USBIOControl>(node_, nodeParam_);
    serial = std::make_unique<SerialDriver>(node_, nodeParam_, usb);
    can    = std::make_unique<CANDriver>(node_, nodeParam_, usb);
    usb->registerErrorCallback(boost::bind(&CommunicationDriver::usbErrorCallback, this, _1));
}

void CommunicationDriver::read(const ros::Time& time, const ros::Duration& period)
{
    usb->read(time, period);
}

void CommunicationDriver::write(const ros::Time& time, const ros::Duration& period)
{
    usb->write(time, period);
}

bool CommunicationDriver::init()
{
    return usb->init() &&
           can->init() &&
           serial->init();
}

void CommunicationDriver::shutdown()
{
    can->shutdown();
    serial->shutdown();
    usb->shutdown();
}

void CommunicationDriver::usbErrorCallback(std::string& errMsg)
{
    ROS_ERROR("USB Error! Error message: %s", errMsg.c_str());
}

}  // namespace robot_control