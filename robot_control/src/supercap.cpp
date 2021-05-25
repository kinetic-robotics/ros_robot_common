#include <ros/ros.h>

#include <robot_interface/supercap.h>
#include <robot_toolbox/tool.h>

#include "robot_control/communication/communication.h"
#include "robot_control/supercap.h"

namespace robot_control
{
SupercapDriver::SupercapDriver(ros::NodeHandle& node, ros::NodeHandle& nodeParam, std::string urdf, CommunicationDriver& driver, hardware_interface::RobotHW& robotHW, bool& isDisableOutput)
    : ModuleInterface(node, nodeParam, urdf, driver, robotHW, isDisableOutput), node_(node), nodeParam_(nodeParam), urdf_(urdf), driver_(driver), robotHW_(robotHW), isDisableOutput_(isDisableOutput)
{
}

bool SupercapDriver::init()
{
    /* 读取配置 */
    CONFIG_ASSERT("supercap/can_num", nodeParam_.getParam("supercap/can_num", canNum_));
    CONFIG_ASSERT("supercap/send_rate", nodeParam_.getParam("supercap/send_rate", sendRate_));
    CONFIG_ASSERT("supercap/recv_can_id", nodeParam_.getParam("supercap/recv_can_id", recvCANID_));
    CONFIG_ASSERT("supercap/send_can_id", nodeParam_.getParam("supercap/send_can_id", sendCANID_));
    CONFIG_ASSERT("supercap/handle_name", nodeParam_.getParam("supercap/handle_name", handleName_));
    CONFIG_ASSERT("supercap/down_cap_voltage", nodeParam_.getParam("supercap/down_cap_voltage", downCapVoltage_));
    CONFIG_ASSERT("supercap/up_cap_voltage", nodeParam_.getParam("supercap/up_cap_voltage", upCapVoltage_) && upCapVoltage_ > downCapVoltage_);
    /* 注册CAN接收回调 */
    driver_.can->registerRxCallback(boost::bind(&SupercapDriver::canRXCallback, this, _1, _2));
    /* 注册接口 */
    interface_.registerHandle(robot_interface::SupercapHandle(handleName_, &percent_, &inputVoltage_, &inputCurrent_, &capVoltage_, &nowTargetPower_, &cmdTargetPower_));
    robotHW_.registerInterface(&interface_);
    return true;
}

void SupercapDriver::shutdown()
{
}

void SupercapDriver::canRXCallback(unsigned int canNum, CANDriver::Frame& f)
{
    /* 过滤非超级电容的包 */
    if (f.isRemoteTransmission || f.type != CANDriver::Frame::Type::STD || canNum != canNum_ || f.id != recvCANID_ || f.data.size() != 8) return;
    inputVoltage_   = ((unsigned short)(f.data[0] | f.data[1] << 8)) / 100.0f;
    capVoltage_     = ((unsigned short)(f.data[2] | f.data[3] << 8)) / 100.0f;
    inputCurrent_   = ((unsigned short)(f.data[4] | f.data[5] << 8)) / 100.0f;
    nowTargetPower_ = ((unsigned short)(f.data[6] | f.data[7] << 8)) / 100.0f;
    /* 计算百分比 */
    percent_ = (capVoltage_ - downCapVoltage_) / (upCapVoltage_ - downCapVoltage_);
    LIMIT(percent_, 0, 1);
}

void SupercapDriver::read(const ros::Time& time, const ros::Duration& period)
{
}

void SupercapDriver::write(const ros::Time& time, const ros::Duration& period)
{
    /* 发送CAN报文 */
    lastSendDuration_ += period;
    if (lastSendDuration_.toSec() >= 1 / sendRate_) {
        lastSendDuration_ = ros::Duration(0);
        CANDriver::Frame frame;
        frame.id                   = sendCANID_;
        frame.type                 = CANDriver::Frame::Type::STD;
        frame.isRemoteTransmission = false;
        frame.data.resize(2, 0);
        unsigned short power = cmdTargetPower_ * 100;
        frame.data[0]        = power >> 8;
        frame.data[1]        = power & 0xFF;
        driver_.can->sendFrame(canNum_, frame);
    }
}

}  // namespace robot_control