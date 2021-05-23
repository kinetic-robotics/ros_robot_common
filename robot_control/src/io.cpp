#include <ros/ros.h>

#include <robot_interface/io.h>
#include <robot_toolbox/tool.h>

#include "robot_control/communication/communication.h"
#include "robot_control/io.h"

namespace robot_control
{
IODriver::IODriver(ros::NodeHandle& node, ros::NodeHandle& nodeParam, std::string urdf, CommunicationDriver& driver, hardware_interface::RobotHW& robotHW, bool& isDisableOutput)
    : ModuleInterface(node, nodeParam, urdf, driver, robotHW, isDisableOutput), node_(node), nodeParam_(nodeParam), urdf_(urdf), driver_(driver), robotHW_(robotHW), isDisableOutput_(isDisableOutput)
{
}

bool IODriver::init()
{
    /* 读取配置 */
    CONFIG_ASSERT("io/send_rate", nodeParam_.getParam("io/send_rate", sendRate_));
    /* 解析配置注册接口 */
    XmlRpc::XmlRpcValue ioList;
    nodeParam_.getParam("io/device", ioList);
    CONFIG_ASSERT("io/device", ioList.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (auto iter = ioList.begin(); iter != ioList.end(); ++iter) {
        memset(&ios_[iter->first], 0, sizeof(ios_[iter->first]));
        CONFIG_ASSERT("io/device/" + iter->first + "/id", iter->second["id"].getType() == XmlRpc::XmlRpcValue::TypeInt && static_cast<int>(iter->second["id"]) >= 0);
        ios_[iter->first].id = static_cast<int>(iter->second["id"]);
        CONFIG_ASSERT("io/device/" + iter->first + "/can_num", iter->second["can_num"].getType() == XmlRpc::XmlRpcValue::TypeInt && static_cast<int>(iter->second["can_num"]) >= 0);
        ios_[iter->first].canNum = static_cast<int>(iter->second["can_num"]);
        CONFIG_ASSERT("io/device/" + iter->first + "/timeout", iter->second["timeout"].getType() == XmlRpc::XmlRpcValue::TypeDouble && static_cast<double>(iter->second["timeout"]) >= 0);
        ios_[iter->first].timeout = static_cast<double>(iter->second["timeout"]);
        interface_.registerHandle(robot_interface::IOHandle(
            iter->first,
            &ios_[iter->first].currentLevel,
            &ios_[iter->first].targetLevel));
    }
    robotHW_.registerInterface(&interface_);
    return true;
}

void IODriver::shutdown()
{
}

void IODriver::timeoutCallback(std::string& ioName)
{
    ROS_ERROR("IO %s is offline!", ioName.c_str());
    ios_[ioName].isOnline = false;
}

void IODriver::canRXCallback(unsigned int canNum, CANDriver::Frame& f)
{
    if (f.isRemoteTransmission || f.type != CANDriver::Frame::Type::STD || f.id < 0x120) return;
    int idStart = (f.id - 0x120) * 64;
    /* 解析CAN数据得到当前电平状态 */
    for (size_t i = 0; i < f.data.size() * 8; i++) {
        auto iter = ioIDSearchTable_.find(idStart + i);
        if (iter != ioIDSearchTable_.end() && iter->second.id == idStart + i) {
            iter->second.isOnline = true;
            iter->second.timeoutTimer.stop();
            iter->second.timeoutTimer.start();
            iter->second.currentLevel = GET_BIT(f.data[i / 8], i % 8);
        }
    }
}

void IODriver::read(const ros::Time& time, const ros::Duration& period)
{
}

void IODriver::write(const ros::Time& time, const ros::Duration& period)
{
    /* 发送CAN报文 */
    lastSendDuration_ += period;
    if (lastSendDuration_.toSec() >= 1 / sendRate_) {
        lastSendDuration_ = ros::Duration(0);
        /* 该MAP结构为:map[CAN编号][报文ID] = 具体报文 */
        std::map<unsigned int, std::map<unsigned int, CANDriver::Frame>> canFrames;
        for (auto iter = ios_.begin(); iter != ios_.end(); ++iter) {
            unsigned short canID = iter->second.id / 64 + 0x130;
            if (canFrames[iter->second.canNum].find(canID) == canFrames[iter->second.canNum].end()) {
                canFrames[iter->second.canNum][canID].id                   = canID;
                canFrames[iter->second.canNum][canID].type                 = CANDriver::Frame::Type::STD;
                canFrames[iter->second.canNum][canID].isRemoteTransmission = false;
            }
            int byteNum = (iter->second.id % 64) / 8;
            canFrames[iter->second.canNum][canID].data.resize(byteNum + 1, 0);
            SET_BIT(canFrames[iter->second.canNum][canID].data[byteNum], iter->second.id % 8, iter->second.targetLevel ? 1 : 0);
        }
        /* 发送CAN报文 */
        for (auto iter = canFrames.begin(); iter != canFrames.end(); ++iter) {
            for (auto iterFrame = iter->second.begin(); iterFrame != iter->second.end(); iterFrame++) {
                driver_.can->sendFrame(iter->first, iterFrame->second);
            }
        }
    }
}

}  // namespace robot_control