#include <ros/ros.h>

#include <robot_interface/remote_controller.h>
#include <robot_toolbox/tool.h>

#include "robot_control/communication/communication.h"
#include "robot_control/rc.h"

namespace robot_control
{
RCDriver::RCDriver(ros::NodeHandle& node, ros::NodeHandle& nodeParam, std::string urdf, CommunicationDriver& driver, hardware_interface::RobotHW& robotHW)
    : ModuleInterface(node, nodeParam, urdf, driver, robotHW), node_(node), nodeParam_(nodeParam), urdf_(urdf), driver_(driver), robotHW_(robotHW)
{
}

bool RCDriver::init()
{
    /* 读取配置 */
    CONFIG_ASSERT("rc/timeout", nodeParam_.getParam("rc/timeout", rcTimeout_));
    CONFIG_ASSERT("rc/serial_num", nodeParam_.getParam("rc/serial_num", rcSerialNum_));
    CONFIG_ASSERT("rc/handle_name", nodeParam_.getParam("rc/handle_name", handleName_));
    /* 定时器 */
    timeoutTimer_ = node_.createTimer(ros::Duration(rcTimeout_), boost::bind(&RCDriver::timeoutCallback, this), true, true);
    driver_.serial->registerRxCallback(boost::bind(&RCDriver::serialRXCallback, this, _1, _2));
    /* 注册接口 */
    interface_.registerHandle(robot_interface::RemoteControllerHandle(handleName_, &isOnline_, &ch_, &sw_));
    robotHW_.registerInterface(&interface_);
    /* 添加通道和开关 */
    ch_.resize(8, 0);
    sw_.resize(20, robot_interface::RemoteControllerHandle::Switch::MID);
    return true;
}

void RCDriver::shutdown()
{
    timeoutCallback();
}

void RCDriver::read(const ros::Time& time, const ros::Duration& period)
{
}

void RCDriver::write(const ros::Time& time, const ros::Duration& period)
{
}

void RCDriver::serialRXCallback(unsigned int serialNum, std::vector<uint8_t>& data)
{
    if (serialNum != rcSerialNum_) return;
    /* 10ms内收到的认为是同一包数据,否则丢弃之前数据 */
    if (ros::Time::now() - lastRecvByteTime_ > ros::Duration(0.01)) {
        serialRecvData_.resize(0);
    }
    lastRecvByteTime_ = ros::Time::now();
    /* 每18字节调用一次解析 */
    int remainLength   = data.size();
    auto dataBeginIter = data.begin();
    while (remainLength > 0) {
        int copyLength = std::min(18 - static_cast<int>(serialRecvData_.size()), remainLength);
        serialRecvData_.insert(serialRecvData_.end(), dataBeginIter, dataBeginIter + copyLength);
        if (serialRecvData_.size() == 18) {
            parsedData(serialRecvData_);
            serialRecvData_.resize(0);
        }
        dataBeginIter += copyLength;
        remainLength -= copyLength;
    }
}

void RCDriver::parsedData(std::vector<uint8_t>& data)
{
    /* 通道解析,并换算成百分比 */
    ch_[0] = (((data[0] | data[1] << 8) & 0x07FF) - 1024) / 660.0f;
    ch_[1] = (((data[1] >> 3 | data[2] << 5) & 0x07FF) - 1024) / 660.0f;
    ch_[2] = (((data[2] >> 6 | data[3] << 2 | data[4] << 10) & 0x07FF) - 1024) / 660.0f;
    ch_[3] = (((data[4] >> 1 | data[5] << 7) & 0x07FF) - 1024) / 660.0f;
    ch_[4] = ((data[16] | data[17] << 8) - 1024) / 660.0f;
    /* 防止遥控器零点有偏差 */
    if (fabs(ch_[0]) < 0.01) ch_[0] = 0;
    if (fabs(ch_[1]) < 0.01) ch_[1] = 0;
    if (fabs(ch_[2]) < 0.01) ch_[2] = 0;
    if (fabs(ch_[3]) < 0.01) ch_[3] = 0;
    if (fabs(ch_[4]) < 0.01) ch_[4] = 0;
    /* 遥控器数据不能异常 */
    if (fabs(ch_[0]) > 1 || fabs(ch_[1]) > 1 || fabs(ch_[2]) > 1 || fabs(ch_[3]) > 1) return;
    /* 开关解析,分别是左右开关 */
    std::array<int, 2> swValues = {((data[5] >> 4) & 0x000C) >> 2, (data[5] >> 4) & 0x0003};
    for (size_t i = 0; i < swValues.size(); i++) {
        if (swValues[i] == 1) {
            sw_[i] = robot_interface::RemoteControllerHandle::Switch::UP;
        } else if (swValues[i] == 3) {
            sw_[i] = robot_interface::RemoteControllerHandle::Switch::MID;
        } else if (swValues[i] == 2) {
            sw_[i] = robot_interface::RemoteControllerHandle::Switch::DOWN;
        }
    }
    /* 鼠标信息解析 */
    ch_[5]  = data[6] | (data[7] << 8);
    ch_[6]  = data[8] | (data[9] << 8);
    ch_[7]  = data[10] | (data[11] << 8);
    sw_[18] = data[12] == 0 ? robot_interface::RemoteControllerHandle::Switch::MID : robot_interface::RemoteControllerHandle::Switch::UP;
    sw_[19] = data[13] == 0 ? robot_interface::RemoteControllerHandle::Switch::MID : robot_interface::RemoteControllerHandle::Switch::UP;
    /* 键盘信息解析 */
    /* 依次是W/S/A/D/SHIFT/CTRL/Q/E/R/F/G/Z/X/C/V/B */
    sw_[2]  = GET_BIT(data[14], 0) == 0 ? robot_interface::RemoteControllerHandle::Switch::MID : robot_interface::RemoteControllerHandle::Switch::UP;
    sw_[3]  = GET_BIT(data[14], 1) == 0 ? robot_interface::RemoteControllerHandle::Switch::MID : robot_interface::RemoteControllerHandle::Switch::UP;
    sw_[4]  = GET_BIT(data[14], 2) == 0 ? robot_interface::RemoteControllerHandle::Switch::MID : robot_interface::RemoteControllerHandle::Switch::UP;
    sw_[5]  = GET_BIT(data[14], 3) == 0 ? robot_interface::RemoteControllerHandle::Switch::MID : robot_interface::RemoteControllerHandle::Switch::UP;
    sw_[6]  = GET_BIT(data[14], 4) == 0 ? robot_interface::RemoteControllerHandle::Switch::MID : robot_interface::RemoteControllerHandle::Switch::UP;
    sw_[7]  = GET_BIT(data[14], 5) == 0 ? robot_interface::RemoteControllerHandle::Switch::MID : robot_interface::RemoteControllerHandle::Switch::UP;
    sw_[8]  = GET_BIT(data[14], 6) == 0 ? robot_interface::RemoteControllerHandle::Switch::MID : robot_interface::RemoteControllerHandle::Switch::UP;
    sw_[9]  = GET_BIT(data[14], 7) == 0 ? robot_interface::RemoteControllerHandle::Switch::MID : robot_interface::RemoteControllerHandle::Switch::UP;
    sw_[10] = GET_BIT(data[15], 0) == 0 ? robot_interface::RemoteControllerHandle::Switch::MID : robot_interface::RemoteControllerHandle::Switch::UP;
    sw_[11] = GET_BIT(data[15], 1) == 0 ? robot_interface::RemoteControllerHandle::Switch::MID : robot_interface::RemoteControllerHandle::Switch::UP;
    sw_[12] = GET_BIT(data[15], 2) == 0 ? robot_interface::RemoteControllerHandle::Switch::MID : robot_interface::RemoteControllerHandle::Switch::UP;
    sw_[13] = GET_BIT(data[15], 3) == 0 ? robot_interface::RemoteControllerHandle::Switch::MID : robot_interface::RemoteControllerHandle::Switch::UP;
    sw_[14] = GET_BIT(data[15], 4) == 0 ? robot_interface::RemoteControllerHandle::Switch::MID : robot_interface::RemoteControllerHandle::Switch::UP;
    sw_[15] = GET_BIT(data[15], 5) == 0 ? robot_interface::RemoteControllerHandle::Switch::MID : robot_interface::RemoteControllerHandle::Switch::UP;
    sw_[16] = GET_BIT(data[15], 6) == 0 ? robot_interface::RemoteControllerHandle::Switch::MID : robot_interface::RemoteControllerHandle::Switch::UP;
    sw_[17] = GET_BIT(data[15], 7) == 0 ? robot_interface::RemoteControllerHandle::Switch::MID : robot_interface::RemoteControllerHandle::Switch::UP;
    if (!isOnline_) {
        ROS_INFO("Remote Controller connected!");
    }
    isOnline_ = true;
    /* 重置计时器 */
    timeoutTimer_.stop();
    timeoutTimer_.start();
}

void RCDriver::timeoutCallback()
{
    /* 把所有遥控器数据恢复到默认值 */
    std::fill(ch_.begin(), ch_.end(), 0);
    std::fill(sw_.begin(), sw_.end(), robot_interface::RemoteControllerHandle::Switch::MID);
    isOnline_ = false;
    ROS_ERROR("Remote Controller disconnected!");
}

}  // namespace robot_control