#include <ros/ros.h>

#include <robot_interface/remote_controller.h>
#include <robot_toolbox/tool.h>

#include "robot_control/communication/communication.h"
#include "robot_control/rc.h"

namespace robot_control
{
RCDriver::RCDriver(ros::NodeHandle& node, ros::NodeHandle& nodeParam, std::string urdf, CommunicationDriver& driver, hardware_interface::RobotHW& robotHW, bool& isDisableOutput)
    : ModuleInterface(node, nodeParam, urdf, driver, robotHW, isDisableOutput), node_(node), nodeParam_(nodeParam), urdf_(urdf), driver_(driver), robotHW_(robotHW), isDisableOutput_(isDisableOutput)
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
    lastRecvByteTime_ = ros::Time::now();
    /* 每18字节调用一次解析 */
    int remainLength   = data.size();
    auto dataBeginIter = data.begin();
    serialRecvData_.insert(serialRecvData_.end(), data.begin(), data.end());
    size_t i;
    for (i = 0;i < serialRecvData_.size();) {
        /* 尝试解析包,符合以下条件的包就视为有效包,提高解包有效率 */
        if (serialRecvData_.size() - i < 18) break;
        if (((serialRecvData_[i + 0]      | serialRecvData_[i + 1] << 8) & 0x07FF) <= 1684  &&
            ((serialRecvData_[i + 0]      | serialRecvData_[i + 1] << 8) & 0x07FF) >= 364   &&
            ((serialRecvData_[i + 1] >> 3 | serialRecvData_[i + 2] << 5) & 0x07FF) <= 1684  &&
            ((serialRecvData_[i + 1] >> 3 | serialRecvData_[i + 2] << 5) & 0x07FF) >= 364   &&
            ((serialRecvData_[i + 2] >> 6 | serialRecvData_[i + 3] << 2 | serialRecvData_[i + 4] << 10) & 0x07FF) <= 1684 &&
            ((serialRecvData_[i + 2] >> 6 | serialRecvData_[i + 3] << 2 | serialRecvData_[i + 4] << 10) & 0x07FF) >= 364  &&
            ((serialRecvData_[i + 4] >> 1 | serialRecvData_[i + 5] << 7) & 0x07FF) <= 1684  &&
            ((serialRecvData_[i + 4] >> 1 | serialRecvData_[i + 5] << 7) & 0x07FF) >= 364   &&
            ((serialRecvData_[i + 16]     | serialRecvData_[i + 17] << 8) & 0x07FF) <= 1684 &&
            ((serialRecvData_[i + 16]     | serialRecvData_[i + 17] << 8) & 0x07FF) >= 364  &&
            (((serialRecvData_[i + 5] >> 4) & 0x000C) >> 2) > 0 &&
            (((serialRecvData_[i + 5] >> 4) & 0x000C) >> 2) < 4 &&
            ((serialRecvData_[i + 5] >> 4) & 0x0003) > 0        &&
            ((serialRecvData_[i + 5] >> 4) & 0x0003) < 4        &&
            (serialRecvData_[i + 12] == 0 || data[i + 12] == 1) &&
            (serialRecvData_[i + 13] == 0 || data[i + 13] == 1)
        ) {
            std::vector<uint8_t> realData(serialRecvData_.begin() + i, serialRecvData_.begin() + i + 18);
            parsedData(realData);
            i += 18;
        } else {
            i += 1;
        }
    }
    serialRecvData_.erase(serialRecvData_.begin(), serialRecvData_.begin() + i);
}

void RCDriver::parsedData(std::vector<uint8_t>& data)
{
    ros::Time now = ros::Time::now();
    /* 通道解析,并换算成百分比 */
    ch_[0] = -(((data[0] | data[1] << 8) & 0x07FF) - 1024) / 660.0f;
    ch_[1] = (((data[1] >> 3 | data[2] << 5) & 0x07FF) - 1024) / 660.0f;
    ch_[2] = -(((data[2] >> 6 | data[3] << 2 | data[4] << 10) & 0x07FF) - 1024) / 660.0f;
    ch_[3] = (((data[4] >> 1 | data[5] << 7) & 0x07FF) - 1024) / 660.0f;
    ch_[4] = ((data[16] | data[17] << 8) - 1024) / 660.0f;
    /* 防止遥控器零点有偏差 */
    if (fabs(ch_[0]) < 0.01) ch_[0] = 0;
    if (fabs(ch_[1]) < 0.01) ch_[1] = 0;
    if (fabs(ch_[2]) < 0.01) ch_[2] = 0;
    if (fabs(ch_[3]) < 0.01) ch_[3] = 0;
    if (fabs(ch_[4]) < 0.01) ch_[4] = 0;
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
    /* 鼠标信息解析,注意遥控器返回的数据是反的 */
    ch_[5]  = -(int16_t)(data[6] | (data[7] << 8)) / 32767.0f;
    ABS_LIMIT(ch_[5], 1);
    ch_[6]  = -(int16_t)(data[8] | (data[9] << 8)) / 32767.0f;
    ABS_LIMIT(ch_[6], 1);
    ch_[7]  = -(int16_t)(data[10] | (data[11] << 8)) / 32767.0f;
    ABS_LIMIT(ch_[7], 1);
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
    rcFPS_.fps++;
    if (now - rcFPS_.lastTime > ros::Duration(1)) {
        if (rcFPS_.lastFPS - rcFPS_.fps > 10 && isOnline_) {
            ROS_WARN("Detected RC FPS is lower than previous! fps = %d, last_fps = %d.", rcFPS_.fps, rcFPS_.lastFPS);
        }
        rcFPS_.lastFPS  = rcFPS_.fps;
        rcFPS_.fps      = 0;
        rcFPS_.lastTime = now;
    }
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