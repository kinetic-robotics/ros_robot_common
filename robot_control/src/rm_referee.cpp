#include <ros/ros.h>

#include <robot_interface/rm_referee.h>
#include <robot_toolbox/crc.h>
#include <robot_toolbox/tool.h>

#include "robot_control/communication/communication.h"
#include "robot_control/rm_referee.h"

namespace robot_control
{
RMRefereeDriver::RMRefereeDriver(ros::NodeHandle& node, ros::NodeHandle& nodeParam, std::string urdf, CommunicationDriver& driver, hardware_interface::RobotHW& robotHW, bool& isDisableOutput)
    : ModuleInterface(node, nodeParam, urdf, driver, robotHW, isDisableOutput), node_(node), nodeParam_(nodeParam), urdf_(urdf), driver_(driver), robotHW_(robotHW), isDisableOutput_(isDisableOutput)
{
}

bool RMRefereeDriver::init()
{
    /* 读取配置 */
    CONFIG_ASSERT("rm_referee/timeout", nodeParam_.getParam("rm_referee/timeout", timeout_));
    CONFIG_ASSERT("rm_referee/serial_num", nodeParam_.getParam("rm_referee/serial_num", refereeSerialNum_));
    CONFIG_ASSERT("rm_referee/handle_name", nodeParam_.getParam("rm_referee/handle_name", handleName_));
    int gameGroup, robotType = 0;
    CONFIG_ASSERT("rm_referee/default/game_group", nodeParam_.getParam("rm_referee/default/game_group", gameGroup) && (gameGroup == 1 || gameGroup == 2));
    CONFIG_ASSERT("rm_referee/default/robot_type", nodeParam_.getParam("rm_referee/default/robot_type", robotType) && robotType > 0 && robotType < 10);
    refereeData_.robotStatus.group = static_cast<robot_interface::RMRefereeHandle::GameGroup>(gameGroup);
    refereeData_.robotStatus.type  = static_cast<robot_interface::RMRefereeHandle::RobotType>(robotType);
    /* 定时器 */
    timeoutTimer_ = node_.createTimer(ros::Duration(timeout_), boost::bind(&RMRefereeDriver::timeoutCallback, this), true, true);
    driver_.serial->registerRxCallback(boost::bind(&RMRefereeDriver::serialRXCallback, this, _1, _2));
    /* 注册接口 */
    interface_.registerHandle(robot_interface::RMRefereeHandle(
        handleName_, &refereeData_, &gameResultCallback_, &dartLaunchCallback_, &supplyProjectileActionCallback_, &refereeWarningCallback_, &hurtCallback_, &shootCallback_,
        &interactiveCallback_, &customControllerCallback_, boost::bind(&RMRefereeDriver::sendGraphUIFunction, this, _1),
        boost::bind(&RMRefereeDriver::deleteLayerUIFunction, this, _1, _2), boost::bind(&RMRefereeDriver::sendInteractiveFunction, this, _1, _2, _3)));
    robotHW_.registerInterface(&interface_);
    return true;
}

void RMRefereeDriver::shutdown()
{
}

void RMRefereeDriver::read(const ros::Time& time, const ros::Duration& period)
{
}

void RMRefereeDriver::write(const ros::Time& time, const ros::Duration& period)
{
}

void RMRefereeDriver::addInteractiveHeader(int cmdID, int recvID, std::vector<uint8_t>& data)
{
    int senderID = refereeData_.robotStatus.group == robot_interface::RMRefereeHandle::GameGroup::BLUE ? 100 + static_cast<int>(refereeData_.robotStatus.type) : static_cast<int>(refereeData_.robotStatus.type);
    data.push_back(cmdID);
    data.push_back(cmdID >> 8);
    data.push_back(senderID);
    data.push_back(senderID >> 8);
    data.push_back(recvID);
    data.push_back(recvID >> 8);
}

bool RMRefereeDriver::checkInteractiveSendFrequent()
{
    if (interactiveCount_ == 10) {
        /* 1秒,10hz */
        if (ros::Time::now() - interactiveCountCleanTime_ < ros::Duration(1)) return false;
        interactiveCount_ = 0;
        interactiveCountCleanTime_ = ros::Time::now();
    }
    interactiveCount_++;
    return true;
}

size_t RMRefereeDriver::sendGraphUIFunction(std::vector<robot_interface::RMRefereeHandle::UIData>& data)
{
    /* 为什么不用DP?因为DP的话,时间复杂度没有降低,而只是牺牲了一点点传输性能的代价,并且可维护性提高不少 */
    std::vector<robot_interface::RMRefereeHandle::UIData> noStringData; /* 不是字符类型的数据 */
    std::vector<robot_interface::RMRefereeHandle::UIData> stringData;   /* 是字符类型的数据 */
    for (size_t i = 0; i < data.size(); i++) {
        if (data[i].type == robot_interface::RMRefereeHandle::GraphType::STRING) {
            stringData.push_back(data[i]);
        } else {
            noStringData.push_back(data[i]);
        }
    }
    std::vector<uint8_t> sendData;
    int recvID = refereeData_.robotStatus.group == robot_interface::RMRefereeHandle::GameGroup::BLUE ? 0x0164 + static_cast<int>(refereeData_.robotStatus.type) : 0x0100 + static_cast<int>(refereeData_.robotStatus.type);
    for (size_t i = 0; i < noStringData.size(); i++) {
        /* 每7次图形更新触发添加包头 */
        if (i % 7 == 0) {
            sendData.resize(0);
            if (!checkInteractiveSendFrequent()) return i;
            addInteractiveHeader(RM_REFEREE_INTERACTIVE_UPDATE_7_GRAPH, recvID, sendData);
        }
        uint32_t config1 = static_cast<uint32_t>(noStringData[i].method) | static_cast<uint32_t>(noStringData[i].type) << 3 | noStringData[i].layer << 6 | static_cast<uint32_t>(noStringData[i].color) << 9;
        uint32_t config2 = 0, config3 = 0;
        int startAngle, endAngle;
        /* 打包数据 */
        switch (noStringData[i].type) {
            case robot_interface::RMRefereeHandle::GraphType::LINE:
                if (noStringData[i].line.width > 1023 || noStringData[i].line.start.x > 2047 || noStringData[i].line.start.y > 2047 ||
                    noStringData[i].line.end.x > 2047 || noStringData[i].line.end.y > 2047) {
                    ROS_ERROR("Referee Systrem Send UI Function failed, because line parameter error!");
                    continue;
                }
                config2 = noStringData[i].line.width | noStringData[i].line.start.x << 10 | noStringData[i].line.start.y << 21;
                config3 = noStringData[i].line.end.x << 10 | noStringData[i].line.end.y << 21;
                break;
            case robot_interface::RMRefereeHandle::GraphType::RECTANGLE:
                if (noStringData[i].rectangle.width > 1023 || noStringData[i].rectangle.start.x > 2047 || noStringData[i].rectangle.start.y > 2047 ||
                    noStringData[i].rectangle.end.x > 2047 || noStringData[i].rectangle.end.y > 2047) {
                    ROS_ERROR("Referee Systrem Send UI Function failed, because rectangle parameter error!");
                    continue;
                }
                config2 = noStringData[i].rectangle.width | noStringData[i].rectangle.start.x << 10 | noStringData[i].rectangle.start.y << 21;
                config3 = noStringData[i].rectangle.end.x << 10 | noStringData[i].rectangle.end.y << 21;
                break;
            case robot_interface::RMRefereeHandle::GraphType::CIRCLE:
                if (noStringData[i].circle.width > 1023 || noStringData[i].circle.center.x > 2047 || noStringData[i].circle.center.y > 2047 ||
                    noStringData[i].circle.radius > 1023) {
                    ROS_ERROR("Referee Systrem Send UI Function failed, because rectangle parameter error!");
                    continue;
                }
                config2 = noStringData[i].circle.width | noStringData[i].circle.center.x << 10 | noStringData[i].circle.center.y << 21;
                config3 = noStringData[i].circle.radius;
                break;
            case robot_interface::RMRefereeHandle::GraphType::OVAL:
                if (noStringData[i].oval.width > 1023 || noStringData[i].oval.center.x > 2047 || noStringData[i].oval.center.y > 2047 ||
                    noStringData[i].oval.xRadius > 2047 || noStringData[i].oval.yRadius > 2047) {
                    ROS_ERROR("Referee Systrem Send UI Function failed, because oval parameter error!");
                    continue;
                }
                config2 = noStringData[i].oval.width | noStringData[i].oval.center.x << 10 | noStringData[i].oval.center.y << 21;
                config3 = noStringData[i].oval.xRadius << 10 | noStringData[i].oval.yRadius << 21;
                break;
            case robot_interface::RMRefereeHandle::GraphType::ARC:
                startAngle = noStringData[i].arc.angle.start / M_PI * 180;
                endAngle = noStringData[i].arc.angle.end / M_PI * 180;
                if (noStringData[i].arc.width > 1023 || noStringData[i].arc.center.x > 2047 || noStringData[i].arc.center.y > 2047 ||
                    noStringData[i].arc.xRadius > 2047 || noStringData[i].arc.yRadius > 2047 || startAngle > 360 || startAngle < 0 || endAngle > 360 || endAngle < 0) {
                    ROS_ERROR("Referee Systrem Send UI Function failed, because arc parameter error!");
                    continue;
                }
                config1 = startAngle << 14 | endAngle << 23;
                config2 = noStringData[i].arc.width | noStringData[i].arc.center.x << 10 | noStringData[i].arc.center.y << 21;
                config3 = noStringData[i].arc.xRadius << 10 | noStringData[i].arc.yRadius << 21;
                break;
            case robot_interface::RMRefereeHandle::GraphType::FLOAT_NUMBER:
                if (noStringData[i].floatNumber.width > 1023 || noStringData[i].floatNumber.start.x > 2047 || noStringData[i].floatNumber.start.y > 2047 ||
                    noStringData[i].floatNumber.fontSize > 511 || noStringData[i].floatNumber.digits > 511) {
                    ROS_ERROR("Referee Systrem Send UI Function failed, because float number parameter error!");
                    continue;
                }
                config1 = noStringData[i].floatNumber.fontSize << 14 | noStringData[i].floatNumber.digits << 23;
                config2 = noStringData[i].floatNumber.width | noStringData[i].floatNumber.start.x << 10 | noStringData[i].floatNumber.start.y << 21;
                config3 = noStringData[i].floatNumber.data * 1000;
                break;
            case robot_interface::RMRefereeHandle::GraphType::INT_NUMBER:
                if (noStringData[i].intNumber.width > 1023 || noStringData[i].intNumber.start.x > 2047 || noStringData[i].intNumber.start.y > 2047 ||
                    noStringData[i].intNumber.fontSize > 511) {
                    ROS_ERROR("Referee Systrem Send UI Function failed, because int number parameter error!");
                    continue;
                }
                config1 = noStringData[i].intNumber.fontSize << 14;
                config2 = noStringData[i].intNumber.width | noStringData[i].intNumber.start.x << 10 | noStringData[i].intNumber.start.y << 21;
                config3 = noStringData[i].intNumber.data;
                break;
        }
        sendData.push_back(noStringData[i].name);
        sendData.push_back(noStringData[i].name >> 8);
        sendData.push_back(noStringData[i].name >> 16);
        sendData.push_back(config1);
        sendData.push_back(config1 >> 8);
        sendData.push_back(config1 >> 16);
        sendData.push_back(config1 >> 24);
        sendData.push_back(config2);
        sendData.push_back(config2 >> 8);
        sendData.push_back(config2 >> 16);
        sendData.push_back(config2 >> 24);
        sendData.push_back(config3);
        sendData.push_back(config3 >> 8);
        sendData.push_back(config3 >> 16);
        sendData.push_back(config3 >> 24);
        if (i % 7 == 6 || i == noStringData.size() - 1) {
            sendData.resize(111, 0);
            sendFrame(RM_REFEREE_INTERACTIVE, sendData);
        }
    }
    return data.size();
}

bool RMRefereeDriver::deleteLayerUIFunction(robot_interface::RMRefereeHandle::GraphDeleteCMD cmd, int layer)
{
    if (!checkInteractiveSendFrequent()) return false;
    std::vector<uint8_t> sendData;
    int recvID = refereeData_.robotStatus.group == robot_interface::RMRefereeHandle::GameGroup::BLUE ? 0x0164 + static_cast<int>(refereeData_.robotStatus.type) : 0x0100 + static_cast<int>(refereeData_.robotStatus.type);
    addInteractiveHeader(RM_REFEREE_INTERACTIVE_DELETE_GRAPH, recvID, sendData);
    sendData.push_back(static_cast<uint8_t>(cmd));
    sendData.push_back(static_cast<uint8_t>(layer));
    sendFrame(RM_REFEREE_INTERACTIVE, sendData);
    return true;
}

bool RMRefereeDriver::sendInteractiveFunction(int cmdID, robot_interface::RMRefereeHandle::RobotType type, std::vector<uint8_t>& data)
{
    if (cmdID < 0x0200 || cmdID > 0x02FF || data.size() > 113) {
        ROS_ERROR("Referee Systrem Send Interactive failed, because parameter error!");
        return false;
    }
    if (!checkInteractiveSendFrequent()) return false;
    std::vector<uint8_t> sendData;
    int recvID = refereeData_.robotStatus.group == robot_interface::RMRefereeHandle::GameGroup::BLUE ? 100 + static_cast<int>(type) : static_cast<int>(type);
    addInteractiveHeader(cmdID, recvID, sendData);
    sendData.insert(sendData.end(), data.begin(), data.end());
    sendFrame(RM_REFEREE_INTERACTIVE, sendData);
    return true;
}

void RMRefereeDriver::sendFrame(uint16_t cmdID, std::vector<uint8_t>& data)
{
    std::vector<uint8_t> sendData;
    sendData.push_back(0xA5);
    sendData.push_back(data.size());
    sendData.push_back(data.size() >> 8);
    sendData.push_back(lastSendSeq_++);
    sendData.push_back(robot_toolbox::CRC::getCRC8(sendData));
    sendData.push_back(cmdID);
    sendData.push_back(cmdID >> 8);
    sendData.insert(sendData.end(), data.begin(), data.end());
    int crc16 = robot_toolbox::CRC::getCRC16(sendData);
    sendData.push_back(crc16);
    sendData.push_back(crc16 >> 8);
    driver_.serial->send(refereeSerialNum_, sendData);
}

void RMRefereeDriver::serialRXCallback(unsigned int serialNum, std::vector<uint8_t>& data)
{
    if (serialNum != refereeSerialNum_) return;
    for (size_t i = 0; i < data.size(); i++) {
        uint8_t oneData = data[i];
        receivePacket_.data.push_back(oneData);
        std::vector<uint8_t> crc8Data, crc16Data, packetData;
        /* 裁判系统解包状态机 */
        switch (receiveMachineState_) {
            case ReceiveMachineState::SOF:
                receivePacket_.data.resize(0);
                receivePacket_.data.push_back(oneData);
                if (oneData == 0xA5) {
                    receiveMachineState_ = ReceiveMachineState::LENGTH_1;
                }
                break;
            case ReceiveMachineState::LENGTH_1:
                receivePacket_.targetLength = oneData;
                receiveMachineState_        = ReceiveMachineState::LENGTH_2;
                break;
            case ReceiveMachineState::LENGTH_2:
                receivePacket_.targetLength |= oneData << 8;
                receiveMachineState_ = ReceiveMachineState::SEQ;
                break;
            case ReceiveMachineState::SEQ:
                receivePacket_.seq   = oneData;
                receiveMachineState_ = ReceiveMachineState::CRC8;
                break;
            case ReceiveMachineState::CRC8:
                crc8Data = std::vector<uint8_t>(receivePacket_.data.begin(), receivePacket_.data.begin() + 4);
                if (robot_toolbox::CRC::getCRC8(crc8Data) == oneData) {
                    receiveMachineState_ = ReceiveMachineState::CMD_ID_1;
                } else {
                    ROS_ERROR("Referee System packet header CRC8 verification failed! %d %d", robot_toolbox::CRC::getCRC8(crc8Data), oneData);
                    receiveMachineState_ = ReceiveMachineState::SOF;
                }
                break;
            case ReceiveMachineState::CMD_ID_1:
                receivePacket_.cmdID = oneData;
                receiveMachineState_ = ReceiveMachineState::CMD_ID_2;
                break;
            case ReceiveMachineState::CMD_ID_2:
                receivePacket_.cmdID |= oneData << 8;
                receiveMachineState_ = ReceiveMachineState::DATA;
                break;
            case ReceiveMachineState::DATA:
                if (receivePacket_.data.size() == 7 + receivePacket_.targetLength) {
                    receiveMachineState_ = ReceiveMachineState::CRC16_1;
                }
                break;
            case ReceiveMachineState::CRC16_1:
                receivePacket_.crc16 = oneData;
                receiveMachineState_ = ReceiveMachineState::CRC16_2;
                break;
            case ReceiveMachineState::CRC16_2:
                receivePacket_.crc16 |= oneData << 8;
                crc16Data = std::vector<uint8_t>(receivePacket_.data.begin(), receivePacket_.data.end() - 2);
                if (robot_toolbox::CRC::getCRC16(crc16Data) == receivePacket_.crc16) {
                    if (!refereeData_.isOnline) {
                        ROS_INFO("Referee System connected!");
                    }
                    refereeData_.isOnline = true;
                    /* 重置计时器 */
                    timeoutTimer_.stop();
                    timeoutTimer_.start();
                    packetData = std::vector<uint8_t>(receivePacket_.data.begin() + 7, receivePacket_.data.end() - 2);
                    parsedData(receivePacket_.cmdID, receivePacket_.seq, packetData);
                } else {
                    ROS_ERROR("Referee System packet tail CRC16 verification failed!");
                }
                receiveMachineState_ = ReceiveMachineState::SOF;
                break;
        }
    }
}

void RMRefereeDriver::parsedData(int cmdID, int seq, std::vector<uint8_t>& data)
{
    /* 去重复数据 */
    if (lastRecvSeq_ == seq) {
        ROS_ERROR("Referee System packet duplicated!");
        return;
    }
    /* 丢包分析 */
    if (lastRecvSeq_ != -1 && (seq - lastRecvSeq_ > 1 || (lastRecvSeq_ == 255 && seq > 0))) {
        ROS_ERROR("Referee System packet losted, we lost %d packets!", lastRecvSeq_ == 255 ? seq : seq - lastRecvSeq_ - 1);
        return;
    }
    lastRecvSeq_ = seq;
    /* 分包数据解析 */
    /* 由于switch分支中不能放变量,所以统一存放在这,并不是公用的 */
    std::vector<uint8_t> interactiveData;
    uint32_t status, type = 0, robotID;
    double bulletSpeed = 0;
    switch (cmdID) {
        case RM_REFEREE_GAME_STATUS:
            if (data.size() < 3) {
                ROS_ERROR("Referee System Game Status packet length too short!");
                return;
            }
            refereeData_.gameStatus.type            = static_cast<robot_interface::RMRefereeHandle::GameType>(GET_BITS(data[0], 0, 3));
            refereeData_.gameStatus.process         = static_cast<robot_interface::RMRefereeHandle::GameProcess>(GET_BITS(data[0], 4, 7));
            refereeData_.gameStatus.stageRemainTime = ros::Duration(data[1] | data[2] << 8);
            if (data.size() > 3) {
                uint64_t time                = data[3] | data[4] << 8 | data[5] << 16 | data[6] << 24 | (uint64_t)data[7] << 32 | (uint64_t)data[8] << 40 | (uint64_t)data[9] << 48 | (uint64_t)data[10] << 56;
                refereeData_.gameStatus.syncTime = ros::Time(time);
            }
            break;
        case RM_REFEREE_GAME_RESULT:
            if (data.size() < 1) {
                ROS_ERROR("Referee System Game Result packet length too short!");
                return;
            }
            if (gameResultCallback_) gameResultCallback_(static_cast<robot_interface::RMRefereeHandle::GameResult>(data[0]));
            break;
        case RM_REFEREE_HP:
            if (data.size() < 16) {
                ROS_ERROR("Referee System HP packet length too short!");
                return;
            }
            refereeData_.hp.red.hero1      = data[0] | data[1] << 8;
            refereeData_.hp.red.engineer2  = data[2] | data[3] << 8;
            refereeData_.hp.red.standard3  = data[4] | data[5] << 8;
            refereeData_.hp.red.standard4  = data[6] | data[7] << 8;
            refereeData_.hp.red.standard5  = data[8] | data[9] << 8;
            refereeData_.hp.red.sentry7    = data[10] | data[11] << 8;
            refereeData_.hp.red.outpost    = data[12] | data[13] << 8;
            refereeData_.hp.red.base       = data[14] | data[15] << 8;
            refereeData_.hp.blue.hero1     = data[0] | data[1] << 8;
            refereeData_.hp.blue.engineer2 = data[2] | data[3] << 8;
            refereeData_.hp.blue.standard3 = data[4] | data[5] << 8;
            refereeData_.hp.blue.standard4 = data[6] | data[7] << 8;
            refereeData_.hp.blue.standard5 = data[8] | data[9] << 8;
            refereeData_.hp.blue.sentry7   = data[10] | data[11] << 8;
            refereeData_.hp.blue.outpost   = data[12] | data[13] << 8;
            refereeData_.hp.blue.base      = data[14] | data[15] << 8;
            break;
        case RM_REFEREE_DART_LAUNCH:
            if (data.size() < 3) {
                ROS_ERROR("Referee System Dart Launch packet length too short!");
                return;
            }
            if (dartLaunchCallback_) dartLaunchCallback_(static_cast<robot_interface::RMRefereeHandle::GameGroup>(data[0]), ros::Duration(data[1] | data[2] << 8));
            break;
        case RM_REFEREE_ICRA:
            if (data.size() < 11) {
                ROS_ERROR("Referee System ICRA packet length too short!");
                return;
            }
            status                                  = data[0] | data[1] << 8 | data[2] << 16;
            refereeData_.icraStatus.buff.f1.isActive      = GET_BIT(status, 0);
            refereeData_.icraStatus.buff.f1.type          = static_cast<robot_interface::RMRefereeHandle::ICRABuffType>(GET_BITS(status, 1, 3));
            refereeData_.icraStatus.buff.f2.isActive      = GET_BIT(status, 4);
            refereeData_.icraStatus.buff.f2.type          = static_cast<robot_interface::RMRefereeHandle::ICRABuffType>(GET_BITS(status, 5, 7));
            refereeData_.icraStatus.buff.f3.isActive      = GET_BIT(status, 8);
            refereeData_.icraStatus.buff.f3.type          = static_cast<robot_interface::RMRefereeHandle::ICRABuffType>(GET_BITS(status, 9, 11));
            refereeData_.icraStatus.buff.f4.isActive      = GET_BIT(status, 12);
            refereeData_.icraStatus.buff.f4.type          = static_cast<robot_interface::RMRefereeHandle::ICRABuffType>(GET_BITS(status, 13, 15));
            refereeData_.icraStatus.buff.f5.isActive      = GET_BIT(status, 16);
            refereeData_.icraStatus.buff.f5.type          = static_cast<robot_interface::RMRefereeHandle::ICRABuffType>(GET_BITS(status, 17, 19));
            refereeData_.icraStatus.buff.f6.isActive      = GET_BIT(status, 20);
            refereeData_.icraStatus.buff.f6.type          = static_cast<robot_interface::RMRefereeHandle::ICRABuffType>(GET_BITS(status, 21, 23));
            refereeData_.icraStatus.remainingBullet.red1  = data[3] | data[4] << 8;
            refereeData_.icraStatus.remainingBullet.red2  = data[5] | data[6] << 8;
            refereeData_.icraStatus.remainingBullet.blue1 = data[7] | data[8] << 8;
            refereeData_.icraStatus.remainingBullet.blue1 = data[9] | data[10] << 8;
            break;
        case RM_REFEREE_EVENT:
            if (data.size() < 4) {
                ROS_ERROR("Referee System Event packet length too short!");
                return;
            }
            type                                               = data[0] | data[1] << 8 | data[2] << 16 | data[3] << 24;
            refereeData_.event.restoration.isFirstOccupied     = GET_BIT(type, 0);
            refereeData_.event.restoration.isSecondOccupied    = GET_BIT(type, 1);
            refereeData_.event.restoration.isThirdOccupied     = GET_BIT(type, 2);
            refereeData_.event.powerRune.isAttackPointOccupied = GET_BIT(type, 3);
            refereeData_.event.powerRune.isSmallActive         = GET_BIT(type, 4);
            refereeData_.event.powerRune.isBigActive           = GET_BIT(type, 5);
            refereeData_.event.elevated.is2Occupied            = GET_BIT(type, 6);
            refereeData_.event.elevated.is3Occupied            = GET_BIT(type, 7);
            refereeData_.event.elevated.is4Occupied            = GET_BIT(type, 8);
            refereeData_.event.isVirtualShieldActive           = GET_BIT(type, 9);
            refereeData_.event.isOutpostActive                 = GET_BIT(type, 10);
            break;
        case RM_REFEREE_SUPPLY_ACTION:
            if (data.size() < 4) {
                ROS_ERROR("Referee System Supply Action packet length too short!");
                return;
            }
            if (supplyProjectileActionCallback_) supplyProjectileActionCallback_(
                data[0],
                data[1] != 0,
                data[1] != 0 ? static_cast<robot_interface::RMRefereeHandle::RobotType>(data[1] > 100 ? data[1] - 100 : data[1]) : robot_interface::RMRefereeHandle::RobotType::HERO,
                static_cast<robot_interface::RMRefereeHandle::SupplyProjectileStep>(data[2]),
                data[3]);
            break;
        case RM_REFEREE_REFEREE_WARNING:
            if (data.size() < 2) {
                ROS_ERROR("Referee System Referee Warning packet length too short!");
                return;
            }
            if (refereeWarningCallback_) refereeWarningCallback_(
                static_cast<robot_interface::RMRefereeHandle::RefereeWarning>(data[0]),
                data[1] != 0 ? static_cast<robot_interface::RMRefereeHandle::RobotType>(data[0] > 100 ? data[0] - 100 : data[0]) : robot_interface::RMRefereeHandle::RobotType::HERO);
            break;
        case RM_REFEREE_DART_COUNTDOWN:
            if (data.size() < 1) {
                ROS_ERROR("Referee System Dart Remaining Time packet length too short!");
                return;
            }
            refereeData_.dartRemainingTime = ros::Duration(data[0]);
            break;
        case RM_REFEREE_ROBOT_STATUS:
            if (data.size() < 27) {
                ROS_ERROR("Referee System Robot Status packet length too short!");
                return;
            }
            refereeData_.robotStatus.group                           = data[0] > 100 ? robot_interface::RMRefereeHandle::GameGroup::BLUE : robot_interface::RMRefereeHandle::GameGroup::RED;
            refereeData_.robotStatus.type                            = static_cast<robot_interface::RMRefereeHandle::RobotType>(data[0] > 100 ? data[0] - 100 : data[0]);
            refereeData_.robotStatus.level                           = data[1];
            refereeData_.robotStatus.hp                              = data[2] | data[3] << 8;
            refereeData_.robotStatus.maxHP                           = data[4] | data[5] << 8;
            refereeData_.robotStatus.shooter.first17mm.coolingRate   = data[6] | data[7] << 8;
            refereeData_.robotStatus.shooter.first17mm.coolingLimit  = data[8] | data[9] << 8;
            refereeData_.robotStatus.shooter.first17mm.speedLimit    = data[10] | data[11] << 8;
            refereeData_.robotStatus.shooter.second17mm.coolingRate  = data[12] | data[13] << 8;
            refereeData_.robotStatus.shooter.second17mm.coolingLimit = data[14] | data[15] << 8;
            refereeData_.robotStatus.shooter.second17mm.speedLimit   = data[16] | data[17] << 8;
            refereeData_.robotStatus.shooter.first42mm.coolingRate   = data[18] | data[19] << 8;
            refereeData_.robotStatus.shooter.first42mm.coolingLimit  = data[20] | data[21] << 8;
            refereeData_.robotStatus.shooter.first42mm.speedLimit    = data[22] | data[23] << 8;
            refereeData_.robotStatus.chassisPowerLimit               = data[24] | data[25] << 8;
            refereeData_.robotStatus.power.gimbal                    = GET_BIT(data[26], 0);
            refereeData_.robotStatus.power.chassis                   = GET_BIT(data[26], 1);
            refereeData_.robotStatus.power.shooter                   = GET_BIT(data[26], 2);
            break;
        case RM_REFEREE_POWER_HEAT:
            if (data.size() < 16) {
                ROS_ERROR("Referee System Power Heat packet length too short!");
                return;
            }
            refereeData_.powerHeatData.voltage = (data[0] | data[1] << 8) / 1000.0f;
            refereeData_.powerHeatData.current = (data[2] | data[3] << 8) / 1000.0f;
            CONVERT_BYTES_TO_FLOAT(data, 4, refereeData_.powerHeatData.power);
            refereeData_.powerHeatData.buffer                 = data[9] | data[10] << 8;
            refereeData_.powerHeatData.shooterHeat.first17mm  = data[11] | data[12] << 8;
            refereeData_.powerHeatData.shooterHeat.second17mm = data[13] | data[14] << 8;
            refereeData_.powerHeatData.shooterHeat.first42mm  = data[15] | data[16] << 8;
            break;
        case RM_REFEREE_POSITION:
            if (data.size() < 16) {
                ROS_ERROR("Referee System Position packet length too short!");
                return;
            }
            CONVERT_BYTES_TO_FLOAT(data, 0, refereeData_.position.x);
            CONVERT_BYTES_TO_FLOAT(data, 4, refereeData_.position.y);
            CONVERT_BYTES_TO_FLOAT(data, 8, refereeData_.position.z);
            CONVERT_BYTES_TO_FLOAT(data, 12, refereeData_.position.shooterYaw);
            /* 转换为弧度 */
            refereeData_.position.shooterYaw = refereeData_.position.shooterYaw / 180.0f * M_PI;
            break;
        case RM_REFEREE_BUFF:
            if (data.size() < 1) {
                ROS_ERROR("Referee System Buff packet length too short!");
                return;
            }
            refereeData_.buff.isHealing              = GET_BIT(data[0], 0);
            refereeData_.buff.isShooterCoolingFaster = GET_BIT(data[0], 1);
            refereeData_.buff.isDefenseBuff          = GET_BIT(data[0], 2);
            refereeData_.buff.isAttackBuff           = GET_BIT(data[0], 3);
            break;
        case RM_REFEREE_AERIAL_POWER:
            if (data.size() < 1) {
                ROS_ERROR("Referee System Aerial Power packet length too short!");
                return;
            }
            refereeData_.aerialAttackTime = ros::Duration(data[0]);
            break;
        case RM_REFEREE_HURT:
            if (data.size() < 1) {
                ROS_ERROR("Referee System Hurt packet length too short!");
                return;
            }
            if (hurtCallback_) hurtCallback_(GET_BITS(data[0], 0, 3), static_cast<robot_interface::RMRefereeHandle::HurtType>(GET_BITS(data[0], 4, 7)));
            break;
        case RM_REFEREE_SHOOT:
            if (data.size() < 7) {
                ROS_ERROR("Referee System Shoot packet length too short!");
                return;
            }
            CONVERT_BYTES_TO_FLOAT(data, 3, bulletSpeed);
            if (shootCallback_) shootCallback_(
                static_cast<robot_interface::RMRefereeHandle::BulletType>(data[0]),
                static_cast<robot_interface::RMRefereeHandle::ShooterType>(data[1]),
                data[2],
                bulletSpeed);
            break;
        case RM_REFEREE_BULLET_REMAINING:
            if (data.size() < 6) {
                ROS_ERROR("Referee System Bullet Remaining packet length too short!");
                return;
            }
            refereeData_.bulletRemaininggStatus.all17mmRemainingNum = data[0] | data[1] << 8;
            refereeData_.bulletRemaininggStatus.all42mmRemainingNum = data[2] | data[3] << 8;
            refereeData_.bulletRemaininggStatus.remainingCoin       = data[4] | data[5] << 8;
            break;
        case RM_REFEREE_RFID:
            if (data.size() < 4) {
                ROS_ERROR("Referee System RFID packet length too short!");
                return;
            }
            status                              = data[0] | data[1] << 8 | data[2] << 16 | data[3] << 24;
            refereeData_.rfidStatus.base        = GET_BIT(status, 0);
            refereeData_.rfidStatus.elevated    = GET_BIT(status, 1);
            refereeData_.rfidStatus.powerRune   = GET_BIT(status, 2);
            refereeData_.rfidStatus.launchRamp  = GET_BIT(status, 3);
            refereeData_.rfidStatus.outpost     = GET_BIT(status, 4);
            refereeData_.rfidStatus.restoration = GET_BIT(status, 6);
            refereeData_.rfidStatus.engineer    = GET_BIT(status, 7);
            break;
        case RM_REFEREE_DART_CMD:
            if (data.size() < 12) {
                ROS_ERROR("Referee System Dart CMD packet length too short!");
                return;
            }
            refereeData_.dartClientCMD.status            = static_cast<robot_interface::RMRefereeHandle::DartOpeningStatus>(data[0]);
            refereeData_.dartClientCMD.target            = static_cast<robot_interface::RMRefereeHandle::DartTarget>(data[1]);
            refereeData_.dartClientCMD.targetChangeTime  = ros::Duration(data[2] | data[3] << 8);
            refereeData_.dartClientCMD.speed.first       = data[4];
            refereeData_.dartClientCMD.speed.second      = data[5];
            refereeData_.dartClientCMD.speed.third       = data[6];
            refereeData_.dartClientCMD.speed.fourth      = data[7];
            refereeData_.dartClientCMD.lastLaunchTime    = ros::Duration(data[8] | data[9] << 8);
            refereeData_.dartClientCMD.operateLaunchTime = ros::Duration(data[10] | data[11] << 8);
            break;
        case RM_REFEREE_INTERACTIVE:
            if (data.size() < 6) {
                ROS_ERROR("Referee System Interactive packet length too short!");
                return;
            }
            robotID = static_cast<int>(refereeData_.robotStatus.type);
            robotID += refereeData_.robotStatus.group == robot_interface::RMRefereeHandle::GameGroup::RED ? 0 : 100;
            /* 忽略不是发往自己的信息 */
            if ((data[4] | data[5] << 8) != robotID) return;
            interactiveData = std::vector<uint8_t>(data.begin() + 6, data.end());
            if (interactiveCallback_) interactiveCallback_(
                data[0] | data[1] << 8,
                data[2] | data[3] << 8,
                interactiveData);
            break;
        case RM_REFEREE_CUSTOM_CONTROLLER:
            if (customControllerCallback_) customControllerCallback_(data);
            break;
    }
}

void RMRefereeDriver::timeoutCallback()
{
    refereeData_.isOnline = false;
    ROS_ERROR("Referee System disconnected!");
}

}  // namespace robot_control