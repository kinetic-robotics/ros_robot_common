#include <boost/bind.hpp>
#include <string>

#include <ros/ros.h>

#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <robot_toolbox/tool.h>
#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <urdf/model.h>

#include "robot_control/communication/communication.h"
#include "robot_control/motor.h"

namespace robot_control
{
MotorDriver::MotorDriver(ros::NodeHandle& node, ros::NodeHandle& nodeParam, std::string urdf, CommunicationDriver& driver, hardware_interface::RobotHW& robotHW)
    : ModuleInterface(node, nodeParam, urdf, driver, robotHW), node_(node), nodeParam_(nodeParam), urdf_(urdf), driver_(driver), robotHW_(robotHW)
{
}

bool MotorDriver::init()
{
    /* 解析配置注册执行器 */
    XmlRpc::XmlRpcValue motorList;
    nodeParam_.getParam("motor", motorList);
    CONFIG_ASSERT("motor", motorList.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (auto iter = motorList.begin(); iter != motorList.end(); iter++) {
        CONFIG_ASSERT("motor/" + iter->first, iter->second.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        memset(&motors_[iter->first], 0, sizeof(motors_[iter->first]));
        CONFIG_ASSERT("motor/" + iter->first + "/can_id", iter->second["can_id"].getType() == XmlRpc::XmlRpcValue::TypeInt && static_cast<int>(iter->second["can_id"]) > 0);
        motors_[iter->first].canID = static_cast<int>(iter->second["can_id"]);
        CONFIG_ASSERT("motor/" + iter->first + "/can_num", iter->second["can_num"].getType() == XmlRpc::XmlRpcValue::TypeInt && static_cast<int>(iter->second["can_num"]) >= 0);
        motors_[iter->first].canNum = static_cast<int>(iter->second["can_num"]);
        CONFIG_ASSERT("motor/" + iter->first + "/timeout", iter->second["timeout"].getType() == XmlRpc::XmlRpcValue::TypeDouble && static_cast<double>(iter->second["timeout"]) >= 0);
        motors_[iter->first].timeout = static_cast<double>(iter->second["timeout"]);
        CONFIG_ASSERT("motor/" + iter->first + "/reverse", iter->second["reverse"].getType() == XmlRpc::XmlRpcValue::TypeBoolean);
        motors_[iter->first].isReverse = static_cast<bool>(iter->second["reverse"]);
        CONFIG_ASSERT("motor/" + iter->first + "/position_offset", iter->second["position_offset"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        motors_[iter->first].positionOffset = static_cast<double>(iter->second["position_offset"]);
        CONFIG_ASSERT("motor/" + iter->first + "/type", iter->second["type"].getType() == XmlRpc::XmlRpcValue::TypeString);
        std::string type = iter->second["type"];
        if (type == "RM3508") {
            motors_[iter->first].type = MotorType::RM3508;
        } else if (type == "RM2006") {
            motors_[iter->first].type = MotorType::RM2006;
        } else if (type == "RM6020") {
            motors_[iter->first].type = MotorType::RM6020;
        } else if (type == "PWM") {
            motors_[iter->first].type = MotorType::PWM;
        } else {
            ROS_FATAL("Read parameter [motor/%s/type] failed! Motor type must in ['RM3508', 'RM6020', 'RM2006', 'PWM'] !", iter->first.c_str());
            return false;
        }
        motors_[iter->first].isFirstLoop  = true;
        motors_[iter->first].timeoutTimer = node_.createTimer(ros::Duration(motors_[iter->first].timeout), boost::bind(&MotorDriver::timeoutCallback, this, iter->first), true, true);
        actuatorStateInterface_.registerHandle(hardware_interface::ActuatorStateHandle(
            static_cast<std::string>(iter->first),
            &motors_[iter->first].position,
            &motors_[iter->first].velocity,
            &motors_[iter->first].effort,
            &motors_[iter->first].absolutePosition));
        actuatorEffortInterface_.registerHandle(hardware_interface::ActuatorHandle(
            actuatorStateInterface_.getHandle(iter->first),
            &motors_[iter->first].setEffort));
    }
    robotHW_.registerInterface(&actuatorStateInterface_);
    robotHW_.registerInterface(&actuatorEffortInterface_);
    /* 解析URDF文件并注册Joint(在下面这行调用) */
    transmissionsLoader_.reset(new transmission_interface::TransmissionInterfaceLoader(&robotHW_, &robotTransmissions_));
    transmissionsLoader_->load(urdf_);
    auto urdfModel = std::make_shared<urdf::Model>();
    urdfModel->initString(urdf_);
    /* 根据URDF文件注册电机限制 */
    for (auto iter = urdfModel->joints_.begin(); iter != urdfModel->joints_.end(); iter++) {
        if (iter->second->type != iter->second->FIXED) {
            joint_limits_interface::JointLimits limits;
            joint_limits_interface::SoftJointLimits softLimits;
            joint_limits_interface::getJointLimits(iter->second, limits);
            joint_limits_interface::getSoftJointLimits(iter->second, softLimits);
            limits_.registerHandle(joint_limits_interface::EffortJointSoftLimitsHandle(
                robotHW_.get<hardware_interface::EffortJointInterface>()->getHandle(iter->first),
                limits,
                softLimits));
        }
    }
    /* 注册所有CAN接收回调 */
    driver_.can->registerRxCallback(boost::bind(&MotorDriver::canRXCallback, this, _1, _2));
    return true;
}

void MotorDriver::shutdown()
{
}

void MotorDriver::timeoutCallback(std::string& motorName)
{
    ROS_ERROR("Motor %s is offline!", motorName.c_str());
    motors_[motorName].isOnline = false;
}

void MotorDriver::canRXCallback(unsigned int canNum, CANDriver::Frame& f)
{
    if (f.isRemoteTransmission || f.type != CANDriver::Frame::Type::STD) return;
    /* 查找ID对应的电机 */
    std::string motorName;
    for (auto iter = motors_.begin(); iter != motors_.end(); iter++) {
        if (iter->second.canID == f.id && iter->second.canNum == canNum) {
            motorName = iter->first;
            break;
        }
    }
    if (motorName.empty()) return;
    MotorInfo& motor = motors_[motorName];
    auto motorState  = robotHW_.get<hardware_interface::EffortActuatorInterface>()->getHandle(motorName);
    if (motor.type == MotorType::RM3508 || motor.type == MotorType::RM2006 || motor.type == MotorType::RM6020) {
        /* 解析电机参数 */
        double motorPosition = (f.data[0] << 8 | f.data[1]) / RM_MOTOR_TOTAL_ECD * 2 * M_PI;
        if (!motor.isFirstLoop) {
            if (motorPosition - motor.lastRealPosition > M_PI) {
                motor.round -= 1;
            } else if (motor.lastRealPosition - motorPosition > M_PI) {
                motor.round += 1;
            }
        }
        motor.lastRealPosition = motorPosition;
        motor.position         = motorPosition + motor.round * 2 * M_PI - motor.positionOffset;
        motor.absolutePosition = motorPosition - motor.positionOffset;
        motor.velocity         = (short)(f.data[2] << 8 | f.data[3]) / 60.0f * 2.0f * M_PI;
    }
    if (motor.type == MotorType::PWM) {
        motor.effort         = (short)(f.data[4] << 8 | f.data[5]) / 100.0f;
    }
    motor.isFirstLoop = false;
    /* 反向电机数据 */
    if (motor.isReverse) {
        motor.position         = -motor.position;
        motor.absolutePosition = -motor.absolutePosition;
        motor.velocity         = -motor.velocity;
    }
    if (!motor.isOnline) {
        ROS_INFO("Motor %s is online!", motorName.c_str());
    }
    motor.isOnline = true;
    motor.timeoutTimer.stop();
    motor.timeoutTimer.start();
}

void MotorDriver::read(const ros::Time& time, const ros::Duration& period)
{
    /* 这个函数不需要实现,因为实际上数据解析都在canRXCallback里了 */
    robotTransmissions_.get<transmission_interface::ActuatorToJointStateInterface>()->propagate();
}

void MotorDriver::write(const ros::Time& time, const ros::Duration& period)
{
    /* 电机限制 */
    limits_.enforceLimits(period);
    /* 转换传动命令 */
    robotTransmissions_.get<transmission_interface::JointToActuatorEffortInterface>()->propagate();
    /* 该MAP结构为:map[CAN编号][报文ID] = 具体报文 */
    std::map<unsigned int, std::map<unsigned int, CANDriver::Frame>> canFrames;
    for (auto iter = motors_.begin(); iter != motors_.end(); iter++) {
        if (iter->second.type == MotorType::RM3508 || iter->second.type == MotorType::RM2006 || iter->second.type == MotorType::RM6020) {
            /* 填充报文 */
            unsigned short canID;
            unsigned short motorID;
            if (iter->second.type == MotorType::RM3508 || iter->second.type == MotorType::RM2006) {
                /* 转换单位 */
                ABS_LIMIT(iter->second.setEffort, iter->second.type == MotorType::RM3508 ? 20 : 10);
                iter->second.setEffort *= iter->second.type == MotorType::RM3508 ? 16384 / 20 : 10000 / 10;
                /* 电调ID */
                motorID = iter->second.canID - 0x201;
                /* 报文ID */
                canID = motorID > 3 ? 0x1FF : 0x200;
            } else if (iter->second.type == MotorType::RM6020) {
                /* 转换单位 */
                ABS_LIMIT(iter->second.setEffort, 30);
                iter->second.setEffort *= 30000 / 30;
                /* 电调ID */
                motorID = iter->second.canID - 0x205;
                /* 报文ID */
                canID = motorID > 3 ? 0x2FF : 0x1FF;
            } else if (iter->second.type == MotorType::PWM) {
                /* 转换单位 */
                ABS_LIMIT(iter->second.setEffort, 5);
                iter->second.setEffort *= 500 / 5;
                /* 电调ID */
                motorID = iter->second.canID - 0x201;
                /* 报文ID */
                canID = motorID > 3 ? 0x1FF : 0x200;
            }
            if (canFrames[iter->second.canNum].find(canID) == canFrames[iter->second.canNum].end()) {
                canFrames[iter->second.canNum][canID].id                   = canID;
                canFrames[iter->second.canNum][canID].type                 = CANDriver::Frame::Type::STD;
                canFrames[iter->second.canNum][canID].isRemoteTransmission = false;
                canFrames[iter->second.canNum][canID].data.resize(8, 0);
            }
            short current = iter->second.isOnline ? iter->second.setEffort : 0;
            /* 反向电机数据 */
            if (iter->second.isReverse) current = -current;
            canFrames[iter->second.canNum][canID].data[(motorID % 4) * 2]     = current >> 8;
            canFrames[iter->second.canNum][canID].data[(motorID % 4) * 2 + 1] = current && 0xFF;
        }
    }
    /* 发送CAN报文 */
    for (auto iter = canFrames.begin(); iter != canFrames.end(); iter++) {
        for (auto iterFrame = iter->second.begin(); iterFrame != iter->second.end(); iterFrame++) {
            driver_.can->sendFrame(iter->first, iterFrame->second);
        }
    }
}

}  // namespace robot_control