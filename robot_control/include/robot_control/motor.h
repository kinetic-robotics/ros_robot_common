#ifndef ROBOT_CONTROL_MOTOR_H_
#define ROBOT_CONTROL_MOTOR_H_

#include <ros/ros.h>

#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <urdf/model.h>

#include "robot_control/communication/communication.h"
#include "robot_control/module.h"

namespace robot_control
{
/* RM电机一圈编码器值 */
#define RM_MOTOR_TOTAL_ECD 8191.0f

class MotorDriver: public ModuleInterface
{
  private:
    enum class MotorType {
        RM3508,
        RM2006,
        RM6020,
        PWM,
        CKYF2305,
        SERVO
    };

    typedef struct {
        unsigned short canID;    /* 电机CAN报文ID */
        unsigned short canNum;   /* 电机CAN编号 */
        MotorType type;          /* 电机类型 */
        int round;               /* 电机转动的圈数 */
        double positionOffset;   /* 电机转子角度偏移量 */
        bool isReverse;          /* 是否反向电机位置和电流和速度等数据 */
        double position;         /* 提供给ROS的位置接口,这里是累计位置,即为电机当前转子角度+转过圈数-偏移 */
        double velocity;         /* 提供给ROS的速度接口 */
        double effort;           /* 提供给ROS的力矩接口 */
        double absolutePosition; /* 提供给ROS的位置接口,这里是电机当前转子角度-偏移 */
        double setEffort;        /* 提供给ROS的目标力矩接口,即电流 */
        double setVelocity;      /* 提供给ROS的目标速度接口,rad/s */
        double setPosition;      /* 提供给ROS的目标位置接口,rad */
        double lastRealPosition; /* 上次循环的电机角度,该角度为真实角度,不考虑取反,偏移等 */
        bool isOnline;           /* 电机是否在线 */
        double timeout;          /* 接收超时时间 */
        bool isFirstLoop;        /* 是否为该电机第一次接收数据,用于防止错误的累计圈数 */
        ros::Timer timeoutTimer; /* 超时计时器 */
    } MotorInfo;
    ros::NodeHandle& node_;                                                                    /* 节点 */
    ros::NodeHandle& nodeParam_;                                                               /* 参数节点 */
    std::string urdf_;                                                                         /* URDF文件 */
    CommunicationDriver& driver_;                                                              /* 通信驱动 */
    hardware_interface::RobotHW& robotHW_;                                                     /* RobotHW层 */
    std::map<std::string, MotorInfo> motors_;                                                  /* 电机信息数组 */
    transmission_interface::RobotTransmissions robotTransmissions_;                            /* 传动信息,从URDF解析 */
    hardware_interface::ActuatorStateInterface actuatorStateInterface_;                        /* 执行器状态接口 */
    hardware_interface::EffortActuatorInterface actuatorEffortInterface_;                      /* 执行器力矩命令接口 */
    hardware_interface::VelocityActuatorInterface actuatorVelocityInterface_;                  /* 执行器速度命令接口 */
    hardware_interface::PositionActuatorInterface actuatorPositionInterface_;                  /* 执行器位置命令接口 */
    std::shared_ptr<transmission_interface::TransmissionInterfaceLoader> transmissionsLoader_; /* 传动解析器 */
    joint_limits_interface::EffortJointSoftLimitsInterface effortLimits_;                      /* 力矩电机限制 */
    joint_limits_interface::VelocityJointSoftLimitsInterface velocityLimits_;                  /* 速度电机限制 */
    joint_limits_interface::PositionJointSoftLimitsInterface positionLimits_;                  /* 位置电机限制 */
    bool& isDisableOutput_;                                                                    /* 是否禁用输出 */

    /**
     * CAN接收回调
     * 
     * @param canNum CAN编号
     * @param f      CAN帧率
     */
    void canRXCallback(unsigned int canNum, CANDriver::Frame& f);

    /**
     * 超时回调
     * 
     * @param motorName 电机名称
     */
    void timeoutCallback(std::string& motorName);

  public:
    /**
     * 构造函数
     * 
     * @param node 节点
     * @param nodeParam 参数节点
     * @param urdf URDF文件
     * @param driver 驱动
     * @param robotHW RobotHW层
     * @param isDisableOutput 是否禁用输出
     */
    MotorDriver(ros::NodeHandle& node, ros::NodeHandle& nodeParam, std::string urdf, CommunicationDriver& driver, hardware_interface::RobotHW& robotHW, bool& isDisableOutput);

    /**
     * 初始化
     * @return 初始化是否成功
     */
    bool init();

    /**
     * 退出
     */
    void shutdown();

    /**
     * ROS Control 需要的读取函数
     */
    void read(const ros::Time& time, const ros::Duration& period);

    /**
     * ROS Control 需要的写入函数
     */
    void write(const ros::Time& time, const ros::Duration& period);
};

}  // namespace robot_control
#endif