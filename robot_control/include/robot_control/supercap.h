#include <ros/ros.h>

#include <robot_interface/supercap.h>
#include <robot_toolbox/function_tool.h>

#include "robot_control/module.h"

namespace robot_control
{
class SupercapDriver: public ModuleInterface
{
  private:
    ros::NodeHandle &node_;                                        /* 节点 */
    ros::NodeHandle &nodeParam_;                                   /* 参数节点 */
    std::string urdf_;                                             /* URDF文件 */
    CommunicationDriver &driver_;                                  /* 通信驱动 */
    hardware_interface::RobotHW &robotHW_;                         /* RobotHW层 */
    bool &isDisableOutput_;                                        /* 是否禁用输出 */
    robot_interface::SupercapInterface interface_;                 /* 超级电容接口 */
    int canNum_;                                                   /* CAN编号 */
    int recvCANID_;                                                /* 反馈报文CAN ID */
    int sendCANID_;                                                /* 控制报文CAN ID */
    ros::Duration lastSendDuration_;                               /* 上次发布控制报文的间隔 */
    double sendRate_;                                              /* 发布控制报文的频率 */
    std::unique_ptr<robot_toolbox::FunctionTool> percentFunction_; /* 电容电压-百分比函数 */
    std::string handleName_;                                       /* 超级电容Handle名称 */
    double percent_        = 0;                                    /* 剩余电量百分比 */
    double inputVoltage_   = 0;                                    /* 输入电压 */
    double inputCurrent_   = 0;                                    /* 输入电流 */
    double capVoltage_     = 0;                                    /* 电容电压 */
    double nowTargetPower_ = 0;                                    /* 实际目标功率 */
    double cmdTargetPower_ = 0;                                    /* 设定目标功率 */

    /**
     * CAN接收回调
     * 
     * @param canNum CAN编号
     * @param f      CAN帧率
     */
    void canRXCallback(unsigned int canNum, CANDriver::Frame &f);

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
    SupercapDriver(ros::NodeHandle &node, ros::NodeHandle &nodeParam, std::string urdf, CommunicationDriver &driver, hardware_interface::RobotHW &robotHW, bool &isDisableOutput);

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
    void read(const ros::Time &time, const ros::Duration &period);

    /**
     * ROS Control 需要的写入函数
     */
    void write(const ros::Time &time, const ros::Duration &period);
};
}  // namespace robot_control