#include <ros/ros.h>

#include <robot_interface/io.h>

#include "robot_control/module.h"

namespace robot_control
{
class IODriver: public ModuleInterface
{
  private:
    typedef struct {
        unsigned short canNum;   /* IO模块CAN编号 */
        unsigned short id;       /* IO模块ID */
        bool targetLevel;        /* 提供给ROS的目标电平接口 */
        bool currentLevel;       /* 提供给ROS的当前电平接口 */
        bool isOnline;           /* IO模块是否在线 */
        double timeout;          /* 接收超时时间 */
        ros::Timer timeoutTimer; /* 超时计时器 */
    } IOInfo;
    ros::NodeHandle &node_;                   /* 节点 */
    ros::NodeHandle &nodeParam_;              /* 参数节点 */
    std::string urdf_;                        /* URDF文件 */
    CommunicationDriver &driver_;             /* 通信驱动 */
    hardware_interface::RobotHW &robotHW_;    /* RobotHW层 */
    robot_interface::IOInterface interface_;  /* IO接口 */
    std::map<std::string, IOInfo> ios_;       /* IO信息数组 */
    std::map<int, IOInfo &> ioIDSearchTable_; /* IO的ID查找表 */
    ros::Duration lastSendDuration_;          /* 上次发布控制报文的间隔 */
    double sendRate_;                         /* 发布控制报文的频率 */

    /**
     * CAN接收回调
     * 
     * @param canNum CAN编号
     * @param f      CAN帧率
     */
    void canRXCallback(unsigned int canNum, CANDriver::Frame &f);

    /**
     * 超时回调
     * 
     * @param ioName IO名称
     */
    void timeoutCallback(std::string &ioName);

  public:
    /**
     * 构造函数
     * 
     * @param node 节点
     * @param nodeParam 参数节点
     * @param urdf URDF文件
     * @param driver 驱动
     * @param robotHW RobotHW层
     */
    IODriver(ros::NodeHandle &node, ros::NodeHandle &nodeParam, std::string urdf, CommunicationDriver &driver, hardware_interface::RobotHW &robotHW);

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