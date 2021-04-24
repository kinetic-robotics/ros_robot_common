#ifndef ROBOT_CONTROL_RM_REFEREE_H_
#define ROBOT_CONTROL_RM_REFEREE_H_

#include <ros/ros.h>

#include <robot_interface/rm_referee.h>

#include "robot_control/module.h"

/* 命令码 */
/* 比赛状态数据 */
#define RM_REFEREE_GAME_STATUS 0x0001
/* 比赛结果数据 */
#define RM_REFEREE_GAME_RESULT 0x0002
/* 比赛机器人血量数据 */
#define RM_REFEREE_HP 0x0003
/* 飞镖发射状态 */
#define RM_REFEREE_DART_LAUNCH 0x0004
/* ICRA数据 */
#define RM_REFEREE_ICRA 0x0005
/* 场地事件数据 */
#define RM_REFEREE_EVENT 0x0101
/* 场地补给站动作数据 */
#define RM_REFEREE_SUPPLY_ACTION 0x0102
/* 裁判警告数据 */
#define RM_REFEREE_REFEREE_WARNING 0x0104
/* 飞镖发射口倒计时数据 */
#define RM_REFEREE_DART_COUNTDOWN 0x0105
/* 机器人状态数据 */
#define RM_REFEREE_ROBOT_STATUS 0x0201
/* 实时热量功率数据 */
#define RM_REFEREE_POWER_HEAT 0x0202
/* 机器人位置数据 */
#define RM_REFEREE_POSITION 0x0203
/* 机器人增益数据 */
#define RM_REFEREE_BUFF 0x0204
/* 空中机器人能量数据 */
#define RM_REFEREE_AERIAL_POWER 0x0205
/* 伤害状态数据 */
#define RM_REFEREE_HURT 0x0206
/* 实时射击数据 */
#define RM_REFEREE_SHOOT 0x0207
/* 弹丸剩余发射数据 */
#define RM_REFEREE_BULLET_REMAINING 0x0208
/* RFID数据 */
#define RM_REFEREE_RFID 0x0209
/* 飞镖机器人客户端指令数据 */
#define RM_REFEREE_DART_CMD 0x020A
/* 机器人间交互数据 */
#define RM_REFEREE_INTERACTIVE 0x0301
/* 自定义控制器交互数据 */
#define RM_REFEREE_CUSTOM_CONTROLLER 0x0302
/* 小地图交互数据 */
#define RM_REFEREE_MAP_CMD 0x0303

/* 机器人间交互内容ID */
/* 删除图形 */
#define RM_REFEREE_INTERACTIVE_DELETE_GRAPH 0x0100
/* 绘制一个图形 */
#define RM_REFEREE_INTERACTIVE_UPDATE_1_GRAPH 0x0101
/* 绘制二个图形 */
#define RM_REFEREE_INTERACTIVE_UPDATE_2_GRAPH 0x0102
/* 绘制五个图形 */
#define RM_REFEREE_INTERACTIVE_UPDATE_5_GRAPH 0x0103
/* 绘制七个图形 */
#define RM_REFEREE_INTERACTIVE_UPDATE_7_GRAPH 0x0104
/* 绘制字符 */
#define RM_REFEREE_INTERACTIVE_UPDATE_STRING 0x0110

namespace robot_control
{
class RMRefereeDriver: public ModuleInterface
{
  private:
    ros::NodeHandle& node_;                                                                                       /* 节点 */
    ros::NodeHandle& nodeParam_;                                                                                  /* 参数节点 */
    std::string urdf_;                                                                                            /* URDF文件 */
    CommunicationDriver& driver_;                                                                                 /* 通信驱动 */
    hardware_interface::RobotHW& robotHW_;                                                                        /* RobotHW层 */
    robot_interface::RMRefereeInterface interface_;                                                               /* 裁判系统接口 */
    ros::Timer timeoutTimer_;                                                                                     /* 裁判系统信息超时计时器 */
    double timeout_;                                                                                              /* 裁判系统超时时间 */
    std::string handleName_;                                                                                      /* 裁判系统Handle名称 */
    int refereeSerialNum_;                                                                                        /* 裁判系统串口编号 */
    std::vector<uint8_t> serialRecvData_;                                                                         /* 串口收到的数据 */
    robot_interface::RMRefereeHandle::RefereeData refereeData_                                       = {0};       /* 裁判系统数据 */
    robot_interface::RMRefereeHandle::GameResultCallback gameResultCallback_                         = {nullptr}; /* 比赛结果数据回调 */
    robot_interface::RMRefereeHandle::DartLaunchCallback dartLaunchCallback_                         = {nullptr}; /* 飞镖发射回调 */
    robot_interface::RMRefereeHandle::SupplyProjectileActionCallback supplyProjectileActionCallback_ = {nullptr}; /* 补给站动作回调 */
    robot_interface::RMRefereeHandle::RefereeWarningCallback refereeWarningCallback_                 = {nullptr}; /* 裁判警告回调 */
    robot_interface::RMRefereeHandle::HurtCallback hurtCallback_                                     = {nullptr}; /* 伤害回调 */
    robot_interface::RMRefereeHandle::ShootCallback shootCallback_                                   = {nullptr}; /* 实时射击回调 */
    robot_interface::RMRefereeHandle::InteractiveCallback interactiveCallback_                       = {nullptr}; /* 机器人间交互数据回调 */
    robot_interface::RMRefereeHandle::CustomControllerCallback customControllerCallback_             = {nullptr}; /* 自定义控制器数据回调 */
    int lastRecvSeq_                                                                                 = -1;        /* 上一次接收到的包序号 */
    uint8_t lastSendSeq_                                                                             = 0;         /* 上一次发送到的包序号 */
    struct {
        std::vector<uint8_t> data; /* 完整包数据 */
        int cmdID;                 /* 命令码 */
        int seq;                   /* 包序号 */
        int targetLength;          /* 目标长度 */
        int crc16;                 /* CRC16 */
    } receivePacket_;              /* 正在接收的数据包数据 */
    enum class ReceiveMachineState {
        SOF      = 0,                                  /* 帧头 */
        LENGTH_1 = 1,                                  /* 长度第一字节 */
        LENGTH_2 = 2,                                  /* 长度第二字节 */
        SEQ      = 3,                                  /* 包序号 */
        CRC8     = 4,                                  /* 包头CRC8校验 */
        CMD_ID_1 = 5,                                  /* 命令码第一字节 */
        CMD_ID_2 = 6,                                  /* 命令码第二字节 */
        DATA     = 7,                                  /* 数据段 */
        CRC16_1  = 8,                                  /* 包尾CRC16校验第一字节 */
        CRC16_2  = 9                                   /* 包尾CRC16校验第二字节 */
    } receiveMachineState_ = ReceiveMachineState::SOF; /* 裁判系统接收解析状机 */

    /**
     * 串口接收回调
     * 
     * @param serialNum 
     * @param data 
     */
    void serialRXCallback(unsigned int serialNum, std::vector<uint8_t>& data);

    /**
     * 裁判系统信息超时回调
     * 
     */
    void timeoutCallback();

    /**
     * 裁判系统解析函数
     * 
     * @param cmdID 命令码
     * @param seq 包序号
     * @param data 数据
     */
    void parsedData(int cmdID, int seq, std::vector<uint8_t>& data);

    /**
     * 为机器人间交互信息添加头
     * 
     * @param cmdID 内容ID
     * @param recvID 接收者ID
     * @param data 输出数据
     */
    void addInteractiveHeader(int cmdID, int recvID, std::vector<uint8_t>& data);

    /**
     * 发送UI图形信息方法实现
     * 
     * @param data 数据
     */
    void sendGraphUIFunction(std::vector<robot_interface::RMRefereeHandle::UIData>& data);

    /**
     * 删除UI图层方法实现
     * 
     * @param cmd 指令
     * @param layer 图层,范围0-9
     */
    void deleteLayerUIFunction(robot_interface::RMRefereeHandle::GraphDeleteCMD cmd, int layer);

    /**
     * 发送机器人间交互数据方法实现
     * 
     * @param cmdID 内容ID,范围0x0200-0x02FF
     * @param type 目标机器人
     * @param data 数据
     */
    void sendInteractiveFunction(int cmdID, robot_interface::RMRefereeHandle::GameType type, std::vector<uint8_t>& data);



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
    RMRefereeDriver(ros::NodeHandle& node, ros::NodeHandle& nodeParam, std::string urdf, CommunicationDriver& driver, hardware_interface::RobotHW& robotHW);

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
    
    /**
     * 发送一帧数据给裁判系统
     * 
     * @param cmdID 命令码
     * @param data 数据
     */
    void sendFrame(uint16_t cmdID, std::vector<uint8_t>& data);
};
}  // namespace robot_control

#endif