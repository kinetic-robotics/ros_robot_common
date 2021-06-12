#ifndef RM_REFEREE_CONTROLLER_RM_REFEREE_CONTROLLER_H_
#define RM_REFEREE_CONTROLLER_RM_REFEREE_CONTROLLER_H_

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <robot_interface/rm_referee.h>
#include <robot_msgs/BoolStamped.h>
#include <robot_msgs/DurationStamped.h>

#include "rm_referee_controller/Buff.h"
#include "rm_referee_controller/BulletRemainingStatus.h"
#include "rm_referee_controller/CustomController.h"
#include "rm_referee_controller/DartClientCMD.h"
#include "rm_referee_controller/DartLaunch.h"
#include "rm_referee_controller/DeleteUI.h"
#include "rm_referee_controller/Event.h"
#include "rm_referee_controller/GameResult.h"
#include "rm_referee_controller/GameStatus.h"
#include "rm_referee_controller/HP.h"
#include "rm_referee_controller/Hurt.h"
#include "rm_referee_controller/ICRAStatus.h"
#include "rm_referee_controller/Interactive.h"
#include "rm_referee_controller/Position.h"
#include "rm_referee_controller/PowerHeat.h"
#include "rm_referee_controller/RFIDStatus.h"
#include "rm_referee_controller/RefereeWarning.h"
#include "rm_referee_controller/RobotStatus.h"
#include "rm_referee_controller/Shoot.h"
#include "rm_referee_controller/SupplyProjectileAction.h"
#include "rm_referee_controller/UIData.h"

namespace rm_referee_controller
{
class RMRefereeController: public controller_interface::Controller<robot_interface::RMRefereeInterface>
{
  private:
    typedef struct {
        bool forceUpdate;                                                                                        /* 是否强制更新 */
        bool isFirstSend;                                                                                        /* 是否为第一次发送 */
        robot_interface::RMRefereeHandle::UIData property;                                                       /* 新数据 */
        robot_interface::RMRefereeHandle::UIData oldProperty;                                                    /* 旧数据 */
    } UIInfo;                                                                                                    /* UI数组数据 */
    ros::Duration lastPublishDuration_;                                                                          /* 上次发布裁判系统信息的间隔 */
    double publishRate_;                                                                                         /* 发布信息的频率 */
    robot_interface::RMRefereeHandle handle_;                                                                    /* 句柄 */
    std::unique_ptr<realtime_tools::RealtimePublisher<robot_msgs::BoolStamped>> isOnlinePublisher_;              /* 裁判系统是否在线发布者 */
    std::unique_ptr<realtime_tools::RealtimePublisher<GameStatus>> gameStatusPublisher_;                         /* 比赛状态发布者 */
    std::unique_ptr<realtime_tools::RealtimePublisher<HP>> hpPublisher_;                                         /* 血量状态发布者 */
    std::unique_ptr<realtime_tools::RealtimePublisher<ICRAStatus>> icraStatusPublisher_;                         /* ICRA状态发布者 */
    std::unique_ptr<realtime_tools::RealtimePublisher<Event>> eventPublisher_;                                   /* 场地事件发布者 */
    std::unique_ptr<realtime_tools::RealtimePublisher<robot_msgs::DurationStamped>> dartRemainingTimePublisher_; /* 飞镖发射口倒计时发布者 */
    std::unique_ptr<realtime_tools::RealtimePublisher<RobotStatus>> robotStatusPublisher_;                       /* 机器人状态数据发布者 */
    std::unique_ptr<realtime_tools::RealtimePublisher<PowerHeat>> powerHeatDataPublisher_;                       /* 实时功率热量数据发布者 */
    std::unique_ptr<realtime_tools::RealtimePublisher<Position>> positionPublisher_;                             /* 机器人位置数据发布者 */
    std::unique_ptr<realtime_tools::RealtimePublisher<Buff>> buffPublisher_;                                     /* 机器人Buff数据发布者 */
    std::unique_ptr<realtime_tools::RealtimePublisher<robot_msgs::DurationStamped>> aerialAttackTimePublisher_;  /* 空中机器人可攻击时间发布者 */
    std::unique_ptr<realtime_tools::RealtimePublisher<BulletRemainingStatus>> bulletRemainingStatusPublisher_;   /* 子弹剩余数据发布者 */
    std::unique_ptr<realtime_tools::RealtimePublisher<RFIDStatus>> rfidStatusPublisher_;                         /* RFID状态发布者 */
    std::unique_ptr<realtime_tools::RealtimePublisher<DartClientCMD>> dartClientCMDPublisher_;                   /* 飞镖机器人客户端指令数据发布者 */
    std::unique_ptr<realtime_tools::RealtimePublisher<GameResult>> gameResultPublisher_;                         /* 比赛结果数据发布者 */
    std::unique_ptr<realtime_tools::RealtimePublisher<DartLaunch>> dartLaunchPublisher_;                         /* 飞镖发射数据发布者 */
    std::unique_ptr<realtime_tools::RealtimePublisher<SupplyProjectileAction>> supplyProjectileActionPublisher_; /* 补给站动作数据发布者 */
    std::unique_ptr<realtime_tools::RealtimePublisher<RefereeWarning>> refereeWarningPublisher_;                 /* 裁判警告数据发布者 */
    std::unique_ptr<realtime_tools::RealtimePublisher<Hurt>> hurtPublisher_;                                     /* 伤害数据发布者 */
    std::unique_ptr<realtime_tools::RealtimePublisher<Shoot>> shootPublisher_;                                   /* 实时射击数据发布者 */
    std::unique_ptr<realtime_tools::RealtimePublisher<Interactive>> interactivePublisher_;                       /* 机器人间交互数据发布者 */
    std::unique_ptr<realtime_tools::RealtimePublisher<CustomController>> customControllerPublisher_;             /* 自定义控制器数据发布者 */
    ros::Subscriber interactiveSubscriber_;                                                                      /* 机器人间交互信息订阅者 */
    ros::Subscriber deleteUISubscriber_;                                                                         /* 删除UI信息订阅者 */
    ros::Subscriber updateUISubscriber_;                                                                         /* 更新UI信息订阅者 */
    std::array<std::map<int, UIInfo>, 9> uiWidgets_;                                                             /* UI组件们 */

    /**
     * ROS更新UI回调
     * 
     * @param result 数据
     */
    void updateUICallback(const UIDataConstPtr &msg);

    /**
     * ROS删除UI回调
     * 
     * @param result 数据
     */
    void deleteUICallback(const DeleteUIConstPtr &msg);

    /**
     * ROS机器人间交互回调
     * 
     * @param result 数据
     */
    void interactiveCallback(const InteractiveConstPtr &msg);

    /**
     * 比赛结果数据回调
     * 
     * @param result 比赛结果
     */
    void gameResultCallback(robot_interface::RMRefereeHandle::GameResult result);

    /**
     * 飞镖发射回调
     * 
     * @param group 发射队伍
     * @param remainingTime 发射时的剩余比赛时间
     * 
     */
    void dartLaunchCallback(robot_interface::RMRefereeHandle::GameGroup group, ros::Duration remainingTime);

    /**
     * 补给站动作回调,只会接收己方机器人数据
     * 
     * @param supplyID 补给站口ID
     * @param isAnyRobotSupply 是否有机器人在补弹
     * @param robotType 补弹机器人类型,若没有机器人在补弹,则此项无意义
     * @param step 出弹口当前状态
     * @param projectileNumber 补弹数量
     */
    void supplyProjectileActionCallback(int supplyID, bool isAnyRobotSupply, robot_interface::RMRefereeHandle::RobotType robotType, robot_interface::RMRefereeHandle::SupplyProjectileStep step, int projectileNumber);

    /**
     * 裁判警告回调
     * 
     * @param level 警告等级
     * @param robotType 机器人类型
     */
    void refereeWarningCallback(robot_interface::RMRefereeHandle::RefereeWarning level, robot_interface::RMRefereeHandle::RobotType robotType);

    /**
     * 伤害回调
     * 
     * @param armorID 装甲ID
     * @param type 伤害类型
     */
    void hurtCallback(int armorID, robot_interface::RMRefereeHandle::HurtType type);

    /**
     * 实时射击回调
     * 
     * @param bulletType 子弹类型
     * @param shooterID 发射机构类型
     * @param hz 射频,单位hz
     * @param speed 射速,单位m/s
     */
    void shootCallback(robot_interface::RMRefereeHandle::BulletType bulletType, robot_interface::RMRefereeHandle::ShooterType shooterType, int hz, double speed);

    /**
     * 机器人间交互数据回调
     * 
     * @param cmdID 内容ID
     * @param senderID 发送者ID
     * @param data 数据
     */
    void interactiveCallback(int cmdID, int senderID, std::vector<uint8_t> &data);

    /**
     * 自定义控制器数据回调
     * 
     * @param data 数据
     */
    void customControllerCallback(std::vector<uint8_t> &data);

  public:
    RMRefereeController();

    /**
     * ROS Control的Controller Init接口
     */
    bool init(robot_interface::RMRefereeInterface *hw, ros::NodeHandle &node);

    /**
     * ROS Control的Controller Update接口
     */
    void update(const ros::Time &time, const ros::Duration &period);

    /**
     * ROS Control的Controller Starting接口
     */
    void starting(const ros::Time &time);

    /**
     * ROS Control的Controller Stopping接口
     */
    void stopping(const ros::Time &time);
};

}  // namespace rm_referee_controller

#endif