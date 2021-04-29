#ifndef ROBOT_INTERFACE_RM_REFEREE_H_
#define ROBOT_INTERFACE_RM_REFEREE_H_

#include <ros/ros.h>

#include <hardware_interface/internal/hardware_resource_manager.h>

namespace robot_interface
{
class RMRefereeHandle
{
  public:
    RMRefereeHandle(){};

    enum class GameType {
        NONE    = 0, /* 没有比赛 */
        RMUC    = 1, /* RoboMaster机甲大师赛 */
        RMUT    = 2, /* RoboMaster机甲大师单项赛 */
        ICRA    = 3, /* ICRA RoboMaster 人工智能挑战赛 */
        RMUL3V3 = 4, /* RoboMaster 高校联盟赛 3V3 */
        RMUL1V1 = 5, /* RoboMaster 高校联盟赛 1V1 */
    };               /* 比赛类型 */

    enum class GameProcess {
        NOT_START = 0, /* 未开始比赛 */
        READY     = 1, /* 准备阶段 */
        CHECK     = 2, /* 裁判系统自检阶段 */
        COUNTDOWN = 3, /* 5S倒计时 */
        BATTLING  = 4, /* 比赛中 */
        STOPPING  = 5, /* 比赛结束,结算中 */
    };                 /* 比赛进程状态 */

    enum class GameResult {
        DRAW = 0, /* 平局 */
        RED  = 1, /* 红方胜利 */
        BLUE = 2, /* 蓝方胜利 */
    };            /* 比赛结果 */

    enum class GameGroup {
        RED  = 1, /* 红方 */
        BLUE = 2, /* 蓝方 */
    };            /* 所属方 */

    enum class RobotType {
        HERO      = 1, /* 英雄机器人 */
        ENGINEER  = 2, /* 工程机器人 */
        STANDARD3 = 3, /* 步兵机器人3号 */
        STANDARD4 = 4, /* 步兵机器人4号 */
        STANDARD5 = 5, /* 步兵机器人5号 */
        AERIAL    = 6, /* 空中机器人 */
        SENTRY    = 7, /* 哨兵机器人 */
        DART      = 8, /* 飞镖机器人 */
        RADER     = 9  /* 雷达站 */
    };                 /* 机器人类型 */

    enum class SupplyProjectileStep {
        CLOSE   = 0, /* 补弹口关闭 */
        PREPARE = 1, /* 子弹准备中 */
        OPEN    = 2, /* 子弹下落 */
    };               /* 补弹状态 */

    enum class RefereeWarning {
        YELLOW = 1, /* 黄牌 */
        RED    = 2, /* 红牌 */
        DEFEAT = 3, /* 判负 */
    };              /* 裁判警告 */

    enum class HurtType {
        ARMOR_DAMAGE        = 0, /* 装甲伤害 */
        MODULE_OFFLINE      = 1, /* 模块掉线 */
        SHOOTER_SPEED_LIMIT = 2, /* 射击超射速 */
        SHOOTER_HEAT_LIMIT  = 3, /* 射击超热量 */
        CHASSIS_POWER_LIMIT = 4, /* 底盘超功率 */
        ARMOR_COLLISION     = 5, /* 底盘超功率 */
    };                           /* 伤害类型 */

    enum class BulletType {
        BULLET_17MM = 1, /* 17mm类型 */
        BULLET_42MM = 2, /* 42mm类型 */
    };                   /* 子弹类型 */

    enum class ShooterType {
        FIRST_17MM  = 1, /* 1号17mm发射机构 */
        SECOND_17MM = 2, /* 2号17mm发射机构 */
        FIRST_42MM  = 3, /* 42mm发射机构 */
    };                   /* 发射机构类型 */

    enum class DartOpeningStatus {
        CLOSE   = 0, /* 关闭 */
        PREPARE = 1, /* 正在打开或者关闭 */
        OPEN    = 2, /* 开启 */
    };               /* 飞镖发射口状态 */

    enum class DartTarget {
        NONE    = 0, /* 未知 */
        OUTPOST = 1, /* 前哨站 */
        BASE    = 2  /* 基地 */
    };               /* 飞镖打击目标 */

    enum class ICRABuffType {
        NONE             = 0, /* 未知 */
        RED_RESTORATION  = 1, /* 红方回血区 */
        RED_SUPPLY       = 2, /* 红方弹药补给区 */
        BLUE_RESTORATION = 3, /* 蓝方回血区 */
        BLUE_SUPPLY      = 4, /* 蓝方弹药补给区 */
        NO_SHOOTING      = 5, /* 禁止射击区 */
        NO_MOVING        = 6, /* 禁止移动区 */
    };                        /* ICRA Buff区类型 */

    enum class GraphMethod {
        EMPTY  = 0, /* 空操作 */
        ADD    = 1, /* 增加图形 */
        EDIT   = 2, /* 修改图形 */
        DELETE = 3, /* 删除图形 */
    };              /* 图形操作类型 */

    enum class GraphType {
        LINE         = 0, /* 直线 */
        RECTANGLE    = 1, /* 矩阵 */
        CIRCLE       = 2, /* 正圆 */
        OVAL         = 3, /* 椭圆 */
        ARC          = 4, /* 圆弧 */
        FLOAT_NUMBER = 5, /* 浮点数 */
        INT_NUMBER   = 6, /* 整数 */
        STRING       = 7, /* 字符 */
    };                    /* 图形类型 */

    enum class GraphColor {
        RED_BLUE  = 0, /* 红蓝主色 */
        YELLOW    = 1, /* 黄色 */
        GREEN     = 2, /* 绿色 */
        ORANGE    = 3, /* 橙色 */
        FUCHSIA   = 4, /* 紫红色 */
        PINK      = 5, /* 粉色 */
        CYAN_BLUE = 6, /* 青色 */
        BLACK     = 7, /* 黑色 */
        WHITE     = 8, /* 白色 */
    };                 /* 图形颜色 */

    enum class GraphDeleteCMD {
        LAYER = 1, /* 删除图层 */
        ALL = 2,   /* 删除所有 */
    };             /* 删除图形指令 */

    typedef struct {
        bool isOnline; /* 裁判系统是否在线 */
        struct {
            GameType type;                 /* 比赛类型 */
            GameProcess process;           /* 比赛进程状态 */
            ros::Duration stageRemainTime; /* 当前阶段剩余时间 */
            ros::Time syncTime;            /* 机器人接收到该指令的精确时间,当机载端收到有效的NTP服务器授时后生效 */
        } gameStatus;                      /* 比赛状态 */
        struct {
            struct {
                int hero1;     /* 英雄机器人1号 */
                int engineer2; /* 工程机器人2号 */
                int standard3; /* 步兵机器人3号 */
                int standard4; /* 步兵机器人4号 */
                int standard5; /* 步兵机器人5号 */
                int sentry7;   /* 哨兵机器人7号 */
                int outpost;   /* 前哨站 */
                int base;      /* 基地 */
            } red, blue;       /* 红蓝方机器人血量 */
        } hp;                  /* 血量状态 */
        struct {
            struct {
                struct {
                    bool isActive;        /* 是否激活 */
                    ICRABuffType type;    /* Buff区类型 */
                } f1, f2, f3, f4, f5, f6; /* F1-F6加成区信息 */
            } buff;                       /* Buff区状态 */
            struct {
                int red1, red2, blue1, blue2; /* 红蓝方1号2号 */
            } remainingBullet;                /* 剩余子弹数量 */
        } icraStatus;                         /* ICRA状态 */
        struct {
            struct {
                bool isFirstOccupied;  /* 1号补血点是否被占领 */
                bool isSecondOccupied; /* 2号补血点是否被占领 */
                bool isThirdOccupied;  /* 3号补血点是否被占领 */
            } restoration;             /* 补血点 */
            struct {
                bool isAttackPointOccupied; /* 击打点是否占领 */
                bool isSmallActive;         /* 小能量机关是否已经激活 */
                bool isBigActive;           /* 大能量机关是否已经激活 */
            } powerRune;                    /* 能量机关 */
            struct {
                bool is2Occupied;       /* 2号环形高地占领状态 */
                bool is3Occupied;       /* 3号梯形高地占领状态 */
                bool is4Occupied;       /* 4号梯形高地占领状态 */
            } elevated;                 /* 高地 */
            bool isVirtualShieldActive; /* 基地虚拟护盾是否还有血量 */
            bool isOutpostActive;       /* 前哨站是否存活 */
        } event;                        /* 场地事件 */
        ros::Duration dartRemainingTime; /* 飞镖发射口倒计时 */
        struct {
            GameGroup group; /* 本台机器人所属方 */
            RobotType type;  /* 本台机器人类型 */
            int level;       /* 机器人等级 */
            int hp;          /* 当前血量 */
            int maxHP;       /* 最大血量 */
            struct {
                struct {
                    int coolingRate;                /* 每秒冷却值 */
                    int coolingLimit;               /* 热量上限 */
                    int speedLimit;                 /* 上限速度,单位m/s */
                } first17mm, second17mm, first42mm; /* 1号,2号17mm和42mm枪口数据 */
            } shooter;                              /* 射击机构数据 */
            int chassisPowerLimit;                  /* 底盘功率限制,单位瓦 */
            struct {
                bool gimbal;  /* 云台 */
                bool chassis; /* 底盘 */
                bool shooter; /* 射击机构 */
            } power;          /* 主控电源输出数据 */
        } robotStatus;        /* 机器人状态数据 */
        struct {
            double voltage; /* 电压,单位伏 */
            double current; /* 电流,单位安 */
            double power;   /* 功率,单位瓦 */
            int buffer;     /* 功率缓冲,单位焦耳 */
            struct {
                int first17mm, second17mm, first42mm; /* 1号,2号17mm和42mm枪口 */
            } shooterHeat;                            /* 枪口热量数据 */
        } powerHeatData;                              /* 实时功率热量数据 */
        struct {
            double x, y, z, shooterYaw; /* XYZ轴和枪口YAW轴角度数据 */
        } position;              /* 机器人位置数据 */
        struct {
            bool isHealing;              /* 是否正在补血 */
            bool isShooterCoolingFaster; /* 是否枪口冷却加速 */
            bool isDefenseBuff;          /* 是否防御加成 */
            bool isAttackBuff;           /* 是否攻击加成 */
        } buff;                          /* 机器人Buff数据 */
        ros::Duration aerialAttackTime;  /* 空中机器人可攻击时间 */
        struct {
            int all17mmRemainingNum; /* 17mm子弹剩余发射数 */
            int all42mmRemainingNum; /* 17mm子弹剩余发射数 */
            int remainingCoin;       /* 剩余金币 */
        } bulletRemaininggStatus;    /* 子弹剩余数据 */
        struct {
            bool base;        /* 基地增益点 */
            bool elevated;    /* 高地增益点 */
            bool powerRune;   /* 能量机关激活点 */
            bool launchRamp;  /* 飞坡增益点 */
            bool outpost;     /* 前哨站增益点 */
            bool restoration; /* 补血点增益点 */
            bool engineer;    /* 工程机器人补血卡 */
        } rfidStatus;         /* RFID状态 */
        struct {
            DartOpeningStatus status;       /* 当前飞镖发射口的状态 */
            DartTarget target;              /* 飞镖打击目标 */
            ros::Duration targetChangeTime; /* 切换打击目标时的比赛剩余时间 */
            struct {
                int first, second, third, fourth; /* 四枚飞镖 */
            } speed;                              /* 飞镖速度 */
            ros::Duration lastLaunchTime;         /* 上一次发射飞镖的比赛剩余时间 */
            ros::Duration operateLaunchTime;      /* 上一次操作手确定发射指令的比赛剩余时间 */
        } dartClientCMD;                          /* 飞镖机器人客户端指令数据 */
    } RefereeData;                                /* 裁判系统数据 */

    typedef struct {
        uint32_t name;      /* 图形名称,范围为0-0xFFFFFF */
        GraphMethod method; /* 图形操作 */
        uint8_t layer;      /* 图层,范围0-9 */
        GraphColor color;   /* 图形颜色 */
        GraphType type;     /* 图形类型,请注意字符类型是特殊的不在本列表内 */
        struct {
            int width; /* 线条宽度 */
            struct {
                int x, y; /* X,Y坐标 */
            } start, end; /* 起点和终点坐标 */
        } line;           /* 直线 */
        struct {
            int width; /* 线条宽度 */
            struct {
                int x, y; /* X,Y坐标 */
            } start, end; /* 起点和对角终点坐标 */
        } rectangle;      /* 矩形 */
        struct {
            int width; /* 线条宽度 */
            struct {
                int x, y; /* X,Y坐标 */
            } center;     /* 圆形坐标 */
            int radius;   /* 半径 */
        } circle;         /* 圆形 */
        struct {
            int width; /* 线条宽度 */
            struct {
                int x, y; /* X,Y坐标 */
            } center;     /* 圆心坐标 */
            int xRadius;  /* X半轴长 */
            int yRadius;  /* Y半轴长 */
        } oval;           /* 椭圆 */
        struct {
            int width; /* 线条宽度 */
            struct {
                double start, end; /* 角度,单位弧度 */
            } angle;               /* 起始角度和终止角度 */
            struct {
                int x, y; /* X,Y坐标 */
            } center;     /* 圆心坐标 */
            int xRadius;  /* X半轴长 */
            int yRadius;  /* Y半轴长 */
        } arc;            /* 圆弧 */
        struct {
            int width; /* 线条宽度 */
            struct {
                int x, y; /* X,Y坐标 */
            } start;      /* 起点坐标 */
            double data;  /* 输出的数 */
            int fontSize; /* 字体大小 */
            int digits;   /* 小数点后位数 */
        } floatNumber;    /* 浮点数 */
        struct {
            int width; /* 线条宽度 */
            struct {
                int x, y; /* X,Y坐标 */
            } start;      /* 起点坐标 */
            int data;     /* 输出的数 */
            int fontSize; /* 字体大小 */
        } intNumber;      /* 整数 */
        struct {
            int width; /* 线条宽度 */
            struct {
                int x, y;     /* X,Y坐标 */
            } start;          /* 起点坐标 */
            std::string data; /* 输出的数 */
            int fontSize;     /* 字体大小 */
        } string;             /* 字符 */
    } UIData;                 /* UI数据 */

    /**
     * 比赛结果数据回调
     * 
     * @param result 比赛结果
     */
    using GameResultCallback = std::function<void(GameResult result)>;

    /**
     * 飞镖发射回调
     * 
     * @param group 发射队伍
     * @param remainingTime 发射时的剩余比赛时间
     * 
     */
    using DartLaunchCallback = std::function<void(GameGroup group, ros::Duration remainingTime)>;

    /**
     * 补给站动作回调,只会接收己方机器人数据
     * 
     * @param supplyID 补给站口ID
     * @param isAnyRobotSupply 是否有机器人在补弹
     * @param robotType 补弹机器人类型,若没有机器人在补弹,则此项无意义
     * @param step 出弹口当前状态
     * @param projectileNumber 补弹数量
     */
    using SupplyProjectileActionCallback = std::function<void(int supplyID, bool isAnyRobotSupply, RobotType robotType, SupplyProjectileStep step, int projectileNumber)>;

    /**
     * 裁判警告回调
     * 
     * @param level 警告等级
     * @param robotType 机器人类型
     */
    using RefereeWarningCallback = std::function<void(RefereeWarning level, RobotType robotType)>;

    /**
     * 伤害回调
     * 
     * @param armorID 装甲ID
     * @param type 伤害类型
     */
    using HurtCallback = std::function<void(int armorID, HurtType type)>;

    /**
     * 实时射击回调
     * 
     * @param bulletType 子弹类型
     * @param shooterID 发射机构类型
     * @param hz 射频,单位hz
     * @param speed 射速,单位m/s
     */
    using ShootCallback = std::function<void(BulletType bulletType, ShooterType shooterType, int hz, double speed)>;

    /**
     * 机器人间交互数据回调
     * 
     * @param cmdID 内容ID
     * @param senderID 发送者ID
     * @param data 数据
     */
    using InteractiveCallback = std::function<void(int cmdID, int senderID, std::vector<uint8_t>& data)>;

    /**
     * 自定义控制器数据回调
     * 
     * @param data 数据
     */
    using CustomControllerCallback = std::function<void(std::vector<uint8_t>& data)>;
    
    /**
     * 发送UI图形信息方法实现
     * 
     * @param data 数据
     */
    using SendGraphUIFunction = std::function<void(std::vector<UIData>& data)>;

    /**
     * 删除UI图层方法实现
     * 
     * @param cmd 指令
     * @param layer 图层,范围0-9
     */
    using DeleteLayerUIFunction = std::function<void(GraphDeleteCMD cmd, int layer)>;

    /**
     * 发送机器人间交互数据方法实现
     * 
     * @param cmdID 内容ID,范围0x0200-0x02FF
     * @param type 目标机器人
     * @param data 数据
     */
    using SendInteractiveFunction = std::function<void(int cmdID, robot_interface::RMRefereeHandle::GameType type, std::vector<uint8_t>& data)>;

    /**
     * 构造函数
     * 
     * @param name 裁判系统名称
     * @param data 裁判系统数据
     * @param gameResultCallback 比赛结果数据回调
     * @param dartLaunchCallback 飞镖发射回调
     * @param supplyProjectileActionCallback 补给站动作回调,只会接收己方机器人数据
     * @param refereeWarningCallback 裁判警告回调
     * @param hurtCallback 伤害回调
     * @param shootCallback 实时射击回调
     * @param interactiveCallback 机器人间交互数据回调
     * @param sendGraphUIFunction 发送UI图形信息方法实现
     * @param deleteLayerUIFunction 删除UI图层方法实现
     * @param sendInteractiveFunction 发送机器人间交互数据方法实现
     */
    RMRefereeHandle(const std::string& name, RefereeData* data, GameResultCallback* gameResultCallback, DartLaunchCallback* dartLaunchCallback, SupplyProjectileActionCallback* supplyProjectileActionCallback,
                    RefereeWarningCallback* refereeWarningCallback, HurtCallback* hurtCallback, ShootCallback* shootCallback, InteractiveCallback* interactiveCallback, CustomControllerCallback* customControllerCallback,
                    SendGraphUIFunction sendGraphUIFunction, DeleteLayerUIFunction deleteLayerUIFunction, SendInteractiveFunction sendInteractiveFunction)
        : name_(name), data_(data), gameResultCallback_(gameResultCallback), dartLaunchCallback_(dartLaunchCallback), supplyProjectileActionCallback_(supplyProjectileActionCallback),
          refereeWarningCallback_(refereeWarningCallback), hurtCallback_(hurtCallback), shootCallback_(shootCallback), interactiveCallback_(interactiveCallback), customControllerCallback_(customControllerCallback),
          callSendGraphUIFunction(sendGraphUIFunction), callDeleteLayerUIFunction(deleteLayerUIFunction), callSendInteractiveFunction(sendInteractiveFunction) {};

    std::string getName() const { return name_; }
    RefereeData getData() const { ROS_ASSERT(data_); return *data_; }
    const RefereeData* getDataPtr() const { return data_; }

    /**
     * 设置比赛结果数据回调
     * 
     * @param callback 回调
     */
    void setGameResultCallback(GameResultCallback callback) const { ROS_ASSERT(gameResultCallback_); *gameResultCallback_ = callback; }

    /**
     * 设置飞镖发射回调
     * 
     * @param callback 回调
     */
    void setDartLaunchCallback(DartLaunchCallback callback) const { ROS_ASSERT(dartLaunchCallback_); *dartLaunchCallback_ = callback; }

    /**
     * 设置补给站动作回调
     * 
     * @param callback 回调
     */
    void setSupplyProjectileActionCallback(SupplyProjectileActionCallback callback) const { ROS_ASSERT(supplyProjectileActionCallback_); *supplyProjectileActionCallback_ = callback; }

    /**
     * 设置裁判警告回调
     * 
     * @param callback 回调
     */
    void setRefereeWarningCallback(RefereeWarningCallback callback) const { ROS_ASSERT(refereeWarningCallback_); *refereeWarningCallback_ = callback; }

    /**
     * 设置伤害回调
     * 
     * @param callback 回调
     */
    void setHurtCallback(HurtCallback callback) const { ROS_ASSERT(hurtCallback_); *hurtCallback_ = callback; }

    /**
     * 设置实时射击回调
     * 
     * @param callback 回调
     */
    void setShootCallback(ShootCallback callback) const { ROS_ASSERT(shootCallback_); *shootCallback_ = callback; }

    /**
     * 设置机器人间交互数据回调
     * 
     * @param callback 回调
     */
    void setInteractiveCallback(InteractiveCallback callback) const { ROS_ASSERT(interactiveCallback_); *interactiveCallback_ = callback; }

    /**
     * 设置自定义控制器数据回调
     * 
     * @param callback 回调
     */
    void setCustomControllerCallback(CustomControllerCallback callback) const { ROS_ASSERT(customControllerCallback_); *customControllerCallback_ = callback; }

    /**
     * 发送UI图形信息
     * 
     * @param data 数据
     */
    SendGraphUIFunction callSendGraphUIFunction;

    /**
     * 删除UI图层方法实现
     * 
     * @param cmd 指令
     * @param layer 图层,范围0-9
     */
    DeleteLayerUIFunction callDeleteLayerUIFunction;

    /**
     * 发送机器人间交互数据方法实现
     * 
     * @param data 数据
     */
    SendInteractiveFunction callSendInteractiveFunction;


  private:
    std::string name_;
    const RefereeData* data_                                        = {nullptr};
    GameResultCallback* gameResultCallback_                         = {nullptr};
    DartLaunchCallback* dartLaunchCallback_                         = {nullptr};
    SupplyProjectileActionCallback* supplyProjectileActionCallback_ = {nullptr};
    RefereeWarningCallback* refereeWarningCallback_                 = {nullptr};
    HurtCallback* hurtCallback_                                     = {nullptr};
    ShootCallback* shootCallback_                                   = {nullptr};
    InteractiveCallback* interactiveCallback_                       = {nullptr};
    CustomControllerCallback* customControllerCallback_             = {nullptr};
};

class RMRefereeInterface: public hardware_interface::HardwareResourceManager<RMRefereeHandle> {};

}  // namespace robot_interface

#endif