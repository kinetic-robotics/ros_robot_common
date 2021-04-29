# 机器人状态数据

Header header

# 本台机器人所属方
# 红方
uint8 GROUP_RED = 1
# 蓝方
uint8 GROUP_BLUE = 2
uint8 group

# 本台机器人类型
# 英雄机器人
uint8 TYPE_HERO = 1
# 工程机器人
uint8 TYPE_ENGINEER = 2
# 步兵机器人3号
uint8 TYPE_STANDARD3 = 3
# 步兵机器人4号
uint8 TYPE_STANDARD4 = 4
# 步兵机器人5号
uint8 TYPE_STANDARD5 = 5
# 空中机器人
uint8 TYPE_AERIAL = 6
# 哨兵机器人
uint8 TYPE_SENTRY = 7
# 飞镖机器人
uint8 TYPE_DART = 8
# 雷达站
uint8 TYPE_RADER = 9
uint8 type

# 机器人等级
int32 level
# 当前血量
int32 hp
# 最大血量
int32 maxHP
# 射击机构数据
rm_referee_controller/RobotStatusShooter shooter
# 主控电源输出数据
rm_referee_controller/RobotStatusPower power
# 底盘功率限制,单位瓦
int32 chassisPowerLimit