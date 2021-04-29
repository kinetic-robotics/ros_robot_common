# 补给站动作

Header header

# 补给站口ID
int32 supplyID
# 是否有机器人在补弹
bool isAnyRobotSupply

# 补弹机器人类型,若没有机器人在补弹,则此项无意义
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
uint8 robotType

# 出弹口当前状态
# 补弹口关闭
uint8 STEP_CLOSE   = 0
# 子弹准备中
uint8 STEP_PREPARE = 1
# 子弹下落
uint8 STEP_OPEN    = 2
uint8 step

# 补弹数量
int32 projectileNumber