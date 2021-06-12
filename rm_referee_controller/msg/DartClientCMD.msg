# 飞镖机器人客户端指令数据

Header header

# 当前飞镖发射口的状态
# 关闭
uint8 STATUS_CLOSE = 0
# 正在打开或者关闭
uint8 STATUS_PREPARE = 1
# 开启
uint8 STATUS_OPEN = 2
uint8 status

# 飞镖打击目标
# 未知
uint8 TARGET_NONE = 0
# 前哨站
uint8 TARGET_OUTPOST = 1
# 基地
uint8 TARGET_BASE = 2
uint8 target

# 切换打击目标时的比赛剩余时间
duration targetChangeTime
# 飞镖速度
rm_referee_controller/DartSpeed speed
# 上一次发射飞镖的比赛剩余时间
duration lastLaunchTime
# 上一次操作手确定发射指令的比赛剩余时间
duration operateLaunchTime