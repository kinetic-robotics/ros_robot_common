# 比赛状态

Header header

# 比赛类型
# 没有比赛
uint8 TYPE_NONE = 0
# RoboMaster机甲大师赛
uint8 TYPE_RMUC = 1
# RoboMaster机甲大师单项赛
uint8 TYPE_RMUT = 2
# ICRA RoboMaster 人工智能挑战赛
uint8 TYPE_ICRA = 3
# RoboMaster 高校联盟赛 3V3
uint8 TYPE_RMUL3V3 = 4
# RoboMaster 高校联盟赛 1V1
uint8 TYPE_RMUL1V1 = 5
uint8 type

# 比赛进程
# 未开始比赛
uint8 PROCESS_NOT_START = 0
# 准备阶段
uint8 PROCESS_READY = 1
# 裁判系统自检阶段
uint8 PROCESS_CHECK = 2
# 5S倒计时
uint8 PROCESS_COUNTDOWN = 3
# 比赛中
uint8 PROCESS_BATTLING = 4
# 比赛结束,结算中
uint8 PROCESS_STOPPING = 5
uint8 process

# 当前阶段剩余时间
duration stageRemainTime
# 机器人接收到该指令的精确时间,当机载端收到有效的NTP服务器授时后生效
time syncTime