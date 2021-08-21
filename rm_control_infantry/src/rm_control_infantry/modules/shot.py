from ..module import ModuleBase
from robot_toolbox.function_tool import FunctionTool
from robot_toolbox.tools import ConfigAssert, CheckValidKey
from rm_referee_controller.msg import PowerHeat, RobotStatus, GameStatus
from enum import Enum
from robot_msgs.msg import EmptyStamped, BoolStamped, Float64Stamped
import rospy

class ShotStatus(Enum):
    NONE = 0
    CONTINOUS = 1
    CONTINOUS_STOP = 2
    ONCE = 3

class ShotModule(ModuleBase):
    def __init__(self):
        super().__init__()
        self.__shotMsg, self.__frictionSpeedMsg = EmptyStamped(), Float64Stamped()
        self.__lastShotStatus, self.__frictionTargetSpeed = ShotStatus.NONE, 0
        self.__isFrictionEnable, self.__shotContinousBecauseJoy, self.__shotContinousBecauseMouse, self.__isNowContinousMode = False, False, False, False
        self.__nowMaxHeat, self.__nowHeat = 0, 0
        self.__isRefereeSystemOnline, self.__lastShouldStartFriction, self.__lastFrictionToggleSwitch = False, False, False

    def init(self):
        self.__robotStatusSubscriber = rospy.Subscriber(rospy.get_param("~shot/topic/robot_status"), RobotStatus, self.robotStatusCallback, queue_size=1000)
        self.__heatSubscriber = rospy.Subscriber(rospy.get_param("~shot/topic/heat"), PowerHeat, self.heatCallback, queue_size=1000)
        self.__onlineSubscriber = rospy.Subscriber(rospy.get_param("~shot/topic/referee_system_online"), BoolStamped, self.onlineCallback, queue_size=1000)
        self.__gameStatusSubscriber = rospy.Subscriber(rospy.get_param("~shot/topic/game_status"), GameStatus, self.gameStatusCallback, queue_size=1000)
        self.__shotContinousStartPublisher = rospy.Publisher(rospy.get_param("~shot/topic/continous_start"), EmptyStamped, queue_size=1000)
        self.__shotContinousStopPublisher = rospy.Publisher(rospy.get_param("~shot/topic/continous_stop"), EmptyStamped, queue_size=1000)
        self.__shotOncePublisher = rospy.Publisher(rospy.get_param("~shot/topic/once"), EmptyStamped, queue_size=1000)
        self.__frictionSpeedPublisher = rospy.Publisher(rospy.get_param("~shot/topic/friction_speed"), Float64Stamped, queue_size=1000)
        self.__shotContinousThresholdHeat = rospy.get_param("~shot/heat_limit/continous_threshold", None)
        self.__shotOnceThresholdHeat = rospy.get_param("~shot/heat_limit/once_threshold", None)
        self.__speedUpKey = rospy.get_param("~shot/keyboard/speed_up", None)
        self.__isAutoControlFriction = rospy.get_param("~shot/auto_control_friction", None)
        self.__shooterType = rospy.get_param("~shot/shooter", None)
        self.__shotContinousCheckTime = rospy.get_param("~shot/mouse/continous_check_time", None)
        if not ConfigAssert("shot/heat_limit/continous_threshold", isinstance(self.__shotContinousThresholdHeat, int)) or \
           not ConfigAssert("shot/heat_limit/once_threshold", isinstance(self.__shotOnceThresholdHeat, int)) or \
           not ConfigAssert("shot/speed_up_key", CheckValidKey(self.__speedUpKey)) or \
           not ConfigAssert("shot/shooter", self.__shooterType in ["first17mm", "second17mm", "first42mm"]) or \
           not ConfigAssert("shot/mouse/continous_check_time", isinstance(self.__shotContinousCheckTime, (int, float))) or \
           not ConfigAssert("shot/auto_control_friction", isinstance(self.__isAutoControlFriction, bool)):
           return False
        return True

    def gameStatusCallback(self, msg):
        """
        裁判系统比赛状态信息接收回调
        
        @param msg 消息
        """
        if not self.__isAutoControlFriction:
            return
        shouldStartFriction = msg.process != GameStatus.PROCESS_NOT_START
        if self.__lastShouldStartFriction != shouldStartFriction:
            if shouldStartFriction:
                rospy.loginfo("Game started, now start friction.")
                self.__isFrictionEnable = True
            else:
                rospy.loginfo("Game stopped, now stop friction.")
                self.__isFrictionEnable = False
        self.__lastShouldStartFriction = shouldStartFriction

    def robotStatusCallback(self, msg):
        """
        裁判系统机器人信息接收回调
        
        @param msg 消息
        """
        self.__frictionTargetSpeed = getattr(msg.shooter, self.__shooterType).speedLimit
        self.__nowMaxHeat = getattr(msg.shooter, self.__shooterType).coolingLimit
    
    def heatCallback(self, msg):
        """
        裁判系统热量接收回调
        
        @param msg 消息
        """
        self.__nowHeat = getattr(msg.shooterHeat, self.__shooterType)

    def onlineCallback(self, msg):
        """
        裁判系统是否在线接收回调
        
        @param msg 消息
        """
        self.__isRefereeSystemOnline = msg.result


    def handleRCAndMouseMessage(self, rc, mouse):
        """
        将用户的输入(如摇杆,鼠标)转换为对应的射击指令
        
        @param rc 遥控器信息
        @param mouse 鼠标信息
        
        @return 射击状态
        """
        shotStatus = ShotStatus.NONE
        # 摇杆逻辑
        # 摩擦轮开关
        frictionToggleSwitchState = rc.buttons[0]
        if self.__lastFrictionToggleSwitch != frictionToggleSwitchState and frictionToggleSwitchState == 0:
            self.__isFrictionEnable = not self.__isFrictionEnable
        self.__lastFrictionToggleSwitch = frictionToggleSwitchState
        # 连发开关
        if rc.buttons[0] == -1:
            self.__shotContinousBecauseJoy = True
            shotStatus = ShotStatus.CONTINOUS
        elif self.__shotContinousBecauseJoy:
            shotStatus = ShotStatus.CONTINOUS_STOP
            self.__shotContinousBecauseJoy = False
        # 鼠标发射逻辑
        if mouse.leftButton:
            if self.__isLastLeftButtonPress:
                if (rospy.Time.now() - self.__lastLeftButtonPressTime) >= self.__shotContinousCheckTime:
                    shotStatus_ = ShotStatus.CONTINOUS
                    self.__shotContinousBecauseMouse = True
            else:
                self.__isLastLeftButtonPress = True
                self.__lastLeftButtonPressTime = rospy.Time.now()
                shotStatus_ = ShotStatus.ONCE
        else:
            isLastLeftButtonPress_ = False
            if self.__shotContinousBecauseMouse:
                self.__shotContinousBecauseMouse = False
                shotStatus_ = ShotStatus.CONTINOUS_STOP
        return shotStatus

    def heatLimit(self, rc, keyboard, shotStatus):
        """
        热量限制
        
        @param rc 遥控器信息
        @param keyboard 键盘信息
        
        @return 射击状态
        """
        if shotStatus == ShotStatus.CONTINOUS:
            self.__isNowContinousMode = True
        if shotStatus == ShotStatus.CONTINOUS_STOP or shotStatus == ShotStatus.ONCE:
            self.__isNowContinousMode = False
        # 无视热量闭环按键
        isEnable = getattr(keyboard, self.__speedUpKey)
        if isEnable and self.__isRefereeSystemOnline:
            if shotStatus == ShotStatus.ONCE and self.__nowMaxHeat - self.__nowHeat < self.__shotOnceThresholdHeat:
                shotStatus = ShotStatus.NONE
            if self.__isNowContinousMode and self.__nowMaxHeat - self.__nowHeat < self.__shotContinousThresholdHeat:
                shotStatus = ShotStatus.CONTINOUS_STOP
        return shotStatus
    
    def run(self, main, rc, mouse, keyboard, joints):
        shotStatus = self.handleRCAndMouseMessage(rc, mouse)
        shotStatus = self.heatLimit(rc, keyboard, shotStatus)
        # 发布射击信息
        self.__shotMsg.header.seq += 1
        self.__shotMsg.header.stamp = rospy.Time.now()
        if self.__lastShotStatus != shotStatus:
            if shotStatus == ShotStatus.CONTINOUS:
                self.__shotContinousStartPublisher.publish(self.__shotMsg)
            elif shotStatus == ShotStatus.CONTINOUS_STOP:
                self.__shotContinousStopPublisher.publish(self.__shotMsg)
            elif shotStatus == ShotStatus.ONCE:
                self.__shotOncePublisher.publish(self.__shotMsg)
        self.__lastShotStatus = shotStatus
        # 发布摩擦轮转速
        self.__frictionSpeedMsg.result = self.__frictionTargetSpeed if self.__isFrictionEnable else 0
        self.__frictionSpeedMsg.header.seq += 1
        self.__frictionSpeedMsg.header.stamp = rospy.Time.now()
        self.__frictionSpeedPublisher.publish(self.__frictionSpeedMsg)