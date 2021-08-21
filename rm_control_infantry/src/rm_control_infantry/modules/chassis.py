from ..module import ModuleBase
from robot_toolbox.function_tool import FunctionTool
from robot_toolbox.tools import ConfigAssert, CheckValidKey
from geometry_msgs.msg import Twist
from enum import Enum
import rospy, math

class KeyState(Enum):
    FORWARD = 1
    BACKWARD = -1
    NONE = 0

class ChassisModule(ModuleBase):
    def __init__(self):
        super().__init__()
        self.__pressVxButtonTime, self.__pressVyButtonTime, self.__pressVrzButtonTime = rospy.Time.now(), rospy.Time.now(), rospy.Time.now()
        self.__vxKeyState, self.__vyKeyState, self.__lastVrzKeyState = KeyState.NONE, KeyState.NONE, False
        self.isVrzEnable = False
        self.__yawPosition = 0


    def init(self):
        self.__yawJointName = rospy.get_param("~chassis/joint/yaw", None)
        self.__twistPublisher = rospy.Publisher(rospy.get_param("~chassis/topic/twist"), Twist, queue_size=1000)
        self.__vrzKey = rospy.get_param("~chassis/keyboard/vrz/key")
        self.__chassisFollowGimbalP = rospy.get_param("~chassis/chassis_follow_gimbal/p")
        if not ConfigAssert("~chassis/joint/yaw", isinstance(self.__yawJointName, str)) or not ConfigAssert("chassis/keyboard/vrz", CheckValidKey(self.__vrzKey)) or \
           not ConfigAssert("chassis/chassis_follow_gimbal/p", isinstance(self.__chassisFollowGimbalP, (int, float))):
            return False
        self.__keyVxFunction = FunctionTool("~chassis/keyboard/vx/function")
        self.__keyVyFunction = FunctionTool("~chassis/keyboard/vy/function")
        self.__keyVrzFunction = FunctionTool("~chassis/keyboard/vrz/function")
        self.__joyVxFunction = FunctionTool("~chassis/joystick/vx/function")
        self.__joyVyFunction = FunctionTool("~chassis/joystick/vy/function")
        self.__joyVrzFunction = FunctionTool("~chassis/joystick/vrz/function")
        if not self.__keyVxFunction.init() or not self.__keyVyFunction.init() or not self.__keyVrzFunction.init() or \
           not self.__joyVxFunction.init() or not self.__joyVyFunction.init() or not self.__joyVrzFunction.init():
            return False
        return True

    def handleRCAndKeyboardMessage(self, rc, keyboard):
        """
        将用户的输入(如摇杆,WASD按键)转换为对应的速度指令
        
        @param rc 遥控器信息
        @param keyboard 键盘信息
        
        @return vx, vy, vrz
        """
        # 摇杆逻辑
        vx = self.__joyVxFunction.compute(rc.axes[1])
        vy = self.__joyVxFunction.compute(rc.axes[0])
        vrz = self.__joyVxFunction.compute(rc.axes[4])
        # 键盘逻辑
        isVxForwardPress, isVxBackwardPress, isVyForwardPress, isVyBackwardPress = getattr(keyboard, "W"), getattr(keyboard, "A"), getattr(keyboard, "S"), getattr(keyboard, "D")
        # X轴按键逻辑处理,同时按下或者全部松开刹车
        if not (isVxForwardPress ^ isVxBackwardPress):
            self.__vxKeyState = KeyState.NONE
        elif isVxForwardPress or isVxBackwardPress:
            keyState = KeyState.FORWARD if isVxForwardPress else KeyState.BACKWARD
            if keyState != self.__vxKeyState:
                self.__pressVxButtonTime = rospy.Time.now()
            self.__vxKeyState = keyState
            if isVxForwardPress:
                vx += self.__keyVxFunction.compute((rospy.Time.now() - self.__pressVxButtonTime).to_sec())
            else:
                vx += self.self.__keyVxFunction.compute(-(rospy.Time.now() - self.__pressVxButtonTime).to_sec())
        # Y轴按键逻辑处理,同时按下或者全部松开刹车
        if not (isVyForwardPress ^ isVyBackwardPress):
            self.__vyKeyState = KeyState.NONE
        elif isVyForwardPress or isVyBackwardPress:
            keyState = KeyState.FORWARD if isVyForwardPress else KeyState.BACKWARD
            if keyState != self.__vyKeyState:
                self.__pressVyButtonTime = rospy.Time.now()
            self.__vyKeyState = keyState
            if isVyForwardPress:
                vy += self.__keyVyFunction.compute((rospy.Time.now() - self.__pressVyButtonTime).to_sec())
            else:
                vy += self.self.__keyVyFunction.compute(-(rospy.Time.now() - self.__pressVyButtonTime).to_sec())
        return vx, vy, vrz

    def handleSpinningTop(self, rc, keyboard, joints, vx, vy, vrz):
        """
        小陀螺模式处理
        
        @param rc 遥控器信息
        @param keyboard 键盘信息
        @param vx 上级X轴线速度
        @param vy 上级Y轴线速度
        @param vrz 上级Z轴旋转速度
        @param joints 关节信息
        
        @return vx, vy, vrz
        """
        vrzKeyState = getattr(keyboard, self.__vrzKey)
        if self.__lastVrzKeyState != vrzKeyState and vrzKeyState:
            self.isVrzEnable = not self.isVrzEnable
            if self.isVrzEnable:
                self.__pressVrzButtonTime = rospy.Time.now()
        self.__lastVrzKeyState = vrzKeyState
        if self.isVrzEnable:
            vrz += self.__keyVrzFunction.compute((rospy.Time.now() - self.__pressVrzButtonTime).to_sec())
        vxTarget, vyTarget = vx, vy
        vx = math.cos(joints[self.__yawJointName]["position"]) * vxTarget - math.sin(joints[self.__yawJointName]["position"]) * vyTarget
        vy = math.cos(joints[self.__yawJointName]["position"]) * vyTarget + math.sin(joints[self.__yawJointName]["position"]) * vxTarget
        return vx, vy, vrz
    
    def chassisFollowGimbal(self, joints, vx, vy, vrz):
        """
        底盘跟随云台
        
        @return vx, vy, vrz
        """
        if not self.isVrzEnable:
            vrz += self.__chassisFollowGimbalP * (0 - joints[self.__yawJointName]["position"])
        return vx, vy, vrz

    def run(self, main, rc, mouse, keyboard, joints):
        vx, vy, vrz = self.handleRCAndKeyboardMessage(rc, keyboard)
        vx, vy, vrz = self.handleSpinningTop(rc, keyboard, joints, vx, vy, vrz)
        vx, vy, vrz = self.chassisFollowGimbal(joints, vx, vy, vrz)
        # 超级电容限速(该部分逻辑暂时不实现,等待新硬件方案)
        # 发送话题
        twist = Twist()
        twist.linear.x  = vx
        twist.linear.y  = vy
        twist.angular.z = vrz
        self.__twistPublisher.publish(twist)