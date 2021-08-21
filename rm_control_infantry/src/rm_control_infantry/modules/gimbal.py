from ..module import ModuleBase
from robot_toolbox.function_tool import FunctionTool
from robot_toolbox.tools import ClampNumber, ConfigAssert
from robot_msgs.msg import Float64Stamped
import rospy

class GimbalModule(ModuleBase):
    def __init__(self):
        super().__init__()
        self.__pit, self.__yaw = 0, 0

    def init(self):
        self.__yawCMD, self.__pitCMD = Float64Stamped(), Float64Stamped()
        self.__pitMaxAngle = rospy.get_param("~gimbal/pitch/max_angle", None)
        self.__pitMinAngle = rospy.get_param("~gimbal/pitch/min_angle", None)
        if not ConfigAssert("gimbal/pitch/max_angle", isinstance(self.__pitMaxAngle, (float, int))) or \
           not ConfigAssert("gimbal/pitch/min_angle", isinstance(self.__pitMinAngle, (float, int))):
           return False
        self.__pitPublisher = rospy.Publisher(rospy.get_param("~gimbal/topic/pitch"), Float64Stamped, queue_size=1000)
        self.__yawPublisher = rospy.Publisher(rospy.get_param("~gimbal/topic/yaw"), Float64Stamped, queue_size=1000)
        self.__joyPitFunction = FunctionTool("~gimbal/joystick/pitch/function")
        self.__joyYawFunction = FunctionTool("~gimbal/joystick/yaw/function")
        self.__pitFunction = FunctionTool("~gimbal/mouse/pitch/function")
        self.__yawFunction = FunctionTool("~gimbal/mouse/yaw/function")
        if not self.__joyPitFunction.init() or not self.__joyYawFunction.init() or \
           not self.__pitFunction.init() or not self.__yawFunction.init() :
            return False
        return True

    def handleRCAndMouseMessage(self, rc, mouse):
        """
        将用户的输入(如摇杆,鼠标)转换为对应的pit和yaw角度指令
        
        @param rc 遥控器信息
        @param mouse 鼠标信息
        
        @return yaw, pit
        """
        # 摇杆逻辑
        self.__pit += self.__joyPitFunction.compute(rc.axes[3])
        self.__yaw += self.__joyYawFunction.compute(rc.axes[2])
        # 鼠标逻辑
        self.__pit += self.__pitFunction.compute(rc.axes[3])
        self.__yaw += self.__yawFunction.compute(rc.axes[2])
        return self.__yaw, self.__pit

    def pitchLimit(self):
        """
        Pitch轴限位

        @return yaw, pit
        """
        self.__pit = ClampNumber(self.__pit, self.__pitMaxAngle, self.__pitMinAngle)
        return self.__yaw, self.__pit
    
    def run(self, main, rc, mouse, keyboard, joints):
        yaw, pit = self.handleRCAndMouseMessage(rc, mouse)
        yaw, pit = self.pitchLimit()
        # 发送话题
        self.__yawCMD.header.stamp = rospy.Time()
        self.__yawCMD.header.seq += 1
        self.__yawCMD.result = yaw
        self.__yawPublisher.publish(self.__yawCMD)
        self.__pitCMD.header.stamp = rospy.Time()
        self.__pitCMD.header.seq += 1
        self.__pitCMD.result = yaw
        self.__pitPublisher.publish(self.__pitCMD)