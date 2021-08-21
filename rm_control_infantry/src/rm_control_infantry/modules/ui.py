import rospy, math
from rm_referee_ui_framework.color import ColorType
from rm_referee_ui_framework.manager import Manager
from rm_referee_ui_framework.widgets import StringWidget, IntNumberWidget, LineWidget, CircleWidget
from robot_msgs.msg import BoolStamped
from supercap_controller.msg import SupercapState
from ..module import ModuleBase

class UIModule(ModuleBase):
    def __init__(self):
        super().__init__()
        self.__isRefereeOnline = False
        self.__lastRedrawUIKeyState = False
        self.uiManager = Manager()

    def init(self):
        # 读取参数  
        self.__uiUpdateTopic = rospy.get_param("~ui/topic/referee_system_ui_update")
        self.__uiDeleteTopic = rospy.get_param("~ui/topic/referee_system_ui_delete")
        self.__refereeOnlineTopic = rospy.get_param("~ui/topic/referee_system_online")
        self.__supercapStateTopic = rospy.get_param("~ui/topic/supercap_state")
        self.__supercapPercentLowThreshold = rospy.get_param("~ui/supercap/low_threshold")
        self.__redrawUIKey = rospy.get_param("~ui/keyboard/redraw_ui")
        self.__pitchJointName = rospy.get_param("~ui/pitch_joint_name")
        self.__layer = rospy.get_param("~ui/layer")
        # 初始化UI框架
        self.uiManager.init(self.__layer, self.__uiUpdateTopic, self.__uiDeleteTopic)
        # 初始化UI
        self.initUI()
        # 等待连接
        rospy.sleep(5)
        # 订阅
        self.__refereeOnlineSubscriber = rospy.Subscriber(self.__refereeOnlineTopic, BoolStamped, self.refereeOnlineCallback)
        self.__supercapStateSubscriber = rospy.Subscriber(self.__supercapStateTopic, SupercapState, self.supercapStateCallback)
        return True

    def initUI(self):
        """
        初始化UI组件
        """
        self.supercapPercentNumber = IntNumberWidget()
        self.supercapPercentNumber.color = ColorType.YELLOW
        self.supercapPercentNumber.relativeX = 1365
        self.supercapPercentNumber.relativeY = 660
        self.supercapPercentNumber.fontSize = 50
        self.supercapPercentNumber.width = 5
        self.supercapPercentNumber.isForceUpdate = True
        self.uiManager.add(self.supercapPercentNumber)

        self.pitchAngleNumber = IntNumberWidget()
        self.pitchAngleNumber.color = ColorType.YELLOW
        self.pitchAngleNumber.relativeX = 1365
        self.pitchAngleNumber.relativeY = 460
        self.pitchAngleNumber.fontSize = 50
        self.pitchAngleNumber.width = 5
        self.pitchAngleNumber.isForceUpdate = True
        self.uiManager.add(self.pitchAngleNumber)

        self.spinCircle = CircleWidget()
        self.spinCircle.color = ColorType.YELLOW
        self.spinCircle.relativeX = 525
        self.spinCircle.relativeY = 635
        self.spinCircle.width = 10
        self.spinCircle.radius = 20
        self.spinCircle.isForceUpdate = True
        self.uiManager.add(self.spinCircle)

        self.bulletCoverCircle = CircleWidget()
        self.bulletCoverCircle.color = ColorType.YELLOW
        self.bulletCoverCircle.relativeX = 525
        self.bulletCoverCircle.relativeY = 430
        self.bulletCoverCircle.width = 10
        self.bulletCoverCircle.radius = 20
        self.bulletCoverCircle.isForceUpdate = True
        self.uiManager.add(self.bulletCoverCircle)

        self.sightXLine = LineWidget()
        self.sightXLine.color = ColorType.YELLOW
        self.sightXLine.relativeX = 910
        self.sightXLine.relativeY = 540
        self.sightXLine.endX = self.sightXLine.relativeX + 100
        self.sightXLine.endY = self.sightXLine.relativeY
        self.sightXLine.width = 5
        self.uiManager.add(self.sightXLine)

        self.sightYLine = LineWidget()
        self.sightYLine.color = ColorType.YELLOW
        self.sightYLine.endX = int((self.sightXLine.endX - self.sightXLine.relativeX) / 2)
        self.sightYLine.endY = 50
        self.sightYLine.relativeX = int((self.sightXLine.endX - self.sightXLine.relativeX) / 2)
        self.sightYLine.relativeY = int(-self.sightYLine.endY)
        self.sightYLine.width = 5
        self.sightXLine.add(self.sightYLine)
    
    def refereeOnlineCallback(self, data):
        """
        裁判系统在线话题回调
        """
        if not self.__isRefereeOnline and data.result:
            self.uiManager.update(forceUpdate = True)
        self.__isRefereeOnline = data.result

    def supercapStateCallback(self, data):
        """
        超级电容在线话题回调
        """
        self.supercapPercentNumber.data = (int)(data.percent * 100)
        if data.percent < self.__supercapPercentLowThreshold:
            self.supercapPercentNumber.color = ColorType.YELLOW
        else:
            self.supercapPercentNumber.color = ColorType.GREEN
    
    def run(self, main, rc, mouse, keyboard, joints):
        # 键盘信息处理
        keyState = getattr(keyboard, self.__redrawUIKey)
        if self.__lastRedrawUIKeyState != keyState and keyState:
            self.uiManager.update(forceUpdate = True)
        self.__lastRedrawUIKeyState = keyState
        # 电机状态处理
        if self.__pitchJointName not in joints:
            rospy.logerr("Not found pitch joint!")
        else:
            # 转换为角度
            angle = int(joints[self.__pitchJointName]["position"] / math.pi * 180)
            self.pitchAngleNumber.color = ColorType.YELLOW if angle >= 0 else ColorType.GREEN
            self.pitchAngleNumber.data = abs(angle)
        # 小陀螺状态
        self.spinCircle.color = ColorType.YELLOW if main.chassisModule.isVrzEnable else ColorType.GREEN
        # 弹舱盖状态
        self.bulletCoverCircle.color = ColorType.YELLOW if main.bulletCoverModule.isOpen else ColorType.GREEN
        if self.__isRefereeOnline:
            self.uiManager.update()