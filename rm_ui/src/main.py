import rospy, math
from rm_referee_ui_framework.color import ColorType
from rm_referee_ui_framework.manager import Manager
from rm_referee_ui_framework.widgets import StringWidget, IntNumberWidget, LineWidget, CircleWidget
from rm_rc_controller.msg import Keyboard, Mouse
from robot_msgs.msg import BoolStamped
from sensor_msgs.msg import JointState
from supercap_controller.msg import SupercapState

rospy.init_node("rm_ui")

class UIMain(object):
    def __init__(self):
        super().__init__()
        # 读取参数
        self.uiUpdateTopic = rospy.get_param("~topic/referee_system/ui/update")
        self.uiDeleteTopic = rospy.get_param("~topic/referee_system/ui/delete")
        self.refereeOnlineTopic = rospy.get_param("~topic/referee_system/online")
        self.supercapStateTopic = rospy.get_param("~topic/supercap_state")
        self.supercapPercentLowThreshold = rospy.get_param("~supercap/low_threshold")
        self.keyboardTopic = rospy.get_param("~topic/keyboard")
        self.jointStatesTopic = rospy.get_param("~topic/joint_states")
        self.bulletCoverTopic = rospy.get_param("~topic/bullet_cover_command")
        self.spinTopic = rospy.get_param("~topic/spin")
        self.redrawUIKey = rospy.get_param("~keyboard/redraw_ui")
        self.pitchJointName = rospy.get_param("~pitch_joint_name")
        self.layer = rospy.get_param("~layer")
        # 裁判系统是否在线
        self.isRefereeOnline = False
        # 是否第一次调用initUI
        self.isFirstCallInitUI = True
        # 键盘重制ui按键上次状态
        self.lastRedrawUIKeyState = False
        # 初始化UI框架
        self.uiManager = Manager()
        self.uiManager.init(self.layer, self.uiUpdateTopic, self.uiDeleteTopic)
        # 初始化UI
        self.initUI()
        # 等待连接
        rospy.sleep(5)
        # 订阅
        self.refereeOnlineSubscriber = rospy.Subscriber(self.refereeOnlineTopic, BoolStamped, self.refereeOnlineCallback)
        self.supercapStateSubscriber = rospy.Subscriber(self.supercapStateTopic, SupercapState, self.supercapStateCallback)
        self.keyboardSubscriber = rospy.Subscriber(self.keyboardTopic, Keyboard, self.keyboardCallback)
        self.jointStatesSubscriber = rospy.Subscriber(self.jointStatesTopic, JointState, self.jointStatesCallback)
        self.bulletCoverCommandSubscriber = rospy.Subscriber(self.bulletCoverTopic, BoolStamped, self.bulletCoverCallback)
        self.spinSubscriber = rospy.Subscriber(self.spinTopic, BoolStamped, self.spinCallback)
        rospy.loginfo("RobotMaster UI Node started!")

    def initUI(self):
        """
        初始化UI组件
        """
        if not self.isFirstCallInitUI:
            rospy.logerr("Call init UI more than once!")
            return
        self.isFirstCallInitUI = False
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

    def spinCallback(self, data):
        """
        小陀螺状态
        """
        self.spinCircle.color = ColorType.YELLOW if data.result else ColorType.GREEN

    def bulletCoverCallback(self, data):
        """
        弹舱盖状态
        """
        self.bulletCoverCircle.color = ColorType.YELLOW if data.result else ColorType.GREEN

    def jointStatesCallback(self, data):
        """
        电机状态
        """
        i = 0
        for key in data.name:
            if key == self.pitchJointName:
                # 转换为角度
                angle = int(data.position[i] / math.pi * 180)
                self.pitchAngleNumber.color = ColorType.YELLOW if angle >= 0 else ColorType.GREEN
                self.pitchAngleNumber.data = abs(angle)
                return
            i = i + 1
        rospy.logerr("Not found pitch joint!")
    
    def refereeOnlineCallback(self, data):
        """
        裁判系统在线话题回调
        """
        if not self.isRefereeOnline and data.result:
            self.uiManager.update(forceUpdate = True)
        self.isRefereeOnline = data.result

    def supercapStateCallback(self, data):
        """
        超级电容在线话题回调
        """
        self.supercapPercentNumber.data = (int)(data.percent * 100)
        if data.percent < self.supercapPercentLowThreshold:
            self.supercapPercentNumber.color = ColorType.YELLOW
        else:
            self.supercapPercentNumber.color = ColorType.GREEN

    def keyboardCallback(self, data):
        """
        键盘话题回调
        """
        keyState = getattr(data, self.redrawUIKey)
        if self.lastRedrawUIKeyState != keyState and keyState:
            self.uiManager.update(forceUpdate = True)
        self.lastRedrawUIKeyState = keyState
    
    def loop(self, event=None):
        """
        主循环,单次调用
        """
        if self.isRefereeOnline:
            self.uiManager.update()

if __name__ == '__main__':
    uiMain = UIMain()
    rospy.Timer(rospy.Duration(1.0/rospy.get_param("~rate")), uiMain.loop)
    rospy.spin()