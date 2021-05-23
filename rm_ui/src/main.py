import rospy
from rm_referee_ui_framework.color import ColorType
from rm_referee_ui_framework.manager import Manager
from rm_referee_ui_framework.widgets import StringWidget, IntNumberWidget
from rm_rc_controller.msg import Keyboard
from robot_msgs.msg import BoolStamped
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
        self.redrawUIKey = rospy.get_param("~keyboard/redraw_ui")
        self.layer = rospy.get_param("~layer")
        # 初始化UI框架
        self.uiManager = Manager()
        self.uiManager.init(self.layer, self.uiUpdateTopic, self.uiDeleteTopic)
        # 订阅
        self.refereeOnlineSubscriber = rospy.Subscriber(self.refereeOnlineTopic, BoolStamped, self.refereeOnlineCallback)
        self.supercapStateSubscriber = rospy.Subscriber(self.supercapStateTopic, SupercapState, self.supercapStateCallback)
        self.keyboardSubscriber = rospy.Subscriber(self.keyboardTopic, Keyboard, self.keyboardCallback)
        # 裁判系统是否在线
        self.isRefereeOnline = False
        # 超级电容百分比
        self.supercapPercent = 0.0
        # 是否第一次调用initUI
        self.isFirstCallInitUI = True
        # 键盘重制ui按键上次状态
        self.lastRedrawUIKeyState = False
        # 等待连接
        rospy.sleep(1)

    def initUI(self):
        """
        初始化UI组件
        """
        if not self.isFirstCallInitUI:
            rospy.logerr("Call init UI more than once!")
            return
        self.isFirstCallInitUI = False
        self.supercapPercentNumber = IntNumberWidget()
        self.supercapPercentNumber.id = 1
        self.supercapPercentNumber.color = ColorType.WHITE
        self.supercapPercentNumber.relativeX = 1365
        self.supercapPercentNumber.relativeY = 660
        self.supercapPercentNumber.fontSize = 50
        self.supercapPercentNumber.width = 5
        self.uiManager.add(self.supercapPercentNumber)
        rospy.loginfo("RobotMaster UI Node started!")
    
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
        self.supercapPercent = data.percent

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
        self.supercapPercentNumber.data = (int)(self.supercapPercent * 100)
        if self.supercapPercent < self.supercapPercentLowThreshold:
            self.supercapPercentNumber.color = ColorType.YELLOW
        else:
            self.supercapPercentNumber.color = ColorType.GREEN
        if self.isRefereeOnline:
            self.uiManager.update()

if __name__ == '__main__':
    uiMain = UIMain()
    uiMain.initUI()
    rospy.Timer(rospy.Duration(1.0/rospy.get_param("~rate")), uiMain.loop)
    rospy.spin()