import rospy
from rm_rc_controller.msg import Keyboard, Mouse
from sensor_msgs.msg import Joy, JointState
from .modules.bullet_cover import BulletCoverModule
from .modules.chassis import ChassisModule
from .modules.gimbal import GimbalModule
from .modules.laser import LaserModule
from .modules.safety import SafetyModule
from .modules.shot import ShotModule
from .modules.ui import UIModule

class Main(object):
    def __init__(self):
        super().__init__()
        self.jointsData, self.mouseData, self.rcData = None, None, None

    def keyboardCallback(self, msg):
        self.keyboardData = msg

    def mouseCallback(self, msg):
        self.mouseData = msg
        
    def rcCallback(self, msg):
        self.rcData = msg

    def jointStateCallback(self, msg):
        data = {}
        for i in range(len(msg.name)):
            data[msg.name[i]] = {
                "position": msg.position[i],
                "velocity": msg.velocity[i],
                "effort": msg.effort[i]
            }
        self.jointsData = data

    def initModules(self):
        self.bulletCoverModule = BulletCoverModule()
        self.chassisModule = ChassisModule()
        self.gimbalModule = GimbalModule()
        self.laserModule = LaserModule()
        self.safetyModule = SafetyModule()
        self.shotModule = ShotModule()
        self.uiModule = UIModule()
        if not self.bulletCoverModule.init() or not self.chassisModule.init() or not self.gimbalModule.init() or \
           not self.laserModule.init() or not self.safetyModule.init() or not self.shotModule.init() or not self.uiModule.init():
           exit(-1)

    def init(self):
        rospy.init_node("rm_control_infantry")
        # 订阅话题
        self.__keyboardSubscriber = rospy.Subscriber(rospy.get_param("~topic/keyboard"), Keyboard, self.keyboardCallback, queue_size=1000)
        self.__mouseSubscriber = rospy.Subscriber(rospy.get_param("~topic/mouse"), Mouse, self.mouseCallback, queue_size=1000)
        self.__rcSubscriber = rospy.Subscriber(rospy.get_param("~topic/rc"), Joy, self.rcCallback, queue_size=1000)
        self.__jointStateSubscriber = rospy.Subscriber(rospy.get_param("~topic/joint_state"), JointState, self.jointStateCallback, queue_size=1000)
        # 初始化模块
        self.initModules()
        # 初始化主循环
        hz = rospy.get_param("~loop/rate")
        self.__loopTimer = rospy.Timer(rospy.Duration.from_sec(1 / hz), self.loop)
        rospy.loginfo("Robomaster Control for infantry started!")
        rospy.on_shutdown(self.shutdownCallback)
        rospy.spin()

    def shutdownCallback(self):
        """
        节点关闭回调
        """
        self.__loopTimer.shutdown()

    def loop(self, timer):
        """
        主循环

        @param timer 计时器
        """
        if self.rcData == None or self.keyboardData == None or self.mouseData == None or self.jointsData == None:
            return
        self.bulletCoverModule.run(self, self.rcData, self.mouseData, self.keyboardData, self.jointsData)
        self.chassisModule.run(self, self.rcData, self.mouseData, self.keyboardData, self.jointsData)
        self.gimbalModule.run(self, self.rcData, self.mouseData, self.keyboardData, self.jointsData)
        self.laserModule.run(self, self.rcData, self.mouseData, self.keyboardData, self.jointsData)
        self.safetyModule.run(self, self.rcData, self.mouseData, self.keyboardData, self.jointsData)
        self.shotModule.run(self, self.rcData, self.mouseData, self.keyboardData, self.jointsData)
        self.uiModule.run(self, self.rcData, self.mouseData, self.keyboardData, self.jointsData)