from ..module import ModuleBase
from robot_msgs.msg import BoolStamped
from robot_toolbox.tools import ConfigAssert, CheckValidKey
from rm_referee_controller.msg import GameStatus
import rospy

class BulletCoverModule(ModuleBase):
    def __init__(self):
        super().__init__()
        self.__cmdMsg = BoolStamped()
        self.isOpen, self.__lastToggleKey = False, False

    def init(self):
        self.__toggleKey = rospy.get_param("~bullet_cover/keyboard/toggle")
        if not ConfigAssert("bullet_cover/keyboard/toggle", CheckValidKey(self.__toggleKey)):
            return False
        self.__cmdPublisher = rospy.Publisher(rospy.get_param("~bullet_cover/topic/cmd"), BoolStamped, queue_size=1000)
        return True
    
    def run(self, main, rc, mouse, keyboard, joints):
        isKeyPress = getattr(keyboard, self.__toggleKey)
        if isKeyPress != self.__lastToggleKey and isKeyPress:
            self.isOpen = not self.isOpen
        self.__cmdMsg.result = self.isOpen
        # 发送话题
        self.__cmdMsg.header.seq += 1
        self.__cmdMsg.header.stamp = rospy.Time.now()
        self.__cmdPublisher.publish(self.__cmdMsg)