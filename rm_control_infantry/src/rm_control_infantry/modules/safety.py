from ..module import ModuleBase
from robot_msgs.msg import BoolStamped
import rospy

class SafetyModule(ModuleBase):
    def __init__(self):
        super().__init__()
        self.__cmdMsg = BoolStamped()
        self.__isRCOnline = False

    def init(self):
        self.__cmdPublisher = rospy.Publisher(rospy.get_param("~safety/topic/cmd"), BoolStamped, queue_size=1000)
        self.__rcOnlineSubscriber = rospy.Subscriber(rospy.get_param("~safety/topic/rc_online"), BoolStamped, self.rcOnlineCallback, queue_size=1000)
        return True

    def rcOnlineCallback(self, msg):
        """
        遥控器是否在线接收回调
        
        @param msg 消息
        """
        self.__isRCOnline = msg.result
    
    def run(self, main, rc, mouse, keyboard, joints):
        self.__cmdMsg.result = self.__isRCOnline and rc.sw[1] != -1
        # 发送话题
        self.__cmdMsg.header.seq += 1
        self.__cmdMsg.header.stamp = rospy.Time.now()
        self.__cmdPublisher.publish(self.__cmdMsg)