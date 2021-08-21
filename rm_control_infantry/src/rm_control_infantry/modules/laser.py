from ..module import ModuleBase
from robot_msgs.msg import BoolStamped
from rm_referee_controller.msg import GameStatus
import rospy

class LaserModule(ModuleBase):
    def __init__(self):
        super().__init__()
        self.__cmdMsg = BoolStamped()
        self.__cmdMsg.result = False

    def init(self):
        self.__cmdPublisher = rospy.Publisher(rospy.get_param("~laser/topic/cmd"), BoolStamped, queue_size=1000)
        self.__gameStatusSubscriber = rospy.Subscriber(rospy.get_param("~laser/topic/game_status"), GameStatus, self.gameStatusCallback, queue_size=1000)
        return True

    def gameStatusCallback(self, msg):
        """
        裁判系统比赛状态信息接收回调
        
        @param msg 消息
        """
        self.__cmdMsg.result = msg.process != GameStatus.PROCESS_NOT_START
    
    def run(self, main, rc, mouse, keyboard, joints):
        # 发送话题
        self.__cmdMsg.header.seq += 1
        self.__cmdMsg.header.stamp = rospy.Time.now()
        self.__cmdPublisher.publish(self.__cmdMsg)