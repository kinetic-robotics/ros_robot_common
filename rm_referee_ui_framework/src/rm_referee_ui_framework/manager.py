from . import widget
from rm_referee_controller.msg import UIData
import rospy

class Manager(object):
    def __init__(self):
        super().__init__()
        # 创建根节点
        self.__rootWidget = widget.Widget()
        # 发布信息的序号
        self.__seq = 0
        
    
    def init(self, layer, uiTopic):
        """
        初始化UI框架,务必保证每个程序图层数不同
        """
        self.__uiPublisher = rospy.Publisher(uiTopic, UIData, queue_size=1000)
        self.__uiSubscriber = rospy.Subscriber(uiTopic, UIData, self.uiCallback)
        self.__layer = layer
    
    def uiCallback(self, data):
        """
        订阅UI话题,看看有没有程序和我们使用相同的图层数
        """
        if data.layer == self.__layer and data._connection_header["callerid"] != rospy.get_caller_id():
            rospy.logwarn("Found programs which send same ui layer as ours!")

    def add(self, child):
        """
        添加子部件
        """
        self.__rootWidget.add(child)
    
    def update(self):
        """
        发布UI更新
        """
        updateData = self.__rootWidget.update(0, 0)
        publishMessage = UIData()
        publishMessage.header.seq = self.__seq
        self.__seq  = self.__seq + 1
        publishMessage.header.stamp = rospy.Time.now()
        for widgetData in updateData:
            widgetType, publishWidgetData = widgetData
            if widgetType == "ArcWidget":
                publishMessage.arcs.append(publishWidgetData)
            elif widgetType == "CircleWidget":
                publishMessage.circles.append(publishWidgetData)
            elif widgetType == "FloatNumberWidget":
                publishMessage.floatNumbers.append(publishWidgetData)
            elif widgetType == "IntNumberWidget":
                publishMessage.intNumbers.append(publishWidgetData)
            elif widgetType == "LineWidget":
                publishMessage.lines.append(publishWidgetData)
            elif widgetType == "OvalWidget":
                publishMessage.ovals.append(publishWidgetData)
            elif widgetType == "RectangleWidget":
                publishMessage.rectangles.append(publishWidgetData)
            elif widgetType == "StringWidget":
                publishMessage.strings.append(publishWidgetData)
        publishMessage.layer = self.__layer
        self.__uiPublisher.publish(publishMessage)