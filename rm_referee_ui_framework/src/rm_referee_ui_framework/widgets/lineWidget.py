from .. import widget, color
from rm_referee_controller.msg import WidgetLine

class LineWidget(widget.Widget):
    # 颜色
    color = widget.creatProperty("color", lambda val: color.ColorType.RED_BLUE <= val <= color.ColorType.WHITE and isinstance(val, color.ColorType))
    # 线条宽度
    width = widget.creatProperty("width", lambda val: 0 <= val <= 1023 and isinstance(val, int)) 
    # 终点X坐标
    endX = widget.creatProperty("endX", lambda val: 0 <= val <= 2047 and isinstance(val, int))
    # 终点Y坐标
    endY = widget.creatProperty("endY", lambda val: 0 <= val <= 2047 and isinstance(val, int))
    # 消息类型
    msgType = WidgetLine
    # 重命名消息参数
    mapName = {
        "startX": "absoluteX",
        "startY": "absoluteY"
    }

    def __init__(self, **kargs):
        super(LineWidget, self).__init__(**kargs)
        # 默认值
        self.color = color.ColorType.RED_BLUE
        self.width = 0
        self.endX = 0
        self.endY = 0
        self.__oldEndX = 0
        self.__oldEndY = 0

    def updateCallback(self, parentX, parentY):
        # 如果变更了,就重新计算end坐标,因为用户输入的是相对坐标
        if self.__oldEndX != self.endX:
            self.endX = self.endX + parentX
        self.__oldEndX = self.endX
        if self.__oldEndY != self.endY:
            self.endY = self.endY + parentY
        self.__oldEndY = self.endY