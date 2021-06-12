from .. import widget, color
from rm_referee_controller.msg import WidgetRectangle

class RectangleWidget(widget.Widget):
    # 颜色
    color = widget.creatProperty("color", lambda val: color.ColorType.RED_BLUE <= val <= color.ColorType.WHITE and isinstance(val, color.ColorType))
    # 线条宽度
    width = widget.creatProperty("width", lambda val: 0 < val <= 1023 and isinstance(val, int)) 
    # 终点X坐标
    endX = widget.creatProperty("endX", lambda val: 0 <= val <= 2047 and isinstance(val, int))
    # 终点Y坐标
    endY = widget.creatProperty("endY", lambda val: 0 <= val <= 2047 and isinstance(val, int))
    # 消息类型
    msgType = WidgetRectangle
    # 重命名消息参数,__DEL__为key的,内容为需要删除的属性,请注意更新检查这个不会影响
    mapName = {
        "startX": "absoluteX",
        "startY": "absoluteY",
        "__DEL__": ["endRelativeX", "endRelativeY"]
    }
    
    def __init__(self, **kargs):
        super(RectangleWidget, self).__init__(**kargs)
        # 默认值
        self.color = color.ColorType.RED_BLUE
        self.width = 1
        self.endX = 0
        self.endY = 0

    def updateCallback(self, parentX, parentY):
        self._Widget__property["endX"] =  self.endX + parentX
        self._Widget__property["endY"] =  self.endY + parentY
        # 绕过对这两个相对坐标的更新检测
        self._Widget__oldProperty["endRelativeX"] = self._Widget__property["endRelativeX"]
        self._Widget__oldProperty["endRelativeY"] = self._Widget__property["endRelativeY"]
