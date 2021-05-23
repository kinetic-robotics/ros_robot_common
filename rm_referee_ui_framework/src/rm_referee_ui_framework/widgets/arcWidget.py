from .. import widget, color
import math
from rm_referee_controller.msg import WidgetArc

class ArcWidget(widget.Widget):
    # 颜色
    color = widget.creatProperty("color", lambda val: color.ColorType.RED_BLUE <= val <= color.ColorType.WHITE and isinstance(val, color.ColorType))
    # 线条宽度
    width = widget.creatProperty("width", lambda val: 0 < val <= 1023 and isinstance(val, int)) 
    # 起始角度,单位弧度
    startAngle = widget.creatProperty("startAngle", lambda val: 0 <= val <= math.pi * 2 and isinstance(val, float))
    # 终止角度,单位弧度
    endAngle = widget.creatProperty("endAngle", lambda val: 0 <= val <= math.pi * 2 and isinstance(val, float))
    # X半轴长
    xRadius = widget.creatProperty("xRadius", lambda val: 0 < val <= 2047 and isinstance(val, int))
    # Y半轴长
    yRadius = widget.creatProperty("yRadius", lambda val: 0 < val <= 2047 and isinstance(val, int))
    # 消息类型
    msgType = WidgetArc
    # 重命名消息参数,__DEL__为key的,内容为需要删除的属性,请注意更新检查这个不会影响
    mapName = {
        "centerX": "absoluteX",
        "centerY": "absoluteY"
    }

    def __init__(self, **kargs):
        super(ArcWidget, self).__init__(**kargs)
        # 默认值
        self.color = color.ColorType.RED_BLUE
        self.width = 1
        self.angleStart = 0.0
        self.angleEnd = 0.0
        self.xRadius = 1
        self.yRadius = 1
