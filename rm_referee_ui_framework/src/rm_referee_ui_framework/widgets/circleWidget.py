from .. import widget, color
from rm_referee_controller.msg import WidgetCircle


class CircleWidget(widget.Widget):
    # 颜色
    color = widget.creatProperty("color", lambda val: color.ColorType.RED_BLUE <= val <= color.ColorType.WHITE and isinstance(val, color.ColorType))
    # 线条宽度
    width = widget.creatProperty("width", lambda val: 0 < val <= 1023 and isinstance(val, int)) 
    # 半径
    radius = widget.creatProperty("radius", lambda val: 0 < val <= 1023 and isinstance(val, int))
    # 消息类型
    msgType = WidgetCircle
    # 重命名消息参数,__DEL__为key的,内容为需要删除的属性,请注意更新检查这个不会影响
    mapName = {
        "centerX": "absoluteX",
        "centerY": "absoluteY"
    }
    
    def __init__(self, **kargs):
        super(CircleWidget, self).__init__(**kargs)
        # 默认值
        self.color = color.ColorType.RED_BLUE
        self.width = 1
        self.radius = 1
