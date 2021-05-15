from .. import widget, color
from rm_referee_controller.msg import WidgetOval

class OvalWidget(widget.Widget):
    # 颜色
    color = widget.creatProperty("color", lambda val: color.ColorType.RED_BLUE <= val <= color.ColorType.WHITE and isinstance(val, color.ColorType))
    # 线条宽度
    width = widget.creatProperty("width", lambda val: 0 <= val <= 1023 and isinstance(val, int)) 
    # X半轴长
    xRadius = widget.creatProperty("xRadius", lambda val: 0 <= val <= 2047 and isinstance(val, int))
    # Y半轴长
    yRadius = widget.creatProperty("yRadius", lambda val: 0 <= val <= 2047 and isinstance(val, int))
    # 消息类型
    msgType = WidgetOval
    # 重命名消息参数
    mapName = {
        "centerX": "absoluteX",
        "centerY": "absoluteY"
    }
    
    def __init__(self, **kargs):
        super(OvalWidget, self).__init__(**kargs)
        # 默认值
        self.color = color.ColorType.RED_BLUE
        self.width = 0
        self.xRadius = 0
        self.yRadius = 0
