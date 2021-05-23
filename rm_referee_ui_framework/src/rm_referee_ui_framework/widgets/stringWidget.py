from .. import widget, color
from rm_referee_controller.msg import WidgetString

class StringWidget(widget.Widget):
    # 颜色
    color = widget.creatProperty("color", lambda val: color.ColorType.RED_BLUE <= val <= color.ColorType.WHITE and isinstance(val, color.ColorType))
    # 线条宽度
    width = widget.creatProperty("width", lambda val: 0 < val <= 1023 and isinstance(val, int)) 
    # 输出的字符串
    data = widget.creatProperty("data", lambda val: 0 <= len(val) <= 30 and isinstance(val, str))
    # 字体大小
    fontSize = widget.creatProperty("fontSize", lambda val: 0 < val <= 511 and isinstance(val, int))
    # 消息类型
    msgType = WidgetString
    # 重命名消息参数
    mapName = {
        "startX": "absoluteX",
        "startY": "absoluteY"
    }
    
    def __init__(self, **kargs):
        super(StringWidget, self).__init__(**kargs)
        # 默认值
        self.color = color.ColorType.RED_BLUE
        self.width = 1
        self.data = ""
        self.fontSize = 1
