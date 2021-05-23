from .. import widget, color
from rm_referee_controller.msg import WidgetFloatNumber

class FloatNumberWidget(widget.Widget):
    # 颜色
    color = widget.creatProperty("color", lambda val: color.ColorType.RED_BLUE <= val <= color.ColorType.WHITE and isinstance(val, color.ColorType))
    # 线条宽度
    width = widget.creatProperty("width", lambda val: 0 < val <= 1023 and isinstance(val, int)) 
    # 输出的数
    data = widget.creatProperty("data", lambda val: isinstance(val, float))
    # 字体大小
    fontSize = widget.creatProperty("fontSize", lambda val: 0 < val <= 511 and isinstance(val, int))
    # 小数点后位数
    digits = widget.creatProperty("digits", lambda val: 0 <= val <= 511 and isinstance(val, int))
    # 消息类型
    msgType = WidgetFloatNumber
    # 重命名消息参数,__DEL__为key的,内容为需要删除的属性,请注意更新检查这个不会影响
    mapName = {
        "startX": "absoluteX",
        "startY": "absoluteY"
    }
    
    def __init__(self, **kargs):
        super(FloatNumberWidget, self).__init__(**kargs)
        # 默认值
        self.color = color.ColorType.RED_BLUE
        self.width = 1
        self.data = 0.0
        self.fontSize = 1
        self.digits = 0
