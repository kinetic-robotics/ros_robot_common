import random, copy, rospy
import rm_referee_controller.msg

def creatProperty(name, checkFunction = lambda val: True):
    def getter(self):
        if name not in self._Widget__property:
            self._Widget__property[name] = 0
        return self._Widget__property[name]
    def setter(self, val):
        if checkFunction(val):
            self._Widget__property[name] = val
        else:
            rospy.logerr("UI Parameters check failed!")
    return property(getter, setter)

class Widget(object):
    # ID
    id  = creatProperty("id", lambda val: 0 < val <= 256 * 256 * 256 -1 and isinstance(val, int))
    # 相对父组件X坐标
    relativeX = creatProperty("relativeX", lambda val: val <= 2047 and isinstance(val, int))
    # 相对父组件Y坐标
    relativeY = creatProperty("relativeY", lambda val: val <= 2047 and isinstance(val, int))
    # 是否隐藏该组件
    hide  = creatProperty("hide", lambda val: isinstance(val, bool))
    # 是否刷新组件
    isForceUpdate = creatProperty("isForceUpdate", lambda val: isinstance(val, bool))
    # 消息类型
    msgType = None
    # 重命名消息参数,__DEL__为key的,内容为需要删除的属性,请注意更新检查这个不会影响
    mapName = {}
    
    def __init__(self, **kargs):
        """
        构造函数,可以直接传入组件参数但必须为关键字参数
        """
        self.__property = {}
        # ID是3字节的
        self.__property["id"] = random.randint(0, 256*256*256-1)
        # 绝对坐标
        self.__property["absoluteX"] = 0
        self.__property["absoluteY"] = 0
        # 相对坐标
        self.__property["relativeX"] = 0
        self.__property["relativeY"] = 0
        # 默认不隐藏
        self.__property["hide"] = False
        # 旧属性
        self.__oldProperty = {}
        self.__oldHide = None
        # 子Widget
        self.__children = []
        # 赋值
        for key in kargs:
            if hasattr(self, key):
                setattr(self, key, kargs[key])
    
    def updateCallback(self, parentX, parentY):
        """
        生成更新列表时,子类可以通过重载该方法更新自己的参数
        """
        pass

    def update(self, parentX, parentY, hide = False, forceUpdate = False):
        """
        生成Widget更新列表
        """
        self.updateCallback(parentX, parentY)
        updateData = []
        self.__property["absoluteX"] = self.__property["relativeX"] + parentX
        self.__property["absoluteY"] = self.__property["relativeY"] + parentY
        if self.__property["absoluteX"] < 0 or self.__property["absoluteY"] < 0:
            rospy.logerr("UI Parameters check failed!")
            return []
        # 只要上层为hide或者本层为hide,则hide
        hide = hide or self.__property["hide"]
        # 更新检查,如果两次的内容不同或者从未被刷新过就触发刷新
        # 绕过对relativeX和relativeY的检测
        self.__oldProperty["relativeX"] = self.__property["relativeX"]
        self.__oldProperty["relativeY"] = self.__property["relativeY"]
        if self.__oldHide != hide or (not hide and self.__property != self.__oldProperty) or self.__oldProperty == {} or forceUpdate or self.isForceUpdate:
            # 转换为对应的消息类型
            if not hide:
                self.__oldProperty = copy.deepcopy(self.__property)
                publishWidgetData = copy.deepcopy(self.__property)
            else:
                publishWidgetData = copy.deepcopy(self.__oldProperty)
            self.__oldHide = hide
            for name in self.mapName.keys():
                if name != "__DEL__":
                    publishWidgetData[name] = publishWidgetData[self.mapName[name]]
                    del publishWidgetData[self.mapName[name]]
                else:
                    for delName in self.mapName[name]:
                        del publishWidgetData[delName]
            publishWidgetData["hide"] = hide
            del publishWidgetData["relativeX"], publishWidgetData["relativeY"]
            if self.msgType:
                updateData.append((self.__class__.__name__, self.msgType(**publishWidgetData)))
        for child in self.__children:
            childData = child.update(self.__property["absoluteX"], self.__property["absoluteY"], hide, forceUpdate)
            if childData != []:
                updateData = updateData + childData
        return updateData

    def add(self, child):
        """
        添加子部件
        """
        if issubclass(type(child), type(self)):
            self.__children.append(child)
        else:
            rospy.logerr("Must give an subclass of widget to widget's add function!")
