import rospy

def ConfigAssert(name, condition):
    """
    判断配置是否正确, 若配置有问题会输出日记
    
    @param name 配置名称
    @param condition 判断条件,true即为正确

    @return 是否正确
    """
    if not condition:
        rospy.logfatal("Read parameter [%s] failed!" % name)
        return False
    return True

def CheckValidKey(name):
    """
    判断按键是否为遥控器发送的有效按键
    
    @param name 按键名称

    @return 是否正确
    """
    return name in ["W", "S", "A", "D", "SHIFT", "CTRL", "Q", "E", "R", "F", "G", "Z", "X", "C", "V", "B"]

def ClampNumber(number, up, down):
    """
    限制输入范围
    
    @param number 输入实数,可为int,float等任何支持比较的类型
    @param up 上界
    @param down 下界

    @return 输出实数
    """
    if number > up:
        return up
    if number < down:
        return down
    return number