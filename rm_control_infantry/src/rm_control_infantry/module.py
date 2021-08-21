class ModuleBase(object):
    def init(self):
        """
        初始化

        @return 初始化是否成功
        """
        return False

    def run(self, main, rc, mouse, keyboard, joints):
        """
        循环函数

        @param main main类,用于获取其他模块的信息
        @param rc 遥控器信息
        @param mouse 鼠标信息
        @param keyboard 键盘信息
        @param joints 关节字典,name->{position, velocity, effort}
        """
        pass