import rospy
from std_srvs.srv import Trigger, TriggerResponse
from .tools import ConfigAssert
from bisect import bisect_right

class FunctionTool:
    def readConfig(self):
        """
        读取配置
        
        @return 是否成功 
        """
        self.__xList = rospy.get_param(self.__nodePath + "/x", [])
        self.__yList = rospy.get_param(self.__nodePath + "/y", [])
        return ConfigAssert("x", isinstance(self.__xList, list) and len(self.__xList) > 1) and \
               ConfigAssert("y", isinstance(self.__yList, list) and len(self.__yList) > 1 and len(self.__xList) == len(self.__yList))

    def updateConfigCallback(self, request):
        """
        更新配置服务

        @param request 请求
        @return response 回应
        """
        return TriggerResponse(success = self.readConfig())
    
    def compute(self, inputNumber):
        """
        计算输出

        @param input 输入
        @return 输出
        """
        # 查找对应的X输入范围
        inputUpperNumber = bisect_right(self.__xList, inputNumber)
        inputLowerNumber = inputUpperNumber - 1
        if inputUpperNumber == 0:
            inputLowerNumber = inputUpperNumber
            inputUpperNumber = inputUpperNumber + 1
        if inputUpperNumber == len(self.__xList):
            inputUpperNumber = inputLowerNumber
            inputLowerNumber = inputUpperNumber - 1
        # 计算结果
        return (self.__yList[inputUpperNumber] - self.__yList[inputLowerNumber]) / (self.__xList[inputUpperNumber] - self.__xList[inputLowerNumber]) * (inputNumber - self.__xList[inputLowerNumber]) + self.__yList[inputLowerNumber]
    
    def init(self):
        """
        初始化函数工具

        @return 初始化是否成功
        """
        if not self.readConfig():
            return False
        self.__service = rospy.Service(self.__nodePath + "/update_config", Trigger, self.updateConfigCallback)
        return True

    def __init__(self, nodePath):
        super().__init__()
        self.__nodePath = nodePath