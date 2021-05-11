#include "robot_toolbox/function_tool.h"
#include "robot_toolbox/tool.h"

namespace robot_toolbox
{
FunctionTool::FunctionTool(ros::NodeHandle node, ros::NodeHandle nodeParam)
: node_(node), nodeParam_(nodeParam)
{
}

bool FunctionTool::readConfig()
{
    xList_.clear();
    yList_.clear();
    CONFIG_ASSERT("x", nodeParam_.getParam("x", xList_) && xList_.size() > 1);
    CONFIG_ASSERT("y", nodeParam_.getParam("y", yList_) && yList_.size() > 1 && yList_.size() == xList_.size());
    return true;
}

bool FunctionTool::updateConfigCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
    response.success = readConfig();
    return true;
}

bool FunctionTool::init()
{
    if (!readConfig()) return false;
    return true;
}

double FunctionTool::compute(double input)
{
    /* 查找对应的X输入范围 */
    int inputUpperNumber = std::upper_bound(xList_.begin(), xList_.end(), input) - xList_.begin();
    int inputLowerNumber = inputUpperNumber - 1;
    if (inputUpperNumber == 0) {
        inputLowerNumber = inputUpperNumber;
        inputUpperNumber = inputUpperNumber + 1;
    }
    if (inputUpperNumber == xList_.size()) {
        inputUpperNumber = inputLowerNumber;
        inputLowerNumber = inputUpperNumber - 1;
    }
    /* 计算结果 */
    return (yList_[inputUpperNumber] - yList_[inputLowerNumber]) / (xList_[inputUpperNumber] - xList_[inputLowerNumber]) * (input - xList_[inputLowerNumber]) + yList_[inputLowerNumber];
}

}  // namespace robot_toolbox