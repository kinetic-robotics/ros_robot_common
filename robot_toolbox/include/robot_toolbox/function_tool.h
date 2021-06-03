#ifndef ROBOT_TOOLBOX_FUNCTION_TOOL_H_
#define ROBOT_TOOLBOX_FUNCTION_TOOL_H_

#include <ros/ros.h>

#include <std_srvs/Trigger.h>

namespace robot_toolbox
{
class FunctionTool
{
  private:
    ros::NodeHandle node_;            /* 节点 */
    std::vector<double> xList_;       /* 转折点X坐标 */
    std::vector<double> yList_;       /* 转折点Y坐标 */
    ros::ServiceServer updateServer_; /* 动态更新服务器 */

    /**
     * 更新配置服务
     * 
     * @param request 请求
     * @param response 回应
     * @return true
     */
    bool updateConfigCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

  public:
    FunctionTool(ros::NodeHandle node);

    /**
     * 初始化函数工具
     * 
     * @return 初始化是否成功
     */
    bool init();

    /**
     * 读取配置
     * 
     * @return 是否成功 
     */
    bool readConfig();

    /**
     * 计算输出
     * 
     * @param input 输入
     * @return 输出
     */
    double compute(double input);
};
}  // namespace robot_toolbox

#endif