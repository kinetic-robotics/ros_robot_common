#include <ros/ros.h>

#include <robot_toolbox/tool.h>

#include "rm_control/channel/channel_manager.h"
#include "rm_control/module/chassis_follow_gimbal.h"
#include "rm_control/module/module.h"
#include "rm_control/module/supercap.h"
#include "rm_control/module/friction.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "rm_control_node");
    ros::NodeHandle node(""), nodeParam("~");
    int paramRate; /* 参数里的执行频率 */
    CONFIG_ASSERT("rate", nodeParam.getParam("rate", paramRate) && paramRate > 0);
    ros::Rate rate(paramRate);
    /* 加载各模块 */
    std::map<std::string, std::shared_ptr<rm_control::ModuleInterface>> modules_; /* 模块数组 */
    std::vector<std::string> enableModules;                                       /* 启用的模块 */
    std::map<std::string, bool> modulesStatus;                                    /* 模块的状态,主要允许通道动态修改模块是否启用 */
    nodeParam.getParam("modules", enableModules);
    if (std::find(enableModules.begin(), enableModules.end(), "supercap") != enableModules.end()) {
        modules_["supercap"] = std::make_shared<rm_control::SupercapModule>(node, nodeParam);
    }
    if (std::find(enableModules.begin(), enableModules.end(), "chassis_follow_gimbal") != enableModules.end()) {
        modules_["chassis_follow_gimbal"] = std::make_shared<rm_control::ChassisFollowGimbalModule>(node, nodeParam);
    }
    if (std::find(enableModules.begin(), enableModules.end(), "friction") != enableModules.end()) {
        modules_["friction"] = std::make_shared<rm_control::FrictionModule>(node, nodeParam);
    }
    /* 初始化模块 */
    for (auto iter = modules_.begin(); iter != modules_.end(); iter++) {
        if (!iter->second->init()) {
            ROS_FATAL("Load Module[%s] failed!", iter->first.c_str());
            return -1;
        }
        modulesStatus[iter->first] = true;
    }
    /* 初始化通道 */
    rm_control::ChannelManager channelManager(node, nodeParam, modulesStatus);
    if (!channelManager.init()) {
        ROS_FATAL("Load Channel failed!");
        return -1;
    }
    while (ros::ok()) {
        double vx = 0, vy = 0, vrz = 0, yawAngle = 0, pitchAngle = 0;
        channelManager.update(vx, vy, vrz, yawAngle, pitchAngle, rate.expectedCycleTime());
        for (auto iter = modules_.begin(); iter != modules_.end(); iter++) {
            iter->second->getValue(vx, vy, vrz, yawAngle, pitchAngle, modulesStatus[iter->first], rate.expectedCycleTime());
        }
        ros::spinOnce();
        rate.sleep();
    }
    ros::waitForShutdown();
    return 0;
}