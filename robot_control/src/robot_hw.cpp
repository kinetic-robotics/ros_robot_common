#include <algorithm>

#include <hardware_interface/robot_hw.h>
#include <urdf/model.h>

#include "robot_control/motor.h"
#include "robot_control/rc.h"
#include "robot_control/robot_hw.h"
#include "robot_control/supercap.h"
#include "robot_control/io.h"

namespace robot_control
{
RobotHW::RobotHW(ros::NodeHandle& node, ros::NodeHandle& nodeParam)
    : node_(node), nodeParam_(nodeParam)
{
}

bool RobotHW::init()
{
    /* 解析配置 */
    std::string urdfFile;
    std::vector<std::string> enableModules; /* 启用的模块 */
    nodeParam_.getParam("modules", enableModules);
    /* 读取一遍URDF确认没错误 */
    urdf::Model urdfModel;
    if (!nodeParam_.getParam("/robot_description", urdfFile) || !urdfModel.initString(urdfFile)) {
        ROS_FATAL("Read parameter [/robot_desciption] failed! Parse URDF failed!");
        return false;
    }
    urdfModel.clear();
    communicationDriver_.reset(new CommunicationDriver(node_, nodeParam_));
    /* 加载各个组件 */
    if (std::find(enableModules.begin(), enableModules.end(), "motor") != enableModules.end()) {
        modules_["motor"] = std::make_shared<MotorDriver>(node_, nodeParam_, urdfFile, *communicationDriver_, *this);
    }
    if (std::find(enableModules.begin(), enableModules.end(), "rc") != enableModules.end()) {
        modules_["rc"] = std::make_shared<RCDriver>(node_, nodeParam_, urdfFile, *communicationDriver_, *this);
    }
    if (std::find(enableModules.begin(), enableModules.end(), "supercap") != enableModules.end()) {
        modules_["supercap"] = std::make_shared<SupercapDriver>(node_, nodeParam_, urdfFile, *communicationDriver_, *this);
    }
    if (std::find(enableModules.begin(), enableModules.end(), "io") != enableModules.end()) {
        modules_["io"] = std::make_shared<IODriver>(node_, nodeParam_, urdfFile, *communicationDriver_, *this);
    }
    /* 初始化各组件 */
    if (!communicationDriver_->init()) {
        ROS_FATAL("Load communication module failed!");
        return false;
    }
    for (auto iter = modules_.begin(); iter != modules_.end(); iter++) {
        if (!iter->second->init()) {
            ROS_FATAL("Load module[%s] failed!", iter->first.c_str());
            return false;
        }
    }
    return true;
}

void RobotHW::read(const ros::Time& time, const ros::Duration& period)
{
    communicationDriver_->read(time, period);
    for (auto iter = modules_.begin(); iter != modules_.end(); iter++) {
        iter->second->read(time, period);
    }
}

void RobotHW::write(const ros::Time& time, const ros::Duration& period)
{
    communicationDriver_->write(time, period);
    for (auto iter = modules_.begin(); iter != modules_.end(); iter++) {
        iter->second->write(time, period);
    }
}

void RobotHW::shutdown()
{
    for (auto iter = modules_.begin(); iter != modules_.end(); iter++) {
        iter->second->shutdown();
    }
    communicationDriver_->shutdown();
}

}  // namespace robot_control