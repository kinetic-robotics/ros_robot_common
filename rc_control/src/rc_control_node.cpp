#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <robot_toolbox/tool.h>
#include <std_msgs/Float64.h>

#include "rc_control/channel/channel.h"
#include "rc_control/channel/joystick.h"
#include "rc_control/channel/keyboard.h"
#include "rc_control/channel/mouse.h"
#include "rc_control/module/chassis_follow_gimbal.h"
#include "rc_control/module/module.h"
#include "rc_control/module/supercap.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "rc_control_node");
    ros::NodeHandle node(""), nodeParam("~");
    int paramRate; /* 参数里的执行频率 */
    CONFIG_ASSERT("rate", nodeParam.getParam("rate", paramRate) && paramRate > 0);
    ros::Rate rate(paramRate);
    /* 初始化速度信息和云台信息发布者 */
    std::string twistTopic, pitchAngleTopic, yawAngleTopic; /* 速度和云台两轴话题 */
    CONFIG_ASSERT("twist_topic", nodeParam.getParam("twist_topic", twistTopic));
    CONFIG_ASSERT("yaw_angle_topic", nodeParam.getParam("yaw_angle_topic", yawAngleTopic));
    CONFIG_ASSERT("pitch_angle_topic", nodeParam.getParam("pitch_angle_topic", pitchAngleTopic));
    ros::Publisher twistPublisher      = node.advertise<geometry_msgs::Twist>(twistTopic, 1000);
    ros::Publisher yawAnglePublisher   = node.advertise<std_msgs::Float64>(yawAngleTopic, 1000);
    ros::Publisher pitchAnglePublisher = node.advertise<std_msgs::Float64>(pitchAngleTopic, 1000);
    /* 加载各指令通道 */
    std::map<std::string, std::shared_ptr<rc_control::ChannelInterface>> channels_; /* 通道数组 */
    std::vector<std::string> enableChannels;                                        /* 启用的通道 */
    nodeParam.getParam("channels", enableChannels);
    if (std::find(enableChannels.begin(), enableChannels.end(), "joystick") != enableChannels.end()) {
        channels_["joystick"] = std::make_shared<rc_control::JoystickChannel>(node, nodeParam);
    }
    if (std::find(enableChannels.begin(), enableChannels.end(), "keyboard") != enableChannels.end()) {
        channels_["keyboard"] = std::make_shared<rc_control::KeyboardChannel>(node, nodeParam);
    }
    if (std::find(enableChannels.begin(), enableChannels.end(), "mouse") != enableChannels.end()) {
        channels_["mouse"] = std::make_shared<rc_control::MouseChannel>(node, nodeParam);
    }
    /* 加载各模块 */
    std::map<std::string, std::shared_ptr<rc_control::ModuleInterface>> modules_; /* 模块数组 */
    std::vector<std::string> enableModules;                                       /* 启用的模块 */
    nodeParam.getParam("modules", enableModules);
    if (std::find(enableModules.begin(), enableModules.end(), "supercap") != enableModules.end()) {
        modules_["supercap"] = std::make_shared<rc_control::SupercapModule>(node, nodeParam);
    }
    std::map<std::string, bool> modulesStatus; /* 模块的状态,主要允许通道动态修改模块是否启用 */
    if (std::find(enableModules.begin(), enableModules.end(), "chassis_follow_gimbal") != enableModules.end()) {
        modules_["chassis_follow_gimbal"] = std::make_shared<rc_control::ChassisFollowGimbalModule>(node, nodeParam);
    }
    /* 初始化通道 */
    for (auto iter = channels_.begin(); iter != channels_.end(); iter++) {
        if (!iter->second->init()) {
            ROS_FATAL("Load Channel[%s] failed!", iter->first.c_str());
            return -1;
        }
    }
    /* 初始化模块 */
    for (auto iter = modules_.begin(); iter != modules_.end(); iter++) {
        if (!iter->second->init()) {
            ROS_FATAL("Load Module[%s] failed!", iter->first.c_str());
            return -1;
        }
        modulesStatus[iter->first] = true;
    }
    while (ros::ok()) {
        double vx = 0, vy = 0, vrz = 0, yawAngle = 0, pitchAngle = 0;
        bool isDisableSupercapLimit = false;
        for (auto iter = channels_.begin(); iter != channels_.end(); iter++) {
            iter->second->getValue(vx, vy, vrz, yawAngle, pitchAngle, modulesStatus);
        }
        for (auto iter = modules_.begin(); iter != modules_.end(); iter++) {
            iter->second->getValue(vx, vy, vrz, yawAngle, pitchAngle, modulesStatus[iter->first], rate.expectedCycleTime());
        }
        /* 发布速度信息和云台信息 */
        geometry_msgs::Twist twist;
        twist.linear.x  = vx;
        twist.linear.y  = vy;
        twist.angular.z = vrz;
        twistPublisher.publish(twist);
        std_msgs::Float64 pitchCMD;
        pitchCMD.data = pitchAngle;
        pitchAnglePublisher.publish(pitchCMD);
        std_msgs::Float64 yawCMD;
        yawCMD.data = yawAngle;
        yawAnglePublisher.publish(yawCMD);
        rate.sleep();
    }
    ros::waitForShutdown();
    return 0;
}