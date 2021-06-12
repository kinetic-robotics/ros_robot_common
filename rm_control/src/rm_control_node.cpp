#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <robot_msgs/EmptyStamped.h>
#include <robot_msgs/Float64Stamped.h>
#include <robot_toolbox/tool.h>

#include "rm_control/channel/channel_manager.h"
#include "rm_control/module/bullet_cover.h"
#include "rm_control/module/chassis_follow_gimbal.h"
#include "rm_control/module/friction.h"
#include "rm_control/module/heat_limit.h"
#include "rm_control/module/laser.h"
#include "rm_control/module/module.h"
#include "rm_control/module/rotational_move.h"
#include "rm_control/module/safety.h"
#include "rm_control/module/supercap.h"
#include "rm_control/module/vrz_state_publisher.h"
#include "rm_control/rm_control_node.h"

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
    if (std::find(enableModules.begin(), enableModules.end(), "safety") != enableModules.end()) {
        modules_["safety"] = std::make_shared<rm_control::SafetyModule>(node, nodeParam);
    }
    if (std::find(enableModules.begin(), enableModules.end(), "rotational_move") != enableModules.end()) {
        modules_["rotational_move"] = std::make_shared<rm_control::RotationalMoveModule>(node, nodeParam);
    }
    if (std::find(enableModules.begin(), enableModules.end(), "heat_limit") != enableModules.end()) {
        modules_["heat_limit"] = std::make_shared<rm_control::HeatLimitModule>(node, nodeParam);
    }
    if (std::find(enableModules.begin(), enableModules.end(), "bullet_cover") != enableModules.end()) {
        modules_["bullet_cover"] = std::make_shared<rm_control::BulletCoverModule>(node, nodeParam);
    }
    if (std::find(enableModules.begin(), enableModules.end(), "laser") != enableModules.end()) {
        modules_["laser"] = std::make_shared<rm_control::LaserModule>(node, nodeParam);
    }
    if (std::find(enableModules.begin(), enableModules.end(), "vrz_state_publisher") != enableModules.end()) {
        modules_["vrz_state_publisher"] = std::make_shared<rm_control::VRZStatePublisherModule>(node, nodeParam);
    }
    /* 初始化速度信息和云台信息和射击信息发布者 */
    ros::Publisher twistPublisher;                                                                                          /* 速度话题发布者 */
    ros::Publisher yawAnglePublisher;                                                                                       /* YAW轴空间角度发布者 */
    ros::Publisher pitchAnglePublisher;                                                                                     /* Pitch轴空间角度发布者 */
    std::string twistTopic, pitchAngleTopic, yawAngleTopic, shotOnceTopic, shotContinousStartTopic, shotContinousStopTopic; /* 速度和云台两轴和射击话题 */
    double shotContinousCheckTime = 0;
    ros::Publisher shotOncePublisher;           /* 单次射击话题发布 */
    ros::Publisher shotContinousStartPublisher; /* 连续射击开始话题发布 */
    ros::Publisher shotContinousStopPublisher;  /* 连续射击停止话题发布 */
    CONFIG_ASSERT("twist_topic", nodeParam.getParam("twist_topic", twistTopic));
    CONFIG_ASSERT("yaw_angle_topic", nodeParam.getParam("yaw_angle_topic", yawAngleTopic));
    CONFIG_ASSERT("pitch_angle_topic", nodeParam.getParam("pitch_angle_topic", pitchAngleTopic));
    CONFIG_ASSERT("shot_once/start_topic", nodeParam.getParam("shot_once/start_topic", shotOnceTopic));
    CONFIG_ASSERT("shot_continous/start_topic", nodeParam.getParam("shot_continous/start_topic", shotContinousStartTopic));
    CONFIG_ASSERT("shot_continous/stop_topic", nodeParam.getParam("shot_continous/stop_topic", shotContinousStopTopic));
    twistPublisher              = node.advertise<geometry_msgs::Twist>(twistTopic, 1000);
    yawAnglePublisher           = node.advertise<robot_msgs::Float64Stamped>(yawAngleTopic, 1000);
    pitchAnglePublisher         = node.advertise<robot_msgs::Float64Stamped>(pitchAngleTopic, 1000);
    shotOncePublisher           = node.advertise<robot_msgs::EmptyStamped>(shotOnceTopic, 1000, false);
    shotContinousStartPublisher = node.advertise<robot_msgs::EmptyStamped>(shotContinousStartTopic, 1000, false);
    shotContinousStopPublisher  = node.advertise<robot_msgs::EmptyStamped>(shotContinousStopTopic, 1000, false);
    /* 初始化Pitch限位 */
    double maxPitchAngle; /* 最大Pitch轴角度,单位弧度 */
    double minPitchAngle; /* 最小Pitch轴角度,单位弧度 */
    CONFIG_ASSERT("pitch/max_angle", nodeParam.getParam("pitch/max_angle", maxPitchAngle));
    CONFIG_ASSERT("pitch/min_angle", nodeParam.getParam("pitch/min_angle", minPitchAngle) && maxPitchAngle > minPitchAngle);
    /* 初始化模块 */
    for (auto iter = modules_.begin(); iter != modules_.end(); ++iter) {
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
    ROS_INFO("RoboMaster Control node started.");
    rm_control::ShotStatus lastShotStatus = rm_control::ShotStatus::NONE;
    robot_msgs::EmptyStamped shotMsg;
    robot_msgs::Float64Stamped pitchCMD, yawCMD;
    double yawAngle = 0, pitchAngle = 0;
    while (ros::ok()) {
        double vx = 0, vy = 0, vrz = 0;
        rm_control::ShotStatus shotStatus = rm_control::ShotStatus::NONE;
        channelManager.update(vx, vy, vrz, yawAngle, pitchAngle, shotStatus, rate.expectedCycleTime());
        for (auto iter = modules_.begin(); iter != modules_.end(); ++iter) {
            iter->second->getValue(vx, vy, vrz, yawAngle, pitchAngle, shotStatus, modulesStatus[iter->first], rate.expectedCycleTime(), modulesStatus);
        }
        LIMIT(pitchAngle, minPitchAngle, maxPitchAngle);
        if (yawAngle > M_PI * 2) yawAngle = fmod(yawAngle, M_PI * 2);
        if (yawAngle < 0) yawAngle = (M_PI * 2) * ((int)(yawAngle / (-M_PI * 2)) + 1);
        /* 发布射击信息 */
        shotMsg.header.seq++;
        shotMsg.header.stamp = ros::Time::now();
        if (lastShotStatus != shotStatus) {
            switch (shotStatus) {
                case rm_control::ShotStatus::SHOT_CONTINOUS:
                    shotContinousStartPublisher.publish(shotMsg);
                    break;
                case rm_control::ShotStatus::SHOT_CONTINOUS_STOP:
                    shotContinousStopPublisher.publish(shotMsg);
                    break;
                case rm_control::ShotStatus::SHOT_ONCE:
                    shotOncePublisher.publish(shotMsg);
                    break;
            }
        }
        lastShotStatus = shotStatus;
        /* 发布速度信息和云台信息 */
        geometry_msgs::Twist twist;
        twist.linear.x  = vx;
        twist.linear.y  = vy;
        twist.angular.z = vrz;
        twistPublisher.publish(twist);
        pitchCMD.header.stamp = ros::Time::now();
        pitchCMD.header.seq++;
        pitchCMD.result = pitchAngle;
        pitchAnglePublisher.publish(pitchCMD);
        yawCMD.header.stamp = ros::Time::now();
        yawCMD.header.seq++;
        yawCMD.result = yawAngle;
        yawAnglePublisher.publish(yawCMD);
        ros::spinOnce();
        rate.sleep();
    }
    ros::waitForShutdown();
    return 0;
}