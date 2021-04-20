#include <ros/ros.h>

#include <controller_manager/controller_manager.h>
#include <robot_toolbox/tool.h>

#include "robot_control/robot_hw.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robot_control_node");
    ros::NodeHandle node(""), nodeParam("~");
    robot_control::RobotHW robotHW(node, nodeParam);
    ros::AsyncSpinner spinner(1);
    controller_manager::ControllerManager cm(&robotHW, node);
    int paramRate; /* 参数里的执行频率 */
    CONFIG_ASSERT("rate", nodeParam.getParam("rate", paramRate) && paramRate > 0);
    ros::Rate rate(paramRate);
    spinner.start();
    if (!robotHW.init()) return -1;
    while (ros::ok()) {
        robotHW.read(ros::Time::now(), rate.expectedCycleTime());
        cm.update(ros::Time::now(), rate.expectedCycleTime());
        robotHW.write(ros::Time::now(), rate.expectedCycleTime());
        rate.sleep();
    }
    spinner.stop();
    robotHW.shutdown();
    ros::waitForShutdown();
    return 0;
}