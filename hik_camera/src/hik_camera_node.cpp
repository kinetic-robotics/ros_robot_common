#include <ros/ros.h>
#include <hik_camera/hik_camera.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hik_camera");
  ros::NodeHandle node("~"), nodeParam("~");
  hik_camera::HikCameraNode camera(node, nodeParam);
  if (!camera.init()) return -1;
  ros::spin();
  return 0;
}

