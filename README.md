# robot_common
ROS版本机器人包库,包含了适用于所有机器人的代码.  
搭配[robot_runtime](https://github.com/kinetic-robotics/ros_robot_runtime)食用更佳.

## Usage
### Compile
```
source devel/setup.zsh
catkin_make -Dcmake_build_type=debug
```

## Requirements
- Ubuntu 20.04 或基于它的发行版
- ROS Noetic
- Python3和Pip3
- 其他具体请查看各个库的package.xml