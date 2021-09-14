#ifndef HIK_CAMERA_CONTROLLER_H_
#define HIK_CAMERA_CONTROLLER_H_

#include <ros/ros.h>

#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>
#include <hik_camera/CameraConfig.h>
#include <hik_camera/hik_camera.h>
#include <image_transport/image_transport.h>
#include <std_srvs/Empty.h>

namespace hik_camera
{
class HikCameraNode
{
  private:
    ros::NodeHandle node_;                                                                /* ROS节点 */
    ros::NodeHandle nodeParam_;                                                           /* ROS参数节点 */
    std::unique_ptr<camera_info_manager::CameraInfoManager> cameraInfoManager_;           /* 相机校准信息管理器 */
    ros::ServiceServer startCaptureService_;                                              /* 开始推流服务 */
    ros::ServiceServer stopCaptureService_;                                               /* 停止推流服务 */
    uint64_t targetMac_ = 0;                                                              /* 目标摄像头MAC地址 */
    void* deviceHandle_ = 0;                                                              /* 设备句柄 */
    sensor_msgs::Image imgFrame_;                                                         /* 图像ROS消息帧 */
    sensor_msgs::CameraInfo imgInfoFrame_;                                                /* 图像信息ROS消息帧 */
    int singlePixelSize_      = 0;                                                        /* 单像素点占用的大小 */
    int cameraPixelFormatInt_ = 0;                                                        /* 像素点对应的相机格式字符串 */
    image_transport::CameraPublisher cameraPublisher_;                                    /* 相机信息发布 */
    bool isFirstReceivedImage_ = true;                                                    /* 是否是第一次接受图像 */
    ros::Timer captureTimer_;                                                             /* 推流时钟 */
    std::unique_ptr<dynamic_reconfigure::Server<CameraConfig>> dynamicReconfigureServer_; /* 动态配置服务器 */

    /**
     * 连接摄像头
     * 
     * @return 是否成功
     */
    bool connectDevice();

    /**
     * 推流时钟回调
     * 
     * @param event 时钟事件
     * 
     */
    void captureTimerCallback(const ros::TimerEvent& event);

    /**
     * 更新配置
     * 
     * @param config 新配置
     * 
     * @return 是否成功
     */
    bool updateConfig(CameraConfig& config);

    /**
     * 从参数服务器读取动态配置
     * 
     * @return 是否成功
     */
    bool readDynamicConfigFromParameters();

  public:
    HikCameraNode(ros::NodeHandle node, ros::NodeHandle nodeParam): node_(node), nodeParam_(nodeParam){};
    ~HikCameraNode();

    /**
     * 初始化
     */
    bool init();

    /**
     * 开始推流
     * 
     * @return 是否成功
     */
    bool startCapture();

    /**
     * 停止推流
     * 
     * @return 是否成功
     */
    bool stopCapture();
};

#define UPDATE_CONFIG_ASSERT(function, name, value)                    \
    if (int errCode = function(deviceHandle_, name, value) != MV_OK) { \
        ROS_WARN("Set %s failed: %d", name, errCode);                 \
    }
}  // namespace hik_camera
#endif