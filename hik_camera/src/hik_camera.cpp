#include <ros/ros.h>

#include <hik_camera/mv_camera/MvCameraControl.h>
#include <camera_info_manager/camera_info_manager.h>
#include <hik_camera/hik_camera.h>
#include <image_transport/image_transport.h>
#include <robot_toolbox/tool.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>

namespace hik_camera
{
bool HikCameraNode::connectDevice()
{
    MV_CC_DEVICE_INFO_LIST deviceInfos = {};
    if (int errCode = MV_CC_EnumDevices(MV_USB_DEVICE, &deviceInfos) != MV_OK) {
        ROS_FATAL("Enum devices failed: %d", errCode);
        return false;
    }
    /* 枚举设备 */
    MV_CC_DEVICE_INFO* deviceInfo = 0;
    for (size_t i = 0; i < deviceInfos.nDeviceNum; i++) {
        uint64_t macAddress = (uint64_t)deviceInfos.pDeviceInfo[i]->nMacAddrHigh << 32 | (uint64_t)deviceInfos.pDeviceInfo[i]->nMacAddrLow;
        if (targetMac_ == 0 || macAddress == targetMac_) {
            deviceInfo = deviceInfos.pDeviceInfo[i];
            break;
        }
    }
    if (deviceInfo == 0) {
        ROS_FATAL("Cannot find device!");
        return false;
    }
    /* 检查设备可用并打开设备 */
    if (!MV_CC_IsDeviceAccessible(deviceInfo, MV_ACCESS_Exclusive)) {
        ROS_FATAL("Check device accessible failed!");
        return false;
    }
    if (int errCode = MV_CC_CreateHandleWithoutLog(&deviceHandle_, deviceInfo) != MV_OK) {
        ROS_FATAL("Creat Handle failed: %d", errCode);
        return false;
    }
    if (int errCode = MV_CC_OpenDevice(deviceHandle_, MV_ACCESS_Exclusive, 0) != MV_OK) {
        ROS_FATAL("Open device failed: %d", errCode);
        return false;
    }
    /* 配置 */
    if (int errCode = MV_CC_SetEnumValue(deviceHandle_, "PixelFormat", cameraPixelFormatInt_) != MV_OK) {
        ROS_FATAL("Set Pixel Format failed: %d", errCode);
        return false;
    }
    return true;
}

bool HikCameraNode::startCapture()
{
    if (int errCode = MV_CC_StartGrabbing(deviceHandle_) != MV_OK) {
        ROS_ERROR("Start Grabbing failed: %d", errCode);
        return false;
    }
    return true;
}

bool HikCameraNode::stopCapture()
{
    if (int errCode = MV_CC_StopGrabbing(deviceHandle_) != MV_OK) {
        ROS_ERROR("Start Grabbing failed: %d", errCode);
        return false;
    }
    return true;
}

void HikCameraNode::captureTimerCallback(const ros::TimerEvent& event)
{
    if (!MV_CC_IsDeviceConnected(deviceHandle_)) {
        ROS_ERROR("Device disconnected!");
        return;
    }
    /* 获取图像 */
    MV_FRAME_OUT outFrame = {0};
    if (int errCode = MV_CC_GetImageBuffer(deviceHandle_, &outFrame, 1000) != MV_OK) {
        /* 没有数据 */
        /* 释放内存 */
        if (outFrame.pBufAddr != NULL) {
            if (int errCode = MV_CC_FreeImageBuffer(deviceHandle_, &outFrame) != MV_OK) {
                ROS_ERROR("Free Image Buffer failed: %d", errCode);
                return;
            }
        }
        return;
    }
    /* 拷贝图像 */
    imgFrame_.data.resize(outFrame.stFrameInfo.nWidth * outFrame.stFrameInfo.nHeight * singlePixelSize_);
    memcpy(&imgFrame_.data[0], outFrame.pBufAddr, imgFrame_.data.size());
    /* 释放内存 */
    if (int errCode = MV_CC_FreeImageBuffer(deviceHandle_, &outFrame) != MV_OK) {
        ROS_ERROR("Free Image Buffer failed: %d", errCode);
        return;
    }
    imgFrame_.header.stamp     = event.current_real;
    imgInfoFrame_.header.stamp = event.current_real;
    imgFrame_.header.seq++;
    imgInfoFrame_.header.seq++;
    if (isFirstReceivedImage_) {
        isFirstReceivedImage_ = false;
        imgFrame_.height      = outFrame.stFrameInfo.nHeight;
        imgFrame_.width       = outFrame.stFrameInfo.nWidth;
        imgFrame_.step        = imgFrame_.width * singlePixelSize_;

        /* 如果没有校准数据,那也要图像信息带有基础的宽度和高度 */
        if (!cameraInfoManager_->isCalibrated()) {
            sensor_msgs::CameraInfo info;
            info.height               = imgFrame_.height;
            info.width                = imgFrame_.width;
            imgFrame_.header.frame_id = imgFrame_.header.frame_id;
            cameraInfoManager_->setCameraInfo(info);
        }
    }
    cameraPublisher_.publish(imgFrame_, imgInfoFrame_);
}

bool HikCameraNode::readDynamicConfigFromParameters()
{
    CameraConfig config;
    config.auto_exposure = nodeParam_.param<bool>("exposure/auto_enable", true);
    config.auto_exposure_max = nodeParam_.param<int>("exposure/auto_max", 65);
    config.auto_exposure_min = nodeParam_.param<int>("exposure/auto_min", 10759);
    config.exposure_value = nodeParam_.param<double>("exposure/value", 100);
    CONFIG_ASSERT("exposure/auto_max", config.auto_exposure_max >= 65);
    CONFIG_ASSERT("exposure/auto_min", config.auto_exposure_min >= 65);
    CONFIG_ASSERT("exposure/value", config.exposure_value >= 65);
    config.brightness_value = nodeParam_.param<int>("brightness", 100);
    CONFIG_ASSERT("brightness", config.brightness_value >= 0 && config.brightness_value <= 255);
    config.auto_gain = nodeParam_.param<bool>("gain/auto_enable", true);
    config.auto_gain_max = nodeParam_.param<double>("gain/auto_max", 0);
    config.auto_gain_min = nodeParam_.param<double>("gain/auto_min", 15.0062);
    config.gain_value = nodeParam_.param<double>("gain/value", 2);
    CONFIG_ASSERT("gain/auto_max", config.auto_gain_max >= 0 && config.auto_gain_max <= 15.0062);
    CONFIG_ASSERT("gain/auto_min", config.auto_gain_min >= 0 && config.auto_gain_min <= 15.0062);
    CONFIG_ASSERT("gain/value", config.gain_value >= 0 && config.gain_value <= 15.0062);
    config.black_level_enable = nodeParam_.param<bool>("black_level/enable", false);
    config.black_level_value = nodeParam_.param<double>("black_level/value", 100);
    CONFIG_ASSERT("black_level/value", config.black_level_value >= 0 && config.black_level_value <= 4095);
    config.white_balance_channel = nodeParam_.param<int>("white_balance/channel", 0);
    config.white_balance_value = nodeParam_.param<int>("white_balance/value", 1231);
    config.auto_white_balance = nodeParam_.param<bool>("white_balance/enable", false);
    CONFIG_ASSERT("white_balance/channel", config.white_balance_channel == 0 || config.white_balance_channel == 1 || config.white_balance_channel == 2);
    CONFIG_ASSERT("white_balance/value", config.white_balance_value >= 1 && config.white_balance_value <= 16376);
    config.gamma_enable = nodeParam_.param<bool>("gamma/enable", false);
    config.gamma_mode = nodeParam_.param<int>("gamma/mode", 1);
    config.gamma_value = nodeParam_.param<double>("gamma/value", 0.7);
    CONFIG_ASSERT("gamma/mode", config.gamma_mode == 1 || config.gamma_mode == 2);
    CONFIG_ASSERT("gamma/value", config.gamma_value >= 0 && config.gamma_value <= 4);
    config.hue_enable = nodeParam_.param<bool>("hue/enable", false);
    config.hue_value = nodeParam_.param<int>("hue/value", 128);
    CONFIG_ASSERT("hue/value", config.hue_value >= 0 && config.hue_value <= 255);
    config.saturation_enable = nodeParam_.param<bool>("saturation/enable", false);
    config.saturation_value = nodeParam_.param<int>("saturation/value", 128);
    CONFIG_ASSERT("saturation/value", config.saturation_value >= 0 && config.saturation_value <= 255);
    return updateConfig(config);
}

bool HikCameraNode::init()
{
    /* 读取配置 */
    targetMac_                = std::stoul(nodeParam_.param<std::string>("mac", "0"), 0, 16);
    imgFrame_.header.frame_id = nodeParam_.param<std::string>("frame_id", "camera");
    std::string cameraName = nodeParam_.param<std::string>("camera_name", "main");
    std::string cameraInfoURL = nodeParam_.param<std::string>("camera_info_url", "");
    double fps = nodeParam_.param<int>("framerate", 60);
    CONFIG_ASSERT("framerate", fps > 0);
    /* 读取像素格式 */
    std::string pixelType = nodeParam_.param<std::string>("pixel_format", "rgb8");
    if (pixelType == "rgb8") {
        singlePixelSize_      = 3;
        cameraPixelFormatInt_ = PixelType_Gvsp_RGB8_Packed;
        imgFrame_.encoding    = sensor_msgs::image_encodings::RGB8;
    } else if (pixelType == "mono8") {
        singlePixelSize_      = 1;
        cameraPixelFormatInt_ = PixelType_Gvsp_Mono8;
        imgFrame_.encoding    = sensor_msgs::image_encodings::MONO8;
    } else if (pixelType == "mono16") {
        singlePixelSize_      = 2;
        cameraPixelFormatInt_ = PixelType_Gvsp_Mono16;
        imgFrame_.encoding    = sensor_msgs::image_encodings::MONO16;
    } else if (pixelType == "yuv422_uyvy") {
        singlePixelSize_      = 4;
        cameraPixelFormatInt_ = PixelType_Gvsp_YUV422_Packed;
        imgFrame_.encoding    = sensor_msgs::image_encodings::YUV422;
    } else if (pixelType == "bayer_grbg8") {
        singlePixelSize_      = 3;
        cameraPixelFormatInt_ = PixelType_Gvsp_BayerGR8;
        imgFrame_.encoding    = sensor_msgs::image_encodings::BAYER_GRBG8;
    } else if (pixelType == "bayer_rggb8") {
        singlePixelSize_      = 3;
        cameraPixelFormatInt_ = PixelType_Gvsp_BayerRG8;
        imgFrame_.encoding    = sensor_msgs::image_encodings::BAYER_RGGB8;
    } else if (pixelType == "bayer_gbrg8") {
        singlePixelSize_      = 3;
        cameraPixelFormatInt_ = PixelType_Gvsp_BayerGB8;
        imgFrame_.encoding    = sensor_msgs::image_encodings::BAYER_GBRG8;
    } else if (pixelType == "bayer_bggr8") {
        singlePixelSize_      = 3;
        cameraPixelFormatInt_ = PixelType_Gvsp_BayerBG8;
        imgFrame_.encoding    = sensor_msgs::image_encodings::BAYER_BGGR8;
    } else {
        CONFIG_ASSERT("pixel_format", false);
    }
    /* 初始化帧信息 */
    imgFrame_.is_bigendian        = false;
    imgInfoFrame_.header.frame_id = imgFrame_.header.frame_id;
    /* 连接设备 */
    if (!connectDevice()) return false;
    /* 初始化服务和发布者等 */
    cameraInfoManager_.reset(new camera_info_manager::CameraInfoManager(node_, cameraName, cameraInfoURL));
    imgInfoFrame_ = std::move(cameraInfoManager_->getCameraInfo());
    image_transport::ImageTransport it(node_);
    cameraPublisher_ = it.advertiseCamera("image_raw", 1);
    if (!cameraInfoManager_->isCalibrated()) {
        cameraInfoManager_->setCameraName(cameraName);
    }
    dynamicReconfigureServer_.reset(new dynamic_reconfigure::Server<CameraConfig>(ros::NodeHandle(node_, "camera")));
    dynamic_reconfigure::Server<CameraConfig>::CallbackType callback = boost::bind(&HikCameraNode::updateConfig, this, _1);
    dynamicReconfigureServer_->setCallback(callback);
    startCaptureService_ = node_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("start_capture", boost::bind(&HikCameraNode::startCapture, this));
    stopCaptureService_ = node_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("stop_capture", boost::bind(&HikCameraNode::stopCapture, this));
    /* 打开摄像头 */
    if (!readDynamicConfigFromParameters() || !startCapture()) {
        return false;
    }
    captureTimer_ = node_.createTimer(ros::Duration(1.0f / fps), &HikCameraNode::captureTimerCallback, this);
    return true;
}

bool HikCameraNode::updateConfig(CameraConfig &config)
{
    /* 曝光相关设置 */
    UPDATE_CONFIG_ASSERT(MV_CC_SetEnumValue, "ExposureAuto", config.auto_exposure ? 2 : 0);
    UPDATE_CONFIG_ASSERT(MV_CC_SetFloatValue, "ExposureTime", config.exposure_value);
    UPDATE_CONFIG_ASSERT(MV_CC_SetIntValue, "AutoExposureTimeLowerLimit", config.auto_exposure_min);
    UPDATE_CONFIG_ASSERT(MV_CC_SetIntValue, "AutoExposureTimeUpperLimit", config.auto_exposure_max);
    /* 亮度设置 */
    UPDATE_CONFIG_ASSERT(MV_CC_SetIntValue, "Brightness", config.brightness_value);
    /* 增益相关设置 */
    UPDATE_CONFIG_ASSERT(MV_CC_SetEnumValue, "GainAuto", config.auto_gain ? 2 : 0);
    UPDATE_CONFIG_ASSERT(MV_CC_SetFloatValue, "AutoGainLowerLimit", config.auto_gain_min);
    UPDATE_CONFIG_ASSERT(MV_CC_SetFloatValue, "AutoGainUpperLimit", config.auto_gain_max);
    UPDATE_CONFIG_ASSERT(MV_CC_SetFloatValue, "Gain", config.brightness_value);
    /* 黑电平 */
    UPDATE_CONFIG_ASSERT(MV_CC_SetFloatValue, "BlackLevel", config.black_level_value);
    UPDATE_CONFIG_ASSERT(MV_CC_SetBoolValue, "BlackLevelEnable", config.black_level_enable);
    /* 白平衡 */
    UPDATE_CONFIG_ASSERT(MV_CC_SetEnumValue, "BalanceRatioSelector", config.white_balance_channel);
    UPDATE_CONFIG_ASSERT(MV_CC_SetEnumValue, "BalanceWhiteAuto", config.auto_white_balance ? 1 : 0);
    UPDATE_CONFIG_ASSERT(MV_CC_SetIntValue, "BalanceRatio", config.white_balance_value);
    /* Gamma矫正 */
    UPDATE_CONFIG_ASSERT(MV_CC_SetFloatValue, "Gamma", config.gamma_value);
    UPDATE_CONFIG_ASSERT(MV_CC_SetEnumValue, "GammaSelector", config.gamma_mode);
    UPDATE_CONFIG_ASSERT(MV_CC_SetBoolValue, "GammaEnable", config.gamma_enable);
    /* 色度值矫正 */
    UPDATE_CONFIG_ASSERT(MV_CC_SetIntValue, "Hue", config.hue_value);
    UPDATE_CONFIG_ASSERT(MV_CC_SetBoolValue, "HueEnable", config.hue_enable);
    /* 饱和度调节 */
    UPDATE_CONFIG_ASSERT(MV_CC_SetIntValue, "Saturation", config.saturation_value);
    UPDATE_CONFIG_ASSERT(MV_CC_SetBoolValue, "SaturationEnable", config.saturation_enable);
    return true;
}

HikCameraNode::~HikCameraNode()
{
    MV_CC_CloseDevice(deviceHandle_);
    MV_CC_DestroyHandle(deviceHandle_);
}
}  // namespace hik_camera