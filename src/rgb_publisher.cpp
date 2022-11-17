
#include <ros/ros.h>

#include <iostream>
#include <cstdio>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <camera_info_manager/camera_info_manager.h>

#include <dynamic_reconfigure/server.h>
#include "oak_driver/CameraControlConfig.h"

// Inludes common necessary includes for development using depthai library
#include <depthai/depthai.hpp>
#include <depthai_bridge/depthaiUtility.hpp>


//ToDo: Use member variables instead of global ones!
static std::shared_ptr<dai::DataInputQueue> configQueue = nullptr;
static std::shared_ptr<dai::DataInputQueue> controlQueue = nullptr;

dai::Pipeline createPipeline(const std::string& resolution, const std::string& codec, const float fps = 30, const int quality = 90){
    dai::Pipeline pipeline;

    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto videoEnc = pipeline.create<dai::node::VideoEncoder>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
    auto configIn = pipeline.create<dai::node::XLinkIn>();
    auto controlIn = pipeline.create<dai::node::XLinkIn>();


    // Configure Camera
    dai::ColorCameraProperties::SensorResolution colorResolution;
    if(resolution == "720p"){
        colorResolution = dai::ColorCameraProperties::SensorResolution::THE_720_P; 
    }else if(resolution == "1080p" ){
        colorResolution = dai::ColorCameraProperties::SensorResolution::THE_1080_P; 
    }else if(resolution == "800p" ){
        colorResolution = dai::ColorCameraProperties::SensorResolution::THE_800_P; 
    }else if(resolution == "4K" ){
        colorResolution = dai::ColorCameraProperties::SensorResolution::THE_4_K; 
    }else if(resolution == "12MP" ){
        colorResolution = dai::ColorCameraProperties::SensorResolution::THE_12_MP; 
    }else if(resolution == "13MP" ){
        colorResolution = dai::ColorCameraProperties::SensorResolution::THE_13_MP; 
    }else{
        ROS_ERROR("Invalid parameter. -> Resolution: %s, Use Default: 4K", resolution.c_str());
        //throw std::runtime_error("Invalid color camera resolution.");
        colorResolution = dai::ColorCameraProperties::SensorResolution::THE_4_K;
    }

    colorCam->setBoardSocket(dai::CameraBoardSocket::RGB);
    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(colorResolution);
    colorCam->setFps(fps);
    colorCam->setInterleaved(false);

    ROS_INFO("Resolution: %i", static_cast<int>(colorResolution));
    ROS_INFO("FPS: %i", static_cast<int>(fps));

    //ToDo: Add support for no encoding /raw

    // Configure Codec
    dai::VideoEncoderProperties::Profile encoderProfile;
    if(codec == "H264_BASELINE"){
        encoderProfile = dai::VideoEncoderProperties::Profile::H264_BASELINE; 
    }else if(codec == "H264_HIGH" ){
        encoderProfile = dai::VideoEncoderProperties::Profile::H264_HIGH; 
    }else if(codec == "H264_MAIN" ){
        encoderProfile = dai::VideoEncoderProperties::Profile::H264_MAIN; 
    }else if(codec == "H265_MAIN" ){
        encoderProfile = dai::VideoEncoderProperties::Profile::H265_MAIN; 
    }else if(codec == "MJPEG" ){
        encoderProfile = dai::VideoEncoderProperties::Profile::MJPEG; 
    }else{
        ROS_ERROR("Invalid parameter. -> Codec: %s, Use Default: MJPEG", codec.c_str());
        //throw std::runtime_error("Invalid color camera codec.");
        encoderProfile = dai::VideoEncoderProperties::Profile::MJPEG; 
    }

    videoEnc->setDefaultProfilePreset(fps, encoderProfile);
    ROS_INFO("Codec: %i", static_cast<int>(encoderProfile));

    videoEnc->setQuality(quality);
    if (encoderProfile == dai::VideoEncoderProperties::Profile::MJPEG && quality >= 100)
    {
        // Issue with preview loseless images in ROS-RVIZ!
        videoEnc->setLossless(true);
        videoEnc->setQuality(100);
    }
    ROS_INFO("Quality: %i", quality);

    videoEnc->setFrameRate(fps);

    // Configure Output
    xlinkOut->setStreamName("encoded");
    xlinkOut->setFpsLimit(fps);

    // Configure and Control Input
    configIn->setStreamName("config");
    controlIn->setStreamName("control");

    // Link plugins CAM -> VideoEncoder-> XLINK
    colorCam->video.link(videoEnc->input);
    videoEnc->bitstream.link(xlinkOut->input);
    controlIn->out.link(colorCam->inputControl);
    configIn->out.link(colorCam->inputConfig);

    return pipeline;
}


void reconfigureCallback(oak_driver::CameraControlConfig &config, uint32_t level){
    auto ctrl = dai::CameraControl();
    //ctrl.setCaptureStill(config.CaptureStill);
    ctrl.setAutoFocusMode((dai::CameraControl::AutoFocusMode)config.auto_focus_mode);
    if (config.auto_focus_mode == 0)
    {
        // Auto Focus Off -> Manual Focus
        ctrl.setManualFocus(config.manual_focus);

    }
    else if (config.auto_focus_mode < 3 || config.auto_focus_mode == 5)
    {
        // Disable Continous
        ctrl.setAutoFocusTrigger();
    }
        
    
    if (config.manual_exposure_time == 0)
    {
        // Auto Exposure
        ctrl.setAutoExposureEnable();
        ctrl.setAutoExposureLock(config.auto_exposure_lock);
        ctrl.setAutoExposureCompensation(config.auto_exposure_compensation);
        ctrl.setAntiBandingMode((dai::CameraControl::AntiBandingMode)config.anti_banding_mode);
    }
    else
    {
        // Manual Exposure
        ctrl.setManualExposure(config.manual_exposure_time, config.manual_exposure_iso);
    }

    ctrl.setAutoWhiteBalanceMode((dai::CameraControl::AutoWhiteBalanceMode)config.auto_white_balance_mode);
    if (config.auto_white_balance_mode > 0)
    {
        // Auto White Balance
        ctrl.setAutoWhiteBalanceLock(config.auto_white_balance_mode);
    }
    else
    {
        // Manual White Balance
        ctrl.setManualWhiteBalance(config.manual_white_balance);
    }


    if(controlQueue)
        controlQueue->send(ctrl);

}


int main(int argc, char** argv){

    ros::init(argc, argv, "rgb_encoded_node");
    ros::NodeHandle pnh("~");
    
    std::string tfPrefix;
    std::string camera_param_uri;
    std::string device_name;
    std::string resolution;
    std::string codec;

    float fps;
    int quality; //between 0 and 100

    int badParams = 0;

    badParams += !pnh.getParam("tf_prefix", tfPrefix);
    badParams += !pnh.getParam("camera_param_uri", camera_param_uri);

    if (badParams > 0)
    {
        throw std::runtime_error("Couldn't find one of the parameters");
    }

    // Optional Parameter:
    pnh.param<std::string>("device_name", device_name, "");
    pnh.param<std::string>("resolution", resolution, "4K");
    pnh.param<std::string>("codec", codec, "MJPEG");

    pnh.param<float>("fps", fps, 30);
    pnh.param<int>("quality", quality, 90);

    camera_info_manager::CameraInfoManager camInfoMang(pnh, tfPrefix, camera_param_uri);
    sensor_msgs::CameraInfo camInfo_ = camInfoMang.getCameraInfo();

    // Create pipeline
    dai::Pipeline pipeline = createPipeline(resolution, codec, fps, quality);

    // Connect to device and start pipeline
    std::shared_ptr<dai::Device> device = nullptr;

    if (device_name.empty())
    {
        ROS_INFO("Device Name: <empty>");
        device = std::make_shared<dai::Device>(pipeline); //First device that could be found.
    }
    else
    {
        ROS_INFO("Device Name: %s",  device_name.c_str());
        device = std::make_shared<dai::Device>(pipeline, dai::DeviceInfo(device_name));
    }

    configQueue = device->getInputQueue("config");
    controlQueue = device->getInputQueue("control");

    // Output queue will be used to get the encoded data from the output defined above
    auto videoQueue = device->getOutputQueue("encoded", 30, true);

    ROS_INFO("Start encoded video stream!" );

    auto pubImgCompr = pnh.advertise<sensor_msgs::CompressedImage>("compressed", 30);
    auto pubCameraInfo = pnh.advertise<sensor_msgs::CameraInfo>("camera_info", 30);

    // Dynamic Reconfigure Parameter
    dynamic_reconfigure::Server<oak_driver::CameraControlConfig> dynReconfServer;
    dynamic_reconfigure::Server<oak_driver::CameraControlConfig>::CallbackType dynReconfCB;

    dynReconfCB = boost::bind(&reconfigureCallback, _1, _2);
    dynReconfServer.setCallback(dynReconfCB);

    int seq = 0;
    while(pnh.ok()) {
        auto imgPacket = videoQueue->get<dai::ImgFrame>();

        sensor_msgs::CompressedImage outImageMsg;

        outImageMsg.header.stamp = dai::ros::getFrameTime(ros::Time::now(), std::chrono::steady_clock::now(), imgPacket->getTimestamp());
        outImageMsg.header.seq = seq++;
        outImageMsg.header.frame_id = tfPrefix;
        outImageMsg.format = "jpeg";    //ToDo: depends on codec parameter

        outImageMsg.data = imgPacket->getData();

        pubImgCompr.publish(outImageMsg);


        auto camInfo = camInfo_;
        camInfo.header.stamp = outImageMsg.header.stamp;
        camInfo.header.seq = outImageMsg.header.seq;
        camInfo.header.frame_id = tfPrefix;

        pubCameraInfo.publish(camInfo);

        ros::spinOnce();

    }


    return 0;
}

