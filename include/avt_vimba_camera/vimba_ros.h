/// Copyright (c) 2014,
/// Systems, Robotics and Vision Group
/// University of the Balearican Islands
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///     * Redistributions of source code must retain the above copyright
///       notice, this list of conditions and the following disclaimer.
///     * Redistributions in binary form must reproduce the above copyright
///       notice, this list of conditions and the following disclaimer in the
///       documentation and/or other materials provided with the distribution.
///     * All advertising materials mentioning features or use of this software
///       must display the following acknowledgement:
///       This product includes software developed by
///       Systems, Robotics and Vision Group, Univ. of the Balearican Islands
///     * Neither the name of Systems, Robotics and Vision Group, University of
///       the Balearican Islands nor the names of its contributors may be used
///       to endorse or promote products derived from this software without
///       specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
/// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
/// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef VIMBA_ROS_H
#define VIMBA_ROS_H

#include <VimbaCPP/Include/VimbaCPP.h>

#include <avt_vimba_camera/AvtVimbaCameraConfig.h>
#include <avt_vimba_camera/frame_observer.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>

#include <string>
#include <map>

using AVT::VmbAPI::VimbaSystem;
using AVT::VmbAPI::CameraPtr;
using AVT::VmbAPI::FramePtr;
using AVT::VmbAPI::IFrameObserverPtr;

namespace avt_vimba_camera {

enum FrameStartTriggerMode {
  Freerun,
  FixedRate,
  Software,
  SyncIn1,
  SyncIn2,
  SyncIn3,
  SyncIn4
};

enum AcquisitionMode {
  Continuous,
  SingleFrame,
  MultiFrame,
  Recorder
};

enum PixelFormatMode {
  Mono8,
  Mono12,
  Mono12Packed,
  BayerRG8,
  BayerRG12Packed,
  BayerRG12,
  RGB8Packed,
  BGR8Packed,
  YUV411Packed,
  YUV422Packed,
  YUV444Packed
};

enum AutoSettingMode {
  Off,
  Once,
  Auto
};

class VimbaROS {
  public:
    VimbaROS(ros::NodeHandle nh, ros::NodeHandle nhp);
    int getWidth();
    int getHeight();
    VmbPixelFormatType getPixelFormat();

  private:
    // true if camera is started
    bool running_;

    // ROS Nodehandles
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    // ROS image transport
    image_transport::ImageTransport it_;
    // ROS Camera publisher
    image_transport::CameraPublisher streaming_pub_;
    // ROS Service to set camera info for calibration
    ros::ServiceServer set_camera_info_srv_;
    // Subscriber for input trigger time
    ros::Subscriber trigger_sub_;

    // ROS params
    int num_frames_;

    // ROS messages
    sensor_msgs::Image img_;
    sensor_msgs::CameraInfo cam_info_;

    // Dynamic reconfigure
    typedef avt_vimba_camera::AvtVimbaCameraConfig Config;
    typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
    ReconfigureServer reconfigure_server_;

    // Camera configuration
    Config camera_config_;
    FrameStartTriggerMode trigger_mode_;
    AcquisitionMode acquisition_mode_;
    PixelFormatMode pixel_format_;

    // Vimba singleton
    VimbaSystem& vimba_system_;
    // The currently streaming camera
    FrameObserver* vimba_frame_observer_ptr_;
    // IFrame Observer
    CameraPtr vimba_camera_ptr_;
    // Current frame
    FramePtr vimba_frame_ptr_;
    // The current pixel format
    VmbInt64_t vimba_pixel_format_;
    // The current width
    VmbInt64_t vimba_width_;
    // The current height
    VmbInt64_t vimba_height_;
    // A readable value for every Vimba error code
    std::map<VmbErrorType, std::string> vimba_error_code_to_message_;

    /*************/
    /* FUNCTIONS */
    /*************/
    //! Must be used before calling start() in a non-triggered mode.
    //void setFrameCallback(boost::function<void (tPvFrame*)> callback);
    //! Start capture.
    //void start(FrameStartTriggerMode = Freerun, AcquisitionMode = Continuous);
    //! Stop capture.
    //void stop();
    //! Capture a single frame from the camera. Must be called after
    //! start(Software Triggered).
    //tPvFrame* grab(unsigned long timeout_ms = PVINFINITE);

    void setExposure(unsigned int val, AutoSettingMode auto_mode);
    void setGain(unsigned int val, AutoSettingMode auto_mode);
    void setWhiteBalance(unsigned int blue, unsigned int red, AutoSettingMode auto_mode);
    void setRoi(unsigned int x, unsigned int y,
                unsigned int width, unsigned int height);
    void setRoiToWholeFrame();
    void setBinning(unsigned int binning_x = 1, unsigned int binning_y = 1);
    void setDecimation(unsigned int dec_x = 1, unsigned int dec_y = 1);


    template<typename T> bool getFeatureValue(std::string feature_str, T val);
    template<typename T> bool setFeatureValue(std::string feature_str, T val);

    void updateTriggerConfig(const Config& config, FeaturePtrVector feature_ptr_vec);
    void updateExposureConfig(const Config& config, FeaturePtrVector feature_ptr_vec);
    void updateGainConfig(const Config& config, FeaturePtrVector feature_ptr_vec);
    void updateWhiteBalanceConfig(const Config& config, FeaturePtrVector feature_ptr_vec);
    void updateImageModeConfig(const Config& config, FeaturePtrVector feature_ptr_vec);
    void updateROIConfig(const Config& config, FeaturePtrVector feature_ptr_vec);
    void updateBandwidthConfig(const Config& config, FeaturePtrVector feature_ptr_vec);
    void updatePixelFormatConfig(const Config& config, FeaturePtrVector feature_ptr_vec);


    void start();
    void stop();
    void frameCallback(const FramePtr vimba_frame_ptr);
    bool frameToImage(const FramePtr vimba_frame_ptr, sensor_msgs::Image& image);


    // Service callback for setting calibration.
    bool setCameraInfo(sensor_msgs::SetCameraInfo::Request& req,
                       sensor_msgs::SetCameraInfo::Response& rsp);
    // Dynamic Reconfigure Callback
    void configure(const Config& config, uint32_t level);

    // Init function to setup AVT Vimba SDK
    void initApi(void);
    // Translates Vimba error codes to readable error messages
    std::string errorCodeToMessage(VmbErrorType eErr);
    // Translates Vimba access mode codes to readable messages
    std::string accessModeToString(VmbAccessModeType modeType);
    // Parser of Vimba Interface Type
    std::string interfaceToString(VmbInterfaceType interfaceType);
    // Open the required camera
    CameraPtr openCamera(std::string id);
    // Dummy function to print all features of a camera
    void printAllCameraFeatures(CameraPtr camera);
    // Dummy function to list all cameras
    void listAvailableCameras(void);
};
}

#endif
