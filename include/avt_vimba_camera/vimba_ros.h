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
#include <dynamic_reconfigure/SensorLevels.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <polled_camera/publication_server.h>

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
  BGR8Packed
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
    int getMaxWidth();
    int getMaxHeight();

  private:
    // true if camera is started
    bool running_;
    bool first_run_;

    // ROS Nodehandles
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    // ROS image transport
    image_transport::ImageTransport it_;
    // ROS Camera publisher
    image_transport::CameraPublisher streaming_pub_;
    // ROS Service to get images
    polled_camera::PublicationServer poll_srv_;
    // Subscriber for input trigger time
    ros::Subscriber trigger_sub_;

    // ROS params
    int num_frames_;
    std::string frame_id_;
    std::string trigger_source_;
    int trigger_source_int_;

    // ROS messages
    typedef dynamic_reconfigure::SensorLevels Levels;
    sensor_msgs::CameraInfo cam_info_;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

    // Dynamic reconfigure
    typedef avt_vimba_camera::AvtVimbaCameraConfig Config;
    typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
    ReconfigureServer reconfigure_server_;

    // Camera configuration
    Config camera_config_;

    // Vimba singleton
    VimbaSystem& vimba_system_;
    // IFrame Observer
    FrameObserver* vimba_frame_observer_ptr_;
    // The currently streaming camera
    CameraPtr vimba_camera_ptr_;
    // Current frame
    FramePtr vimba_frame_ptr_;
    // The max width
    VmbInt64_t vimba_camera_max_width_;
    // The max height
    VmbInt64_t vimba_camera_max_height_;
    // A readable value for every Vimba error code
    std::map<VmbErrorType, std::string> vimba_error_code_to_message_;

    /*************/
    /* FUNCTIONS */
    /*************/

    template<typename T> bool getFeatureValue(const std::string& feature_str, T& val);
    bool getFeatureValue(const std::string& feature_str, std::string& val);
    template<typename T> bool setFeatureValue(const std::string& feature_str, const T& val);
    bool runCommand(const std::string& command_str);

    void updateAcquisitionConfig(const Config& config, FeaturePtrVector feature_ptr_vec);
    void updateExposureConfig(const Config& config, FeaturePtrVector feature_ptr_vec);
    void updateGainConfig(const Config& config, FeaturePtrVector feature_ptr_vec);
    void updateWhiteBalanceConfig(const Config& config, FeaturePtrVector feature_ptr_vec);
    void updateImageModeConfig(const Config& config, FeaturePtrVector feature_ptr_vec);
    void updateROIConfig(Config& config, FeaturePtrVector feature_ptr_vec);
    void updateBandwidthConfig(const Config& config, FeaturePtrVector feature_ptr_vec);
    void updatePixelFormatConfig(const Config& config, FeaturePtrVector feature_ptr_vec);
    void updateGPIOConfig(const Config& config, FeaturePtrVector feature_ptr_vec);
    void updatePtpModeConfig(const Config& config, FeaturePtrVector feature_ptr_vec);
    void updateIrisConfig(const Config& config, FeaturePtrVector feature_ptr_vec);

    void start(Config& config);
    void stop();
    void frameCallback(const FramePtr vimba_frame_ptr);
    void pollCallback(polled_camera::GetPolledImage::Request& req,
                      polled_camera::GetPolledImage::Response& rsp,
                      sensor_msgs::Image& image,
                      sensor_msgs::CameraInfo& info);
    bool frameToImage(const FramePtr vimba_frame_ptr, sensor_msgs::Image& image);


    // Configure camera info
    void updateCameraInfo(const Config& config);
    // Dynamic Reconfigure Callback
    void configure(Config& newconfig, uint32_t level);

    // Init function to setup AVT Vimba SDK
    void initApi(void);
    // Translates Vimba error codes to readable error messages
    std::string errorCodeToMessage(VmbErrorType eErr);
    // Translates Vimba access mode codes to readable messages
    std::string accessModeToString(VmbAccessModeType modeType);
    // Parser of Vimba Interface Type
    std::string interfaceToString(VmbInterfaceType interfaceType);
    // Parser of trigger mode to non-readable integer
    int getTriggerModeInt(std::string mode);
    // Open the required camera
    CameraPtr openCamera(std::string id);
    // Dummy function to print all features of a camera
    void printAllCameraFeatures(CameraPtr camera);
    // Dummy function to list all cameras
    void listAvailableCameras(void);
};
}

#endif
