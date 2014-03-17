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

#include <avt_vimba_camera/vimba_ros.h>

#include <ros/console.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

#include <boost/lexical_cast.hpp>
#include <sstream>

namespace avt_vimba_camera {

static const char* AutoMode[] = {"Off", "Once", "Continuous"};
static const char* TriggerMode[] = {
  "Freerun",
  "FixedRate",
  "Software",
  "SyncIn1",
  "SyncIn2",
  "SyncIn3",
  "SyncIn4" };
static const char* AcquisitionMode[] = {
  "Continuous",
  "SingleFrame",
  "MultiFrame",
  "Recorder"};
static const char* PixelFormatMode[] = {
  "Mono8",
  "Mono12",
  "Mono12Packed",
  "BayerRG8",
  "BayerRG12Packed",
  "BayerRG12",
  "RGB8Packed",
  "BGR8Packed"};
static const char* BalanceRatioMode[] = {"Red", "Blue"};
static const char* FeatureDataType[] = {"Unknown", "int", "float", "enum", "string", "bool"};


VimbaROS::VimbaROS(ros::NodeHandle nh, ros::NodeHandle nhp)
:vimba_system_(VimbaSystem::GetInstance()), it_(nhp), nh_(nh), nhp_(nhp) {

  // Vimba startup & list all available cameras
  initApi();

  cinfo_ = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(nhp_));

  // Init global variables
  running_ = false;
  first_run_ = true;

  // Service call for setting calibration.
  //set_camera_info_srv_ = nh_.advertiseService("set_camera_info",
  //                                            &VimbaROS::setCameraInfo,
  //                                            this);

  // Start dynamic_reconfigure & run configure()
  reconfigure_server_.setCallback(boost::bind(&VimbaROS::configure,
                                              this,
                                              _1,
                                              _2));
}

void VimbaROS::start(Config& config) {
  if (running_) return;

  std::string ip_str   = config.ip_address;
  std::string guid_str = config.guid;

  // Determine which camera to use. Try IP first
  if (!ip_str.empty()) {
    ROS_INFO_STREAM("Trying to open camera by IP: " << ip_str);
    vimba_camera_ptr_ = openCamera(ip_str);
    // If both guid and IP are available, open by IP and check guid
    if (!guid_str.empty()) {
      std::string cam_guid_str;
      vimba_camera_ptr_->GetSerialNumber(cam_guid_str);
      assert(cam_guid_str == guid_str);
      ROS_INFO_STREAM("GUID " << cam_guid_str << " matches for camera with IP: " << ip_str);
    }
  } else if (!guid_str.empty()) {
    // Only guid available
    ROS_INFO_STREAM("Trying to open camera by ID: " << guid_str);
    vimba_camera_ptr_ = openCamera(guid_str);
  }

  // Get all cam properties we need for initialization
  getFeatureValue("WidthMax",vimba_camera_max_width_);
  getFeatureValue("HeightMax",vimba_camera_max_height_);

  // Set all camera configurations
  FeaturePtrVector feature_ptr_vec;
  vimba_camera_ptr_->GetFeatures(feature_ptr_vec);
  updateAcquisitionConfig(config, feature_ptr_vec);
  updateExposureConfig(config, feature_ptr_vec);
  updateGainConfig(config, feature_ptr_vec);
  updateWhiteBalanceConfig(config, feature_ptr_vec);
  updateImageModeConfig(config, feature_ptr_vec);
  updateROIConfig(config, feature_ptr_vec);
  updateBandwidthConfig(config, feature_ptr_vec);
  updatePixelFormatConfig(config, feature_ptr_vec);
  updateCameraInfo(config);

  /////////////////////////////////////////////////////////////////////////////
  getFeatureValue("TriggerSource", trigger_mode_);
  ROS_INFO_STREAM("[AVT_Vimba_ROS]: Trigger mode is " << trigger_mode_);

  trigger_mode_int_ = getTriggerModeInt(trigger_mode_);

  if (trigger_mode_int_ == Freerun || trigger_mode_int_ == FixedRate || trigger_mode_int_ == SyncIn1) {
    // Create a frame observer for this camera
    vimba_frame_observer_ptr_ = new FrameObserver(vimba_camera_ptr_,
                    boost::bind(&VimbaROS::frameCallback, this, _1));

    // Setup the image publisher before the streaming
    streaming_pub_ = it_.advertiseCamera("image_raw", 1);

    // Start streaming
    VmbErrorType err =
      vimba_camera_ptr_->StartContinuousImageAcquisition(1,//num_frames_,
                                IFrameObserverPtr(vimba_frame_observer_ptr_));
    if (VmbErrorSuccess == err){
      ROS_INFO_STREAM("[AVT_Vimba_ROS]: StartContinuousImageAcquisition");
    } else {
      ROS_ERROR_STREAM("[AVT_Vimba_ROS]: Could not StartContinuousImageAcquisition. "
        << "\n Error: " << errorCodeToMessage(err));
    }
  } else if (trigger_mode_int_ == Software) {
    vimba_camera_ptr_->Close();
    poll_srv_ = polled_camera::advertise(nhp_, "request_image", &VimbaROS::pollCallback, this);
  } else {
    ROS_ERROR_STREAM("Trigger mode " <<
                       TriggerMode[trigger_mode_int_] <<
                       " not implemented.");
  }
}

void VimbaROS::stop() {
  if (!running_) return;

  vimba_camera_ptr_->Close();  // Must stop camera before streaming_pub_.
  poll_srv_.shutdown();
  //trigger_sub_.shutdown();
  streaming_pub_.shutdown();

  running_ = false;
}

void VimbaROS::pollCallback(polled_camera::GetPolledImage::Request& req,
                            polled_camera::GetPolledImage::Response& rsp,
                            sensor_msgs::Image& image,
                            sensor_msgs::CameraInfo& info) {
  if (trigger_mode_int_ != Software) {
    rsp.success = false;
    rsp.status_message = "Camera is not in software triggered mode";
    return;
  }

  // open the camera
  vimba_camera_ptr_ = openCamera(camera_config_.guid);

  // configure it
  configure(camera_config_,0);

  ROS_INFO_STREAM("POLL CALLBACK CALLED");


  FeaturePtrVector feature_ptr_vec;
  VmbErrorType err = vimba_camera_ptr_->GetFeatures(feature_ptr_vec);
  if (err == VmbErrorSuccess) {
    // Region of interest configuration
    // Make sure ROI fits in image
    if (req.roi.x_offset || req.roi.y_offset || req.roi.width || req.roi.height) {
      unsigned int left_x   = req.roi.x_offset / req.binning_x;
      unsigned int top_y    = req.roi.y_offset / req.binning_y;
      unsigned int right_x  = (req.roi.x_offset + req.roi.width  + req.binning_x - 1) / req.binning_x;
      unsigned int bottom_y = (req.roi.y_offset + req.roi.height + req.binning_y - 1) / req.binning_y;
      unsigned int width    = right_x - left_x;
      unsigned int height   = bottom_y - top_y;
      setFeatureValue("OffsetX", static_cast<VmbInt64_t>(left_x));
      setFeatureValue("OffsetY", static_cast<VmbInt64_t>(top_y));
      setFeatureValue("Width",   static_cast<VmbInt64_t>(width));
      setFeatureValue("Height",  static_cast<VmbInt64_t>(height));
    } else {
      setFeatureValue("OffsetX", static_cast<VmbInt64_t>(0));
      setFeatureValue("OffsetY", static_cast<VmbInt64_t>(0));
      setFeatureValue("Width",   static_cast<VmbInt64_t>(vimba_camera_max_width_));
      setFeatureValue("Height",  static_cast<VmbInt64_t>(vimba_camera_max_height_));
    }
  } else {
    ROS_ERROR_STREAM("[AVT_Vimba_ROS]: Could not GetFeatures. "
    << "\n Error: " << errorCodeToMessage(err));
  }

  ROS_INFO_STREAM("POLL CALLBACK CALLED: all features set");

  // Zero duration means "no timeout"
  VmbUint32_t VmbUint32_inf = 2e4-1;
  unsigned long timeout = req.timeout.isZero() ? VmbUint32_inf : req.timeout.toNSec() / 1e6;
  FramePtr frame;
  //runCommand("TriggerSoftware");
  err = vimba_camera_ptr_->AcquireSingleImage(frame, timeout);
  ROS_INFO_STREAM("POLL CALLBACK CALLED: acquire single image");
  if (err == VmbErrorSuccess) {
    if (!frame) {
      rsp.success = false;
      rsp.status_message = "Failed to capture frame, may have timed out";
      return;
    } else {
      //err = vimba_camera_ptr_->QueueFrame(frame);
      if (err == VmbErrorSuccess) {
        info = cinfo_->getCameraInfo();
        if (!frameToImage(frame, image)) {
          rsp.success = false;
          rsp.status_message = "Captured frame but failed to process it";
          return;
        }
        // do_rectify should be preserved from request
        info.roi.do_rectify = req.roi.do_rectify;
        rsp.success = true;
        ROS_INFO_STREAM("POLL CALLBACK CALLED: DONE!");
      }
    }
  }

  if (err != VmbErrorSuccess){
    ROS_ERROR_STREAM("[AVT_Vimba_ROS]: Could not AcquireSingleImage. "
    << "\n Error: " << errorCodeToMessage(err));
    rsp.success = false;
    rsp.status_message = "AVT_Vimba_ROS error";
    return;
  }

  // close the camera
  vimba_camera_ptr_->Close();
}

void VimbaROS::frameCallback(const FramePtr vimba_frame_ptr) {

  sensor_msgs::Image img;

  vimba_frame_ptr_ = vimba_frame_ptr;

  /// @todo Better trigger timestamp matching
  // if ( trigger_mode_ == SyncIn1  && !trig_time_.isZero() ) {

  img.header.stamp = cam_info_.header.stamp = ros::Time::now();
  //VmbUint64_t timestamp;
  //vimba_frame_ptr->GetTimestamp(timestamp);
  //ROS_INFO_STREAM("[AVT_Vimba_ROS]: NEW_FRAME Timestamp: " << timestamp);

  img.header.frame_id = frame_id_.c_str();

  if(streaming_pub_.getNumSubscribers() > 0){
    if (frameToImage(vimba_frame_ptr, img)){
      streaming_pub_.publish(img, cam_info_); // add timestamp
    }
  }
  // Queue the frame so that we can receive a new one.
  vimba_camera_ptr_->QueueFrame(vimba_frame_ptr);
}

bool VimbaROS::frameToImage(const FramePtr vimba_frame_ptr,
                            sensor_msgs::Image& image) {
  VmbPixelFormatType pixel_format;
  VmbUint32_t width  = camera_config_.width;
  VmbUint32_t height = camera_config_.height;

  vimba_frame_ptr->GetPixelFormat(pixel_format);

  // NOTE: YUV and ARGB formats not supported
  std::string encoding;
  if      (pixel_format == VmbPixelFormatMono8          ) encoding = sensor_msgs::image_encodings::MONO8;
  else if (pixel_format == VmbPixelFormatMono10         ) encoding = sensor_msgs::image_encodings::MONO16;
  else if (pixel_format == VmbPixelFormatMono12         ) encoding = sensor_msgs::image_encodings::MONO16;
  else if (pixel_format == VmbPixelFormatMono12Packed   ) encoding = sensor_msgs::image_encodings::MONO16;
  else if (pixel_format == VmbPixelFormatMono14         ) encoding = sensor_msgs::image_encodings::MONO16;
  else if (pixel_format == VmbPixelFormatMono16         ) encoding = sensor_msgs::image_encodings::MONO16;
  else if (pixel_format == VmbPixelFormatBayerGR8       ) encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
  else if (pixel_format == VmbPixelFormatBayerRG8       ) encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
  else if (pixel_format == VmbPixelFormatBayerGB8       ) encoding = sensor_msgs::image_encodings::BAYER_GBRG8;
  else if (pixel_format == VmbPixelFormatBayerBG8       ) encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
  else if (pixel_format == VmbPixelFormatBayerGR10      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
  else if (pixel_format == VmbPixelFormatBayerRG10      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
  else if (pixel_format == VmbPixelFormatBayerGB10      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
  else if (pixel_format == VmbPixelFormatBayerBG10      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
  else if (pixel_format == VmbPixelFormatBayerGR12      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
  else if (pixel_format == VmbPixelFormatBayerRG12      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
  else if (pixel_format == VmbPixelFormatBayerGB12      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
  else if (pixel_format == VmbPixelFormatBayerBG12      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
  else if (pixel_format == VmbPixelFormatBayerGR12Packed) encoding = sensor_msgs::image_encodings::TYPE_32SC4;
  else if (pixel_format == VmbPixelFormatBayerRG12Packed) encoding = sensor_msgs::image_encodings::TYPE_32SC4;
  else if (pixel_format == VmbPixelFormatBayerGB12Packed) encoding = sensor_msgs::image_encodings::TYPE_32SC4;
  else if (pixel_format == VmbPixelFormatBayerBG12Packed) encoding = sensor_msgs::image_encodings::TYPE_32SC4;
  else if (pixel_format == VmbPixelFormatBayerGR16      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
  else if (pixel_format == VmbPixelFormatBayerRG16      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
  else if (pixel_format == VmbPixelFormatBayerGB16      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
  else if (pixel_format == VmbPixelFormatBayerBG16      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
  else if (pixel_format == VmbPixelFormatRgb8           ) encoding = sensor_msgs::image_encodings::RGB8;
  else if (pixel_format == VmbPixelFormatBgr8           ) encoding = sensor_msgs::image_encodings::BGR8;
  else if (pixel_format == VmbPixelFormatRgba8          ) encoding = sensor_msgs::image_encodings::RGBA8;
  else if (pixel_format == VmbPixelFormatBgra8          ) encoding = sensor_msgs::image_encodings::BGRA8;
  else if (pixel_format == VmbPixelFormatRgb12          ) encoding = sensor_msgs::image_encodings::TYPE_16UC3;
  else if (pixel_format == VmbPixelFormatRgb16          ) encoding = sensor_msgs::image_encodings::TYPE_16UC3;
  else
    ROS_WARN("Received frame with unsupported pixel format %d", pixel_format);

  if (encoding == "") return false;

  VmbUint32_t nSize;
  vimba_frame_ptr->GetImageSize(nSize);
  VmbUint32_t step = nSize / height;

  VmbUchar_t *buffer_ptr;
  VmbErrorType err = vimba_frame_ptr->GetImage(buffer_ptr);
  bool res = false;

  if ( VmbErrorSuccess == err ) {
    res = sensor_msgs::fillImage(image,
                                 encoding,
                                 height,
                                 width,
                                 step,
                                 buffer_ptr);
  } else {
    ROS_ERROR_STREAM("[AVT_Vimba_ROS]: Could not GetImage. "
      << "\n Error: " << errorCodeToMessage(err));
  }
  return res;
}

void VimbaROS::updateCameraInfo(const Config& config) {
  cam_info_.header.frame_id = config.frame_id;

  // Set the operational parameters in CameraInfo (binning, ROI)
  cam_info_.height    = config.height;
  cam_info_.width     = config.width;
  cam_info_.binning_x = config.binning_x;
  cam_info_.binning_y = config.binning_y;
  // ROI in CameraInfo is in unbinned coordinates, need to scale up
  cam_info_.roi.x_offset = config.roi_offset_x;
  cam_info_.roi.y_offset = config.roi_offset_y;
  cam_info_.roi.height   = config.roi_height;
  cam_info_.roi.width    = config.roi_width;

  // set the new URL and load CameraInfo (if any) from it
  if(config.camera_info_url != camera_config_.camera_info_url || first_run_) {
    cinfo_->setCameraName(config.frame_id);
    if (cinfo_->validateURL(config.camera_info_url)) {
      cinfo_->loadCameraInfo(config.camera_info_url);
      cam_info_ = cinfo_->getCameraInfo();
    } else {
      ROS_WARN_STREAM("Camera info URL not valid: " << config.camera_info_url);
    }
  }

  bool roiMatchesCalibration = (cam_info_.height == config.roi_height
                              && cam_info_.width == config.roi_width);
  bool resolutionMatchesCalibration = (cam_info_.width == config.width
                                   && cam_info_.height == config.height);

  cam_info_.roi.do_rectify = roiMatchesCalibration || resolutionMatchesCalibration;
}

/** Dynamic reconfigure callback
*
*  Called immediately when callback first defined. Called again
*  when dynamic reconfigure starts or changes a parameter value.
*
*  @param newconfig new Config values
*  @param level bit-wise OR of reconfiguration levels for all
*               changed parameters (0xffffffff on initial call)
**/
void VimbaROS::configure(Config& newconfig, uint32_t level) {
  // TODO(m): Do not run concurrently with poll().  Tell it to stop running,
  // and wait on the lock until it does.
  //boost::mutex::scoped_lock lock(mutex_);

  try {
    ROS_INFO("dynamic reconfigure level 0x%x", level);


    // resolve frame ID using tf_prefix parameter
    if (newconfig.frame_id == "") {
      newconfig.frame_id = "camera";
    }

    if (running_ && (level & Levels::RECONFIGURE_CLOSE)) {
      // The device has to be closed to change these params
      stop();
      start(newconfig);
    } else if (running_ && (level & Levels::RECONFIGURE_STOP)) {
      // The device has to stop streaming to change these params
      stop();
      start(newconfig);
    } else if (!running_) {
      camera_config_ = newconfig;
      start(newconfig);
      first_run_ = false;
    } else if (running_) {
      // only change those params that can be changed while running
      FeaturePtrVector feature_ptr_vec;
      vimba_camera_ptr_->GetFeatures(feature_ptr_vec);
      updateExposureConfig(newconfig, feature_ptr_vec);
      updateGainConfig(newconfig, feature_ptr_vec);
      updateWhiteBalanceConfig(newconfig, feature_ptr_vec);
      updateImageModeConfig(newconfig, feature_ptr_vec);
      updateROIConfig(newconfig, feature_ptr_vec);
      updateBandwidthConfig(newconfig, feature_ptr_vec);
      updateGPIOConfig(newconfig, feature_ptr_vec);
      updateCameraInfo(newconfig);
    }
    camera_config_ = newconfig;
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Error reconfiguring avt_vimba_camera node : " << e.what());
  }
}

/** Change the Trigger configuration */
void VimbaROS::updateAcquisitionConfig(const Config& config,
                                   FeaturePtrVector feature_ptr_vec) {
  bool changed = false;
  if (config.acquisition_mode != camera_config_.acquisition_mode || first_run_) {
    changed = true;
    setFeatureValue("AcquisitionMode", config.acquisition_mode.c_str());
  }
  if (config.acquisition_rate != camera_config_.acquisition_rate || first_run_) {
    changed = true;
    setFeatureValue("AcquisitionFrameRateAbs",
                    static_cast<float>(config.acquisition_rate));
  }
  if (config.trigger_mode != camera_config_.trigger_mode || first_run_) {
    changed = true;
    setFeatureValue("TriggerSource", config.trigger_mode.c_str());
  }
  if (config.trigger_activation != camera_config_.trigger_activation || first_run_) {
    changed = true;
    setFeatureValue("TriggerActivation", config.trigger_activation.c_str());
  }
  if (config.trigger_delay != camera_config_.trigger_delay || first_run_) {
    changed = true;
    setFeatureValue("TriggerDelayAbs", config.trigger_delay);
  }
  if(changed){
    ROS_INFO_STREAM("New Trigger config: "
      << "\n\tAcquisitionMode         : " << config.acquisition_mode   << " was " << camera_config_.acquisition_mode
      << "\n\tAcquisitionFrameRateAbs : " << config.acquisition_rate   << " was " << camera_config_.acquisition_rate
      << "\n\tTriggerSource           : " << config.trigger_mode       << " was " << camera_config_.trigger_mode
      << "\n\tTriggerActivation       : " << config.trigger_activation << " was " << camera_config_.trigger_activation
      << "\n\tTriggerDelayAbs            : " << config.trigger_delay      << " was " << camera_config_.trigger_delay);
  }
}

/** Change the Exposure configuration */
void VimbaROS::updateExposureConfig(const Config& config,
                                    FeaturePtrVector feature_ptr_vec) {
  bool changed = false;
  if (config.exposure != camera_config_.exposure || first_run_) {
    changed = true;
    setFeatureValue("ExposureTimeAbs", static_cast<float>(config.exposure));
  }
  if (config.exposure_auto != camera_config_.exposure_auto || first_run_) {
    changed = true;
    setFeatureValue("ExposureAuto", config.exposure_auto.c_str());
  }
  if (config.exposure_auto_tol != camera_config_.exposure_auto_tol || first_run_) {
    changed = true;
    setFeatureValue("ExposureAutoAdjustTol",
                    static_cast<VmbInt64_t>(config.exposure_auto_tol));
  }
  if (config.exposure_auto_max != camera_config_.exposure_auto_max || first_run_) {
    changed = true;
    setFeatureValue("ExposureAutoMax",
                    static_cast<VmbInt64_t>(config.exposure_auto_max));
  }
  if (config.exposure_auto_min != camera_config_.exposure_auto_min || first_run_) {
    changed = true;
    setFeatureValue("ExposureAutoMin",
                    static_cast<VmbInt64_t>(config.exposure_auto_min));
  }
  if (config.exposure_auto_outliers != camera_config_.exposure_auto_outliers || first_run_) {
    changed = true;
    setFeatureValue("ExposureAutoOutliers",
                    static_cast<VmbInt64_t>(config.exposure_auto_outliers));
  }
  if (config.exposure_auto_rate != camera_config_.exposure_auto_rate || first_run_) {
    changed = true;
    setFeatureValue("ExposureAutoRate",
                    static_cast<VmbInt64_t>(config.exposure_auto_rate));
  }
  if (config.exposure_auto_target != camera_config_.exposure_auto_target || first_run_) {
    changed = true;
    setFeatureValue("ExposureAutoTarget",
                    static_cast<VmbInt64_t>(config.exposure_auto_target));
  }
  if(changed){
    ROS_INFO_STREAM("New Exposure config: "
      << "\n\tExposureTimeAbs       : " << config.exposure               << " was " << camera_config_.exposure
      << "\n\tExposureAuto          : " << config.exposure_auto          << " was " << camera_config_.exposure_auto
      << "\n\tExposureAutoAdjustTol : " << config.exposure_auto_tol      << " was " << camera_config_.exposure_auto_tol
      << "\n\tExposureAutoMax       : " << config.exposure_auto_max      << " was " << camera_config_.exposure_auto_max
      << "\n\tExposureAutoMin       : " << config.exposure_auto_min      << " was " << camera_config_.exposure_auto_min
      << "\n\tExposureAutoOutliers  : " << config.exposure_auto_outliers << " was " << camera_config_.exposure_auto_outliers
      << "\n\tExposureAutoRate      : " << config.exposure_auto_rate     << " was " << camera_config_.exposure_auto_rate
      << "\n\tExposureAutoTarget    : " << config.exposure_auto_target   << " was " << camera_config_.exposure_auto_target);
  }
}

/** Change the Gain configuration */
void VimbaROS::updateGainConfig(const Config& config,
                                FeaturePtrVector feature_ptr_vec) {
  bool changed = false;
  if (config.gain != camera_config_.gain || first_run_) {
    changed = true;
    setFeatureValue("Gain", static_cast<float>(config.gain));
  }
  if (config.gain_auto != camera_config_.gain_auto || first_run_) {
    changed = true;
    setFeatureValue("GainAuto", config.gain_auto.c_str());
  }
  if (config.gain_auto_tol != camera_config_.gain_auto_tol || first_run_) {
    changed = true;
    setFeatureValue("GainAutoAdjustTol",
                    static_cast<VmbInt64_t>(config.gain_auto_tol));
  }
  if (config.gain_auto_max != camera_config_.gain_auto_max || first_run_) {
    changed = true;
    setFeatureValue("GainAutoMax", static_cast<float>(config.gain_auto_max));
  }
  if (config.gain_auto_min != camera_config_.gain_auto_min || first_run_) {
    changed = true;
    setFeatureValue("GainAutoMin",
                    static_cast<VmbInt64_t>(config.gain_auto_min));
  }
  if (config.gain_auto_outliers != camera_config_.gain_auto_outliers || first_run_) {
    changed = true;
    setFeatureValue("GainAutoMin",
                    static_cast<VmbInt64_t>(config.gain_auto_outliers));
  }
  if (config.gain_auto_rate != camera_config_.gain_auto_rate || first_run_) {
    changed = true;
    setFeatureValue("GainAutoOutliers",
                    static_cast<VmbInt64_t>(config.gain_auto_rate));
  }
  if (config.gain_auto_target != camera_config_.gain_auto_target || first_run_) {
    changed = true;
    setFeatureValue("GainAutoRate", static_cast<VmbInt64_t>(config.gain_auto_target));
  }
  if(changed){
    ROS_INFO_STREAM("New Exposure config: "
      << "\n\tGain              : " << config.gain               << " was " << camera_config_.gain
      << "\n\tGainAuto          : " << config.gain_auto          << " was " << camera_config_.gain_auto
      << "\n\tGainAutoAdjustTol : " << config.gain_auto_tol      << " was " << camera_config_.gain_auto_tol
      << "\n\tGainAutoMax       : " << config.gain_auto_max      << " was " << camera_config_.gain_auto_max
      << "\n\tGainAutoMin       : " << config.gain_auto_min      << " was " << camera_config_.gain_auto_min
      << "\n\tGainAutoOutliers  : " << config.gain_auto_outliers << " was " << camera_config_.gain_auto_outliers
      << "\n\tGainAutoRate      : " << config.gain_auto_rate     << " was " << camera_config_.gain_auto_rate
      << "\n\tGainAutoTarget    : " << config.gain_auto_target   << " was " << camera_config_.gain_auto_target);
  }
}

/** Change the White Balance configuration */
void VimbaROS::updateWhiteBalanceConfig(const Config& config,
                                        FeaturePtrVector feature_ptr_vec){
  bool changed = false;
  if (config.balance_ratio_abs != camera_config_.balance_ratio_abs || first_run_) {
    changed = true;
    setFeatureValue("BalanceRatioAbs", static_cast<float>(config.balance_ratio_abs));
  }
  if (config.balance_ratio_selector != camera_config_.balance_ratio_selector || first_run_) {
    changed = true;
    setFeatureValue("BalanceRatioSelector", config.balance_ratio_selector.c_str());
  }
  if (config.whitebalance_auto != camera_config_.whitebalance_auto || first_run_) {
    changed = true;
    setFeatureValue("BalanceWhiteAuto", config.whitebalance_auto.c_str());
  }
  if (config.whitebalance_auto_tol != camera_config_.whitebalance_auto_tol || first_run_) {
    changed = true;
    setFeatureValue("BalanceWhiteAutoAdjustTol", static_cast<VmbInt64_t>(config.whitebalance_auto_tol));
  }
  if (config.whitebalance_auto_rate != camera_config_.whitebalance_auto_rate || first_run_) {
    changed = true;
    setFeatureValue("BalanceWhiteAutoRate", static_cast<VmbInt64_t>(config.whitebalance_auto_rate));
  }
  if(changed){
    ROS_INFO_STREAM("New ROI config: "
      << "\n\tBalanceRatioAbs           : " << config.balance_ratio_abs      << " was " << camera_config_.balance_ratio_abs
      << "\n\tBalanceRatioSelector      : " << config.balance_ratio_selector << " was " << camera_config_.balance_ratio_selector
      << "\n\tBalanceWhiteAuto          : " << config.whitebalance_auto      << " was " << camera_config_.whitebalance_auto
      << "\n\tBalanceWhiteAutoAdjustTol : " << config.whitebalance_auto_tol  << " was " << camera_config_.whitebalance_auto_tol
      << "\n\tBalanceWhiteAutoRate      : " << config.whitebalance_auto_rate << " was " << camera_config_.whitebalance_auto_rate);
  }
}

/** Change the Binning and Decimation configuration */
void VimbaROS::updateImageModeConfig(const Config& config,
                                     FeaturePtrVector feature_ptr_vec) {
  bool changed = false;
  if (config.decimation_x != camera_config_.decimation_x || first_run_) {
    changed = true;
    setFeatureValue("DecimationHorizontal",
                    static_cast<VmbInt64_t>(config.decimation_x));
  }
  if (config.decimation_y != camera_config_.decimation_y || first_run_) {
    changed = true;
    setFeatureValue("DecimationVertical", static_cast<VmbInt64_t>(config.decimation_y));
  }
  if (config.binning_x != camera_config_.binning_x || first_run_) {
    changed = true;
    setFeatureValue("BinningHorizontal", static_cast<VmbInt64_t>(config.binning_x));
  }
  if (config.binning_y != camera_config_.binning_y || first_run_) {
    changed = true;
    setFeatureValue("BinningVertical", static_cast<VmbInt64_t>(config.binning_y));
  }
  if(changed){
    ROS_INFO_STREAM("New Image config: "
      << "\n\tDecimationHorizontal : " << config.decimation_x << " was " << camera_config_.decimation_x
      << "\n\tDecimationVertical   : " << config.decimation_y << " was " << camera_config_.decimation_y
      << "\n\tBinningHorizontal    : " << config.binning_x    << " was " << camera_config_.binning_x
      << "\n\tBinningVertical      : " << config.binning_y    << " was " << camera_config_.binning_y);
  }
}

/** Change the ROI configuration */
void VimbaROS::updateROIConfig(Config& config,
                               FeaturePtrVector feature_ptr_vec) {
  bool changed = false;

  // Region of interest configuration
  // Make sure ROI fits in image
  config.width        = std::min(config.width,  (int)vimba_camera_max_width_);
  config.height       = std::min(config.height, (int)vimba_camera_max_height_);
  config.roi_offset_x = std::min(config.roi_offset_x, config.width - 1);
  config.roi_offset_y = std::min(config.roi_offset_y, config.height - 1);
  config.roi_width    = std::min(config.roi_width,  config.width  - config.roi_offset_x);
  config.roi_height   = std::min(config.roi_height, config.height - config.roi_offset_y);
  // If width or height is 0, set it as large as possible
  int width  = config.roi_width  ? config.roi_width  : vimba_camera_max_width_  - config.roi_offset_x;
  int height = config.roi_height ? config.roi_height : vimba_camera_max_height_ - config.roi_offset_y;

  // Adjust full-res ROI to binning ROI
  /// @todo Replicating logic from polledCallback
  int offset_x = config.roi_offset_x / config.binning_x;
  int offset_y = config.roi_offset_y / config.binning_y;
  unsigned int right_x  = (config.roi_offset_x + width  + config.binning_x - 1) / config.binning_x;
  unsigned int bottom_y = (config.roi_offset_y + height + config.binning_y - 1) / config.binning_y;
  // Rounding up is bad when at max resolution which is not divisible by the amount of binning
  right_x  = std::min(right_x,  (unsigned)(config.width  / config.binning_x));
  bottom_y = std::min(bottom_y, (unsigned)(config.height / config.binning_y));
  width    = right_x  - offset_x;
  height   = bottom_y - offset_y;

  config.width  = width;
  config.height = height;

  if (config.roi_offset_x != camera_config_.roi_offset_x || first_run_) {
    changed = true;
    setFeatureValue("OffsetX", static_cast<VmbInt64_t>(config.roi_offset_x));
  }
  if (config.roi_offset_y != camera_config_.roi_offset_y || first_run_) {
    changed = true;
    setFeatureValue("OffsetY", static_cast<VmbInt64_t>(config.roi_offset_y));
  }
  if (config.width != camera_config_.width || first_run_) {
    changed = true;
    setFeatureValue("Width", static_cast<VmbInt64_t>(config.width));
  }
  if (config.height != camera_config_.height || first_run_) {
    changed = true;
    setFeatureValue("Height", static_cast<VmbInt64_t>(config.height));
  }

  if(changed){
    ROS_INFO_STREAM("New ROI config: "
      << "\n\tOffsetX : " << config.roi_offset_x << " was " << camera_config_.roi_offset_x
      << "\n\tOffsetY : " << config.roi_offset_y << " was " << camera_config_.roi_offset_y
      << "\n\tWidth   : " << config.width        << " was " << camera_config_.width
      << "\n\tHeight  : " << config.height   << " was " << camera_config_.height);
  }
}

/** Change the Bandwidth configuration */
void VimbaROS::updateBandwidthConfig(const Config& config,
                                     FeaturePtrVector feature_ptr_vec) {
  bool changed = false;
  if (config.stream_bytes_per_second
      != camera_config_.stream_bytes_per_second || first_run_) {
    changed = true;
    setFeatureValue("StreamBytesPerSecond",
                    static_cast<VmbInt64_t>(config.stream_bytes_per_second));
  }
  if(changed){
    ROS_INFO_STREAM("New Bandwidth config: "
      << "\n\tStreamBytesPerSecond : " << config.stream_bytes_per_second << " was " << camera_config_.stream_bytes_per_second);
  }
}

/** Change the Pixel Format configuration */
void VimbaROS::updatePixelFormatConfig(const Config& config,
                                       FeaturePtrVector feature_ptr_vec) {
  bool changed = false;
  if (config.pixel_format != camera_config_.pixel_format || first_run_) {
    changed = true;
    setFeatureValue("PixelFormat", config.pixel_format.c_str());
  }
  if(changed){
    ROS_INFO_STREAM("New PixelFormat config: "
      << "\n\tPixelFormat : " << config.pixel_format << " was " << camera_config_.pixel_format);
  }
}

/** Change the GPIO configuration */
void VimbaROS::updateGPIOConfig(const Config& config,
                                     FeaturePtrVector feature_ptr_vec) {
  bool changed = false;
  if (config.sync_in_selector
      != camera_config_.sync_in_selector || first_run_) {
    changed = true;
    setFeatureValue("SyncInSelector", config.sync_in_selector.c_str());
  }
  if (config.sync_out_polarity
      != camera_config_.sync_out_polarity || first_run_) {
    changed = true;
    setFeatureValue("SyncOutPolarity", config.sync_out_polarity.c_str());
  }
  if (config.sync_out_selector
      != camera_config_.sync_out_selector || first_run_) {
    changed = true;
    setFeatureValue("SyncOutSelector", config.sync_out_selector.c_str());
  }
  if (config.sync_out_source
      != camera_config_.sync_out_source || first_run_) {
    changed = true;
    setFeatureValue("SyncOutSource", config.sync_out_source.c_str());
  }
  if(changed){
    ROS_INFO_STREAM("New Bandwidth config: "
      << "\n\tSyncInSelector  : " << config.sync_in_selector  << " was " << camera_config_.sync_in_selector
      << "\n\tSyncOutPolarity : " << config.sync_out_polarity << " was " << camera_config_.sync_out_polarity
      << "\n\tSyncOutSelector : " << config.sync_out_selector << " was " << camera_config_.sync_out_selector
      << "\n\tSyncOutSource   : " << config.sync_out_source   << " was " << camera_config_.sync_out_source);
  }
}

// Template function to GET a feature value from the camera
template<typename T>
bool VimbaROS::getFeatureValue(const std::string& feature_str, T& val) {
  VmbErrorType err;
  FeaturePtr vimba_feature_ptr;
  VmbFeatureDataType data_type;
  err = vimba_camera_ptr_->GetFeatureByName(feature_str.c_str(),
                                            vimba_feature_ptr);
  if (VmbErrorSuccess == err) {
    bool readable;
    vimba_feature_ptr->IsReadable(readable);
    if (readable) {
      //VmbFeatureDataType data_type;
      vimba_feature_ptr->GetDataType(data_type);
      if ( VmbErrorSuccess != err ) {
        std::cout << "[Could not get feature Data Type. Error code: "
                  << err << "]" << std::endl;
      } else {
        std::string strValue;
        switch ( data_type ) {
          case VmbFeatureDataBool:
          bool bValue;
          err = vimba_feature_ptr->GetValue(bValue);
          if (VmbErrorSuccess == err) {
            val = static_cast<T>(bValue);
          }
          break;
          case VmbFeatureDataFloat:
          double fValue;
          err = vimba_feature_ptr->GetValue(fValue);
          if (VmbErrorSuccess == err) {
            val = static_cast<T>(fValue);
          }
          break;
          case VmbFeatureDataInt:
          VmbInt64_t  nValue;
          err = vimba_feature_ptr->GetValue(nValue);
          if (VmbErrorSuccess == err) {
            val = static_cast<T>(nValue);
          }
          break;
        }
        if (VmbErrorSuccess != err) {
          ROS_ERROR_STREAM("Could not get feature value. Error code: "
                    << errorCodeToMessage(err));
        }
      }
    } else {
      ROS_ERROR_STREAM("[AVT_Vimba_ROS]: Feature "
                       << feature_str
                       << " is not readable.");
    }
  } else {
    ROS_ERROR_STREAM("[AVT_Vimba_ROS]: Could not get feature " << feature_str);
  }
  ROS_INFO_STREAM("Asking for feature " << feature_str << " with datatype " << FeatureDataType[data_type] << " and value " << val);
  return (VmbErrorSuccess == err);
}

// Function to GET a feature value from the camera, overloaded to strings
bool VimbaROS::getFeatureValue(const std::string& feature_str, std::string& val) {
  VmbErrorType err;
  FeaturePtr vimba_feature_ptr;
  VmbFeatureDataType data_type;
  err = vimba_camera_ptr_->GetFeatureByName(feature_str.c_str(),
                                            vimba_feature_ptr);
  if (VmbErrorSuccess == err) {
    bool readable;
    vimba_feature_ptr->IsReadable(readable);
    if (readable) {
      //VmbFeatureDataType data_type;
      vimba_feature_ptr->GetDataType(data_type);
      if ( VmbErrorSuccess != err ) {
        std::cout << "[Could not get feature Data Type. Error code: "
                  << err << "]" << std::endl;
      } else {
        std::string strValue;
        switch ( data_type ) {
          case VmbFeatureDataEnum:
          err = vimba_feature_ptr->GetValue(strValue);
          if (VmbErrorSuccess == err) {
            val = strValue;
          }
          break;
          case VmbFeatureDataString:
          err = vimba_feature_ptr->GetValue(strValue);
          if (VmbErrorSuccess == err) {
            val = strValue;
          }
          break;
        }
        if (VmbErrorSuccess != err) {
          ROS_ERROR_STREAM("Could not get feature value. Error code: "
                    << errorCodeToMessage(err));
        }
      }
    } else {
      ROS_ERROR_STREAM("[AVT_Vimba_ROS]: Feature "
                       << feature_str
                       << " is not readable.");
    }
  } else {
    ROS_ERROR_STREAM("[AVT_Vimba_ROS]: Could not get feature " << feature_str);
  }
  ROS_DEBUG_STREAM("Asking for feature " << feature_str << " with datatype " << FeatureDataType[data_type] << " and value " << val);
  return (VmbErrorSuccess == err);
}

// Template function to SET a feature value from the camera
template<typename T>
bool VimbaROS::setFeatureValue(const std::string& feature_str, const T& val) {
  VmbErrorType err;
  FeaturePtr vimba_feature_ptr;
  err = vimba_camera_ptr_->GetFeatureByName(feature_str.c_str(),
                                            vimba_feature_ptr);
  if (VmbErrorSuccess == err) {
    bool writable;
    err = vimba_feature_ptr->IsWritable(writable);
    if (VmbErrorSuccess == err) {
      if (writable) {
        ROS_DEBUG_STREAM("Setting feature " << feature_str << " value " << val);
        VmbFeatureDataType data_type;
        err == vimba_feature_ptr->GetDataType(data_type);
        if (VmbErrorSuccess == err) {
          if (data_type == VmbFeatureDataEnum) {
            bool available;
            err = vimba_feature_ptr->IsValueAvailable(val, available);
            if (VmbErrorSuccess == err) {
              if (available){
                err = vimba_feature_ptr->SetValue(val);
              } else {
                ROS_ERROR_STREAM("[AVT_Vimba_ROS]: Feature "
                                 << feature_str
                                 << " is available now.");
              }
            } else {
              ROS_ERROR_STREAM("[AVT_Vimba_ROS]: Feature " << feature_str <<
                ": value unavailable\n\tERROR " << errorCodeToMessage(err));
            }
          } else {
            err = vimba_feature_ptr->SetValue(val);
          }
        } else {
          ROS_ERROR_STREAM("[AVT_Vimba_ROS]: Feature " << feature_str <<
            ": Bad data type\n\tERROR " << errorCodeToMessage(err));
        }
      } else {
        ROS_ERROR_STREAM("[AVT_Vimba_ROS]: Feature "
                         << feature_str
                         << " is not writable.");
      }
    } else {
      ROS_ERROR_STREAM("[AVT_Vimba_ROS]: Feature " << feature_str << ": ERROR " 
                       << errorCodeToMessage(err));
    }
  } else {
    ROS_ERROR_STREAM("[AVT_Vimba_ROS]: Could not get feature " << feature_str
      << "\n Error: " << errorCodeToMessage(err));
  }
  return (VmbErrorSuccess == err);
}


// Template function to RUN a command
bool VimbaROS::runCommand(const std::string& command_str) {
  FeaturePtr feature_ptr;
  if ( VmbErrorSuccess == vimba_camera_ptr_->GetFeatureByName( command_str.c_str(), feature_ptr ))
  {
    if ( VmbErrorSuccess == feature_ptr->RunCommand() )
    {
      bool is_command_done = false;
      do
      {
        if ( VmbErrorSuccess != feature_ptr->IsCommandDone(is_command_done) )
        {
          break;
        }
        ROS_INFO_STREAM_THROTTLE(1,"Waiting for command " << command_str.c_str() << "...");
      } while ( false == is_command_done );
      ROS_INFO_STREAM("Command " << command_str.c_str() << " done!");
    }
  }
}


/** Translates Vimba error codes to readable error messages
*
* @param error Vimba error tyme
* @return readable string error
*
**/
std::string VimbaROS::errorCodeToMessage(VmbErrorType error) {
  std::map<VmbErrorType, std::string>::const_iterator iter =
    vimba_error_code_to_message_.find(error);
  if ( vimba_error_code_to_message_.end() != iter ) {
    return iter->second;
  }
  return "Unsupported error code passed.";
}

void VimbaROS::listAvailableCameras(void) {
  std::string name;
  CameraPtrVector cameras;

  if (VmbErrorSuccess == vimba_system_.Startup()) {
    if (VmbErrorSuccess == vimba_system_.GetCameras(cameras)) {
      for (CameraPtrVector::iterator iter = cameras.begin();
      cameras.end() != iter;
      ++iter) {
        if (VmbErrorSuccess == (*iter)->GetName(name)) {
          ROS_INFO_STREAM("[AVT_Vimba_ROS]: Found camera: ");
        }
        std::string strID;            // The ID of the cam
        std::string strName;          // The name of the cam
        std::string strModelname;     // The model name of the cam
        std::string strSerialNumber;  // The serial number of the cam
        std::string strInterfaceID;  // The ID of the interface the cam is connected to
        VmbErrorType err = (*iter)->GetID( strID );
        if ( VmbErrorSuccess != err )
        {
            ROS_ERROR_STREAM("[Could not get camera ID. Error code: " << err << "]");
        }
        err = (*iter)->GetName( strName );
        if ( VmbErrorSuccess != err )
        {
            ROS_ERROR_STREAM("[Could not get camera name. Error code: " << err << "]");
        }

        err = (*iter)->GetModel( strModelname );
        if ( VmbErrorSuccess != err )
        {
            ROS_ERROR_STREAM("[Could not get camera mode name. Error code: " << err << "]");
        }

        err = (*iter)->GetSerialNumber( strSerialNumber );
        if ( VmbErrorSuccess != err )
        {
            ROS_ERROR_STREAM("[Could not get camera serial number. Error code: " << err << "]");
        }

        err = (*iter)->GetInterfaceID( strInterfaceID );
        if ( VmbErrorSuccess != err )
        {
            ROS_ERROR_STREAM("[Could not get interface ID. Error code: " << err << "]");
        }
        ROS_INFO_STREAM("\t\t/// Camera Name: " << strName);
        ROS_INFO_STREAM("\t\t/// Model Name: " << strModelname);
        ROS_INFO_STREAM("\t\t/// Camera ID: " << strID);
        ROS_INFO_STREAM("\t\t/// Serial Number: " << strSerialNumber);
        ROS_INFO_STREAM("\t\t/// @ Interface ID: " << strInterfaceID);

      }
    }
  }
}

std::string VimbaROS::interfaceToString(VmbInterfaceType interfaceType) {
  switch ( interfaceType ) {
    case VmbInterfaceFirewire: return "FireWire";
      break;
    case VmbInterfaceEthernet: return "GigE";
      break;
    case VmbInterfaceUsb: return "USB";
      break;
    default: return "Unknown";
  }
}

std::string VimbaROS::accessModeToString(VmbAccessModeType modeType){
  std::string s;
  if (modeType & VmbAccessModeFull)        s = std::string("Read and write access");
  else if (modeType & VmbAccessModeRead)   s = std::string("Only read access");
  else if (modeType & VmbAccessModeConfig) s = std::string("Device configuration access");
  else if (modeType & VmbAccessModeLite)   s = std::string("Device read/write access without feature access (only addresses)");
  else if (modeType & VmbAccessModeNone)   s = std::string("No access");
  return s;
}

int VimbaROS::getTriggerModeInt(std::string mode_str){
  int mode;
  if (mode_str == TriggerMode[Freerun]){
    mode = Freerun;
  } else if (mode_str == TriggerMode[FixedRate]){
    mode = FixedRate;
  } else if (mode_str == TriggerMode[Software]){
    mode = Software;
  } else if (mode_str == TriggerMode[SyncIn1]){
    mode = SyncIn1;
  } else if (mode_str == TriggerMode[SyncIn2]){
    mode = SyncIn2;
  } else if (mode_str == TriggerMode[SyncIn3]){
    mode = SyncIn3;
  } else if (mode_str == TriggerMode[SyncIn4]){
    mode = SyncIn4;
  }
  return mode;
}

CameraPtr VimbaROS::openCamera(std::string id_str) {
  // Details:   The ID might be one of the following:
  //            "IP:169.254.12.13",
  //            "MAC:000f31000001",
  //            or a plain serial number: "1234567890".

  CameraPtr camera;
  VmbErrorType err = vimba_system_.GetCameraByID(id_str.c_str(), camera);
  if (VmbErrorSuccess == err) {
    std::string cam_id, cam_name, cam_model, cam_sn, cam_int_id;
    VmbInterfaceType cam_int_type;
    VmbAccessModeType accessMode;// = VmbAccessModeNone;
    camera->GetID(cam_id);
    camera->GetName(cam_name);
    camera->GetModel(cam_model);
    camera->GetSerialNumber(cam_sn);
    camera->GetInterfaceID(cam_int_id);
    camera->GetInterfaceType(cam_int_type);
    err = camera->GetPermittedAccess(accessMode);

    ROS_INFO_STREAM("[AVT_Vimba_ROS]: Opened camera with"
    << "\n\t\t * Name     : " << cam_name
    << "\n\t\t * Model    : " << cam_model
    << "\n\t\t * ID       : " << cam_id
    << "\n\t\t * S/N      : " << cam_sn
    << "\n\t\t * Itf. ID  : " << cam_int_id
    << "\n\t\t * Itf. Type: " << interfaceToString(cam_int_type)
    << "\n\t\t * Access   : " << accessModeToString(accessMode));

    err = camera->Open(VmbAccessModeFull);
    if (VmbErrorSuccess == err){
      printAllCameraFeatures(camera);
    } else {
      ROS_ERROR_STREAM("[AVT_Vimba_ROS]: Could not get camera " << id_str
        << "\n Error: " << errorCodeToMessage(err));
    }
  } else {
    ROS_ERROR_STREAM("[AVT_Vimba_ROS]: Could not get camera " << id_str
      << "\n Error: " << errorCodeToMessage(err));
  }
  running_ = true;
  return camera;
}

void VimbaROS::printAllCameraFeatures(CameraPtr camera) {
  VmbErrorType err;
  FeaturePtrVector features;

  // The static details of a feature
  std::string strName;           // The name of the feature
  std::string strDisplayName;    // The display name of the feature
  std::string strTooltip;        // A short description of the feature
  std::string strDescription;    // A long description of the feature
  std::string strCategory;       // A category to group features
  std::string strSFNCNamespace;  // The Std Feature Naming Convention namespace
  std::string strUnit;           // The measurement unit of the value
  VmbFeatureDataType eType;      // The data type of the feature

  // The changeable value of a feature
  VmbInt64_t  nValue;
  std::string strValue;

  std::stringstream strError;

  // Fetch all features of our cam
  err = camera->GetFeatures(features);
  if ( VmbErrorSuccess == err ) {
    // Query all static details as well as the value of
    // all fetched features and print them out.
    for (   FeaturePtrVector::const_iterator iter = features.begin();
      features.end() != iter;
      ++iter ) {
      err = (*iter)->GetName(strName);
      if (VmbErrorSuccess != err) {
        strError << "[Could not get feature Name. Error code: " << err << "]";
        strName.assign(strError.str());
      }

      err = (*iter)->GetDisplayName(strDisplayName);
      if (VmbErrorSuccess != err) {
        strError << "[Could not get feature Display Name. Error code: "
                 << err << "]";
        strDisplayName.assign(strError.str());
      }

      err = (*iter)->GetToolTip(strTooltip);
      if (VmbErrorSuccess != err) {
        strError << "[Could not get feature Tooltip. Error code: "
                 << err << "]";
        strTooltip.assign(strError.str());
      }

      err = (*iter)->GetDescription(strDescription);
      if (VmbErrorSuccess != err) {
        strError << "[Could not get feature Description. Error code: "
                 << err << "]";
        strDescription.assign(strError.str());
      }

      err = (*iter)->GetCategory(strCategory);
      if (VmbErrorSuccess != err) {
        strError << "[Could not get feature Category. Error code: "
                 << err << "]";
        strCategory.assign(strError.str());
      }

      err = (*iter)->GetSFNCNamespace(strSFNCNamespace);
      if (VmbErrorSuccess != err) {
        strError << "[Could not get feature SNFC Namespace. Error code: "
                 << err << "]";
        strSFNCNamespace.assign(strError.str());
      }

      err = (*iter)->GetUnit(strUnit);
      if (VmbErrorSuccess != err) {
        strError << "[Could not get feature Unit. Error code: " << err << "]";
        strUnit.assign(strError.str());
      }

      std::cout << "/// Feature Name: " << strName << std::endl;
      std::cout << "/// Display Name: " << strDisplayName << std::endl;
      std::cout << "/// Tooltip: " << strTooltip << std::endl;
      std::cout << "/// Description: " << strDescription << std::endl;
      std::cout << "/// SNFC Namespace: " << strSFNCNamespace << std::endl;
      std::cout << "/// Unit: " << strUnit << std::endl;
      std::cout << "/// Value: ";

      err = (*iter)->GetDataType(eType);
      if ( VmbErrorSuccess != err ) {
        std::cout << "[Could not get feature Data Type. Error code: "
                  << err << "]" << std::endl;
      } else {
        switch ( eType ) {
          case VmbFeatureDataBool:
          bool bValue;
          err = (*iter)->GetValue(bValue);
          if (VmbErrorSuccess == err) {
            std::cout << bValue << " bool" << std::endl;
          }
          break;
          case VmbFeatureDataEnum:
          err = (*iter)->GetValue(strValue);
          if (VmbErrorSuccess == err) {
            std::cout << strValue << " str Enum" << std::endl;
          }
          break;
          case VmbFeatureDataFloat:
          double fValue;
          err = (*iter)->GetValue(fValue);
          {
            std::cout << fValue << " float" << std::endl;
          }
          break;
          case VmbFeatureDataInt:
          err = (*iter)->GetValue(nValue);
          {
            std::cout << nValue << " int" << std::endl;
          }
          break;
          case VmbFeatureDataString:
          err = (*iter)->GetValue(strValue);
          {
            std::cout << strValue << " str" << std::endl;
          }
          break;
          case VmbFeatureDataCommand:
          default:
          std::cout << "[None]" << std::endl;
          break;
        }
        if (VmbErrorSuccess != err) {
          std::cout << "Could not get feature value. Error code: "
                    << err << std::endl;
        }
      }

      std::cout << std::endl;
    }
  } else {
    std::cout << "Could not get features. Error code: " << err << std::endl;
  }
}

void VimbaROS::initApi(void) {
  vimba_error_code_to_message_[VmbErrorSuccess]        = "Success.";
  vimba_error_code_to_message_[VmbErrorApiNotStarted]  = "API not started.";
  vimba_error_code_to_message_[VmbErrorNotFound]       = "Not found.";
  vimba_error_code_to_message_[VmbErrorBadHandle]      = "Invalid handle ";
  vimba_error_code_to_message_[VmbErrorDeviceNotOpen]  = "Device not open.";
  vimba_error_code_to_message_[VmbErrorInvalidAccess]  = "Invalid access.";
  vimba_error_code_to_message_[VmbErrorBadParameter]   = "Bad parameter.";
  vimba_error_code_to_message_[VmbErrorStructSize]     = "Wrong DLL version.";
  vimba_error_code_to_message_[VmbErrorWrongType]      = "Wrong type.";
  vimba_error_code_to_message_[VmbErrorInvalidValue]   = "Invalid value.";
  vimba_error_code_to_message_[VmbErrorTimeout]        = "Timeout.";
  vimba_error_code_to_message_[VmbErrorOther]          = "TL error.";
  vimba_error_code_to_message_[VmbErrorInvalidCall]    = "Invalid call.";
  vimba_error_code_to_message_[VmbErrorNoTL]           = "TL not loaded.";
  vimba_error_code_to_message_[VmbErrorNotImplemented] = "Not implemented.";
  vimba_error_code_to_message_[VmbErrorNotSupported]   = "Not supported.";
  vimba_error_code_to_message_[VmbErrorResources]      = "Resource not available.";
  vimba_error_code_to_message_[VmbErrorInternalFault]  = "Unexpected fault in VmbApi or driver.";
  vimba_error_code_to_message_[VmbErrorMoreData]       = "More data returned than memory provided.";

  if (VmbErrorSuccess == vimba_system_.Startup()) {
    ROS_INFO_STREAM("[AVT_Vimba_ROS]: AVT Vimba System initialized successfully");
    listAvailableCameras();
  } else {
    ROS_ERROR_STREAM("[AVT_Vimba_ROS]: Could not start Vimba system: "
      << errorCodeToMessage(vimba_system_.Startup()) );
  }
}
};
