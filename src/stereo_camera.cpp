/// Copyright (c) 2014,
/// Systems, Robotics and Vision Group
/// University of the Balearic Islands
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
///       Systems, Robotics and Vision Group, Univ. of the Balearic Islands
///     * Neither the name of Systems, Robotics and Vision Group, University of
///       the Balearic Islands nor the names of its contributors may be used
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

#include <avt_vimba_camera/stereo_camera.h>
#include <driver_base/SensorLevels.h>

#include <boost/bind.hpp>
#include <thread>

namespace avt_vimba_camera {

StereoCamera::StereoCamera(ros::NodeHandle nh, ros::NodeHandle nhp)
:nh_(nh), nhp_(nhp), it_(nhp), left_cam_("left"), right_cam_("right") {
  // TODO use nodelets with getMTNodeHandle()
  // Start Vimba & list all available cameras
  api_.start();

  // Set the image publishers before the streaming
  left_pub_  = it_.advertiseCamera("left/image_raw",  1);
  right_pub_ = it_.advertiseCamera("right/image_raw", 1);
  left_frames_ = 0;
  right_frames_ = 0;

  // Set the frame callbacks
  std::thread left_thread([&]() {
    left_cam_.setCallback(boost::bind(&avt_vimba_camera::StereoCamera::leftFrameCallback, this, _1));
  });
  std::thread right_thread([&]() {
    right_cam_.setCallback(boost::bind(&avt_vimba_camera::StereoCamera::rightFrameCallback, this, _1));
  });

  // Get the parameters
  nhp_.param("left_ip", left_ip_, std::string(""));
  nhp_.param("right_ip", right_ip_, std::string(""));
  nhp_.param("left_guid", left_guid_, std::string(""));
  nhp_.param("right_guid", right_guid_, std::string(""));
  nhp_.param("left_camera_info_url", left_camera_info_url_, std::string(""));
  nhp_.param("right_camera_info_url", right_camera_info_url_, std::string(""));

  double max_allowed_time_error_double;
  nhp_.param("max_allowed_nsec_error", max_nsec_sync_error_, 10e8);  // time in nanoseconds
  nhp_.param("show_debug_prints", show_debug_prints_, false);

  // Publish a hardware message to know & track the state of the cam
  updater_.setHardwareID("Stereo-"+left_guid_+"-"+right_guid_);
  updater_.broadcast(0, "Device is closed.");
  double min_freq = 2.0;
  double max_freq = 20.0;
  diagnostic_updater::FrequencyStatusParam freq_params(&min_freq, &max_freq, 0.1, 10);
  double min_stamp = 0.001;
  double max_stamp = 1.0;
  diagnostic_updater::TimeStampStatusParam stamp_params(min_stamp, max_stamp);
  // pub_freq_ = new diagnostic_updater::TopicDiagnostic("Stereo Image Pair", updater_, freq_params, stamp_params);
  updater_.add("Synchronization", boost::bind(&avt_vimba_camera::StereoCamera::syncDiagnostic, this, _1));
  updater_.update();

  // Set camera info managers
  left_info_man_  = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(ros::NodeHandle(nhp, "left"),"left_optical",left_camera_info_url_));
  right_info_man_ = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(ros::NodeHandle(nhp, "right"),"right_optical",right_camera_info_url_));

  // Start dynamic_reconfigure & run configure()
  reconfigure_server_.setCallback(boost::bind(&StereoCamera::configure, this, _1, _2));
}

StereoCamera::~StereoCamera() {
  left_cam_.stop();
  right_cam_.stop();
  updater_.broadcast(0, "Device is closed.");
  left_pub_.shutdown();
  right_pub_.shutdown();
}

void StereoCamera::leftFrameCallback(const FramePtr& vimba_frame_ptr) {
  left_frames_++;
  ros::Time ros_time = ros::Time::now();
  if(left_pub_.getNumSubscribers() > 0){
    sensor_msgs::Image img;
    if (api_.frameToImage(vimba_frame_ptr, img)){
      sensor_msgs::CameraInfo lci = left_info_man_->getCameraInfo();
      lci.header.stamp = ros_time;
      left_img_.header.stamp = ros_time;
      left_img_.header.frame_id = lci.header.frame_id;
      left_pub_.publish(left_img_, lci);
    }
    else {
      ROS_WARN_STREAM("Function frameToImage returned 0. No image published.");
    }
  }
  updater_.update();
}

void StereoCamera::rightFrameCallback(const FramePtr& vimba_frame_ptr) {
  right_frames_++;
  ros::Time ros_time = ros::Time::now();
  if(right_pub_.getNumSubscribers() > 0){
    sensor_msgs::Image img;
    if (api_.frameToImage(vimba_frame_ptr, img)){
      sensor_msgs::CameraInfo rci = right_info_man_->getCameraInfo();
      rci.header.stamp = ros_time;
      right_img_.header.stamp = ros_time;
      right_img_.header.frame_id = rci.header.frame_id;
      right_pub_.publish(right_img_, rci);
    }
    else {
      ROS_WARN_STREAM("Function frameToImage returned 0. No image published.");
    }
  }
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
void StereoCamera::configure(StereoConfig& newconfig, uint32_t level) {
  // Left camera is considered MASTER and right SLAVE
  try {
    // The camera already stops & starts acquisition
    // so there's no problem on changing any feature.
    if (!left_cam_.isOpened()) {
      left_cam_.start(left_ip_, left_guid_, show_debug_prints_);
    }
    if (!right_cam_.isOpened()) {
      right_cam_.start(right_ip_, right_guid_, show_debug_prints_);
      if (left_cam_.isOpened() && right_cam_.isOpened()){
        left_cam_.resetTimestamp();
        right_cam_.resetTimestamp();
      }
    }
    Config left_config, right_config;
    copyConfig(newconfig, left_config, right_config);
    left_cam_.updateConfig(left_config);
    right_cam_.updateConfig(right_config);
    updateCameraInfo(newconfig);
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Error reconfiguring avt_vimba_camera node : " << e.what());
  }
}

void StereoCamera::copyConfig(StereoConfig& sc, Config& lc, Config& rc) {
  // left camera
  lc.frame_id = sc.left_frame_id;
  lc.trig_timestamp_topic = sc.left_trig_timestamp_topic;
  lc.acquisition_mode = sc.left_acquisition_mode;
  lc.acquisition_rate = sc.left_acquisition_rate;
  lc.trigger_source = sc.left_trigger_source;
  lc.trigger_mode = sc.left_trigger_mode;
  lc.trigger_selector = sc.left_trigger_selector;
  lc.trigger_activation = sc.left_trigger_activation;
  lc.trigger_delay = sc.left_trigger_delay;
  lc.exposure = sc.exposure;
  lc.exposure_auto = sc.exposure_auto;
  lc.exposure_auto_alg = sc.exposure_auto_alg;
  lc.exposure_auto_tol = sc.exposure_auto_tol;
  lc.exposure_auto_max = sc.exposure_auto_max;
  lc.exposure_auto_min = sc.exposure_auto_min;
  lc.exposure_auto_outliers = sc.exposure_auto_outliers;
  lc.exposure_auto_rate = sc.exposure_auto_rate;
  lc.exposure_auto_target = sc.exposure_auto_target;
  lc.gain = sc.gain;
  lc.gain_auto = sc.gain_auto;
  lc.gain_auto_tol = sc.gain_auto_tol;
  lc.gain_auto_max = sc.gain_auto_max;
  lc.gain_auto_min = sc.gain_auto_min;
  lc.gain_auto_outliers = sc.gain_auto_outliers;
  lc.gain_auto_rate = sc.gain_auto_rate;
  lc.gain_auto_target = sc.gain_auto_target;
  lc.balance_ratio_abs = sc.balance_ratio_abs;
  lc.balance_ratio_selector = sc.balance_ratio_selector;
  lc.whitebalance_auto = sc.whitebalance_auto;
  lc.whitebalance_auto_tol = sc.whitebalance_auto_tol;
  lc.whitebalance_auto_rate = sc.whitebalance_auto_rate;
  lc.binning_x = sc.binning_x;
  lc.binning_y = sc.binning_y;
  lc.decimation_x = sc.decimation_x;
  lc.decimation_y = sc.decimation_y;
  lc.width = sc.width;
  lc.height = sc.height;
  lc.roi_width = sc.roi_width;
  lc.roi_height = sc.roi_height;
  lc.roi_offset_x = sc.roi_offset_x;
  lc.roi_offset_y = sc.roi_offset_y;
  lc.pixel_format = sc.pixel_format;
  lc.stream_bytes_per_second = sc.stream_bytes_per_second;
  lc.ptp_mode = sc.left_ptp_mode;
  lc.sync_in_selector = sc.left_sync_in_selector;
  lc.sync_out_polarity = sc.left_sync_out_polarity;
  lc.sync_out_selector = sc.left_sync_out_selector;
  lc.sync_out_source = sc.left_sync_out_source;
  // right camera
  rc.frame_id = sc.right_frame_id;
  rc.trig_timestamp_topic = sc.right_trig_timestamp_topic;
  rc.acquisition_mode = sc.right_acquisition_mode;
  rc.acquisition_rate = sc.right_acquisition_rate;
  rc.trigger_source = sc.right_trigger_source;
  rc.trigger_mode = sc.right_trigger_mode;
  rc.trigger_selector = sc.right_trigger_selector;
  rc.trigger_activation = sc.right_trigger_activation;
  rc.trigger_delay = sc.right_trigger_delay;
  rc.exposure = sc.exposure;
  rc.exposure_auto = sc.exposure_auto;
  rc.exposure_auto_alg = sc.exposure_auto_alg;
  rc.exposure_auto_tol = sc.exposure_auto_tol;
  rc.exposure_auto_max = sc.exposure_auto_max;
  rc.exposure_auto_min = sc.exposure_auto_min;
  rc.exposure_auto_outliers = sc.exposure_auto_outliers;
  rc.exposure_auto_rate = sc.exposure_auto_rate;
  rc.exposure_auto_target = sc.exposure_auto_target;
  rc.gain = sc.gain;
  rc.gain_auto = sc.gain_auto;
  rc.gain_auto_tol = sc.gain_auto_tol;
  rc.gain_auto_max = sc.gain_auto_max;
  rc.gain_auto_min = sc.gain_auto_min;
  rc.gain_auto_outliers = sc.gain_auto_outliers;
  rc.gain_auto_rate = sc.gain_auto_rate;
  rc.gain_auto_target = sc.gain_auto_target;
  rc.balance_ratio_abs = sc.balance_ratio_abs;
  rc.balance_ratio_selector = sc.balance_ratio_selector;
  rc.whitebalance_auto = sc.whitebalance_auto;
  rc.whitebalance_auto_tol = sc.whitebalance_auto_tol;
  rc.whitebalance_auto_rate = sc.whitebalance_auto_rate;
  rc.binning_x = sc.binning_x;
  rc.binning_y = sc.binning_y;
  rc.decimation_x = sc.decimation_x;
  rc.decimation_y = sc.decimation_y;
  rc.width = sc.width;
  rc.height = sc.height;
  rc.roi_width = sc.roi_width;
  rc.roi_height = sc.roi_height;
  rc.roi_offset_x = sc.roi_offset_x;
  rc.roi_offset_y = sc.roi_offset_y;
  rc.pixel_format = sc.pixel_format;
  rc.stream_bytes_per_second = sc.stream_bytes_per_second;
  rc.ptp_mode = sc.right_ptp_mode;
  rc.sync_in_selector = sc.right_sync_in_selector;
  rc.sync_out_polarity = sc.right_sync_out_polarity;
  rc.sync_out_selector = sc.right_sync_out_selector;
  rc.sync_out_source = sc.right_sync_out_source;
}

void StereoCamera::updateCameraInfo(const StereoConfig& config) {
  // Get camera_info from the manager
  sensor_msgs::CameraInfo left_ci = left_info_man_->getCameraInfo();
  sensor_msgs::CameraInfo right_ci = right_info_man_->getCameraInfo();

  // Set the frame id
  left_ci.header.frame_id = config.left_frame_id;
  right_ci.header.frame_id = config.right_frame_id;

  // Set the operational parameters in CameraInfo (binning, ROI)
  int binning_or_decimation_x = std::max(config.binning_x, config.decimation_x);
  int binning_or_decimation_y = std::max(config.binning_y, config.decimation_y);

  left_ci.height    = config.height/binning_or_decimation_x;
  left_ci.width     = config.width/binning_or_decimation_y;
  left_ci.binning_x = 1;
  left_ci.binning_y = 1;

  right_ci.height    = config.height/binning_or_decimation_x;
  right_ci.width     = config.width/binning_or_decimation_y;
  right_ci.binning_x = 1;
  right_ci.binning_y = 1;

  // ROI in CameraInfo is in unbinned coordinates, need to scale up
  left_ci.roi.x_offset = config.roi_offset_x/binning_or_decimation_x;
  left_ci.roi.y_offset = config.roi_offset_y/binning_or_decimation_y;
  left_ci.roi.height   = config.roi_height/binning_or_decimation_x;
  left_ci.roi.width    = config.roi_width/binning_or_decimation_y;

  right_ci.roi.x_offset = config.roi_offset_x/binning_or_decimation_x;
  right_ci.roi.y_offset = config.roi_offset_y/binning_or_decimation_y;
  right_ci.roi.height   = config.roi_height/binning_or_decimation_x;
  right_ci.roi.width    = config.roi_width/binning_or_decimation_y;

  std::string left_camera_info_url, right_camera_info_url;
  nhp_.getParamCached("left_camera_info_url", left_camera_info_url);
  nhp_.getParamCached("right_camera_info_url", right_camera_info_url);

  // set the new URL and load CameraInfo (if any) from it
  if (left_camera_info_url != left_camera_info_url_) {
    left_info_man_->setCameraName(config.left_frame_id);
    if (left_info_man_->validateURL(left_camera_info_url)) {
      left_info_man_->loadCameraInfo(left_camera_info_url);
      left_ci = left_info_man_->getCameraInfo();
    } else {
      ROS_WARN_STREAM("Camera info URL not valid: " << left_camera_info_url);
    }
  }

  if (right_camera_info_url != right_camera_info_url_) {
    right_info_man_->setCameraName(config.right_frame_id);
    if (right_info_man_->validateURL(right_camera_info_url)) {
      right_info_man_->loadCameraInfo(right_camera_info_url);
      right_ci = right_info_man_->getCameraInfo();
    } else {
      ROS_WARN_STREAM("Camera info URL not valid: " << right_camera_info_url);
    }
  }

  bool lRoiMatchesCalibration = (left_ci.height == config.roi_height
                              && left_ci.width == config.roi_width);
  bool lResolutionMatchesCalibration = (left_ci.width == config.width
                                   && left_ci.height == config.height);
  // check
  left_ci.roi.do_rectify = lRoiMatchesCalibration || lResolutionMatchesCalibration;

  bool rRoiMatchesCalibration = (right_ci.height == config.roi_height
                              && right_ci.width == config.roi_width);
  bool rResolutionMatchesCalibration = (right_ci.width == config.width
                                   && right_ci.height == config.height);
  // check
  right_ci.roi.do_rectify = rRoiMatchesCalibration || rResolutionMatchesCalibration;

  // push the changes to manager
  left_info_man_->setCameraInfo(left_ci);
  right_info_man_->setCameraInfo(right_ci);
}

void StereoCamera::syncDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("Number of left frames", left_frames_);
  stat.add("Number of right frames", right_frames_);
}
};
