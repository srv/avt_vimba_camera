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

#include <avt_vimba_camera/stereo_camera.h>
#include <driver_base/SensorLevels.h>

#include <boost/bind.hpp>

#define DEBUG_PRINTS 1

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
  synced_frames_ = 0;

  // Set the frame callbacks
  left_cam_.setCallback(boost::bind(&avt_vimba_camera::StereoCamera::leftFrameCallback, this, _1));
  right_cam_.setCallback(boost::bind(&avt_vimba_camera::StereoCamera::rightFrameCallback, this, _1));

  // Get the parameters
  nhp_.param("left_ip", left_ip_, std::string(""));
  nhp_.param("right_ip", right_ip_, std::string(""));
  nhp_.param("left_guid", left_guid_, std::string(""));
  nhp_.param("right_guid", right_guid_, std::string(""));
  nhp_.param("left_camera_info_url", left_camera_info_url_, std::string(""));
  nhp_.param("right_camera_info_url", right_camera_info_url_, std::string(""));

  double max_allowed_time_error_double;
  nhp_.param("max_allowed_nsec_error", max_nsec_sync_error_, 5e6);  // time in seconcs

  nhp_.param("master_out_source", master_out_source_, std::string("Exposing"));
  nhp_.param("slave_trigger_source", slave_trigger_source_, std::string("Line1"));
  nhp_.param("slave_in_source", slave_in_source_, std::string("SyncIn1"));
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
      left_img_ = img;
      left_ready_ = true;
      left_time_ = ros_time;
      vimba_frame_ptr->GetTimestamp(left_timestamp_);
      sync();
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
      right_img_ = img;
      right_ready_ = true;
      right_time_ = ros_time;
      vimba_frame_ptr->GetTimestamp(right_timestamp_);
      sync();
    }
    else {
      ROS_WARN_STREAM("Function frameToImage returned 0. No image published.");
    }
  }
}

void StereoCamera::sync(void) {
  if (left_ready_ && right_ready_) {
    if( abs(left_time_.toNSec() - right_time_.toNSec()) <= max_nsec_sync_error_ ) {
      synced_frames_++;
      ROS_INFO_STREAM("Publishing sync'd pair with " << abs(left_time_.toNSec() - right_time_.toNSec()) << " ns error.");
      ros::Time ros_time = left_time_;
      sensor_msgs::CameraInfo lci = left_info_man_->getCameraInfo();
      sensor_msgs::CameraInfo rci = right_info_man_->getCameraInfo();

      lci.header.stamp = ros_time;
      left_img_.header.stamp = ros_time;
      left_img_.header.frame_id = lci.header.frame_id;
      rci.header.stamp = ros_time;
      right_img_.header.stamp = ros_time;
      right_img_.header.frame_id = rci.header.frame_id;
      left_pub_.publish(left_img_, lci);
      right_pub_.publish(right_img_, rci);
      // pub_freq_->tick(ros_time);
      right_ready_ = false;
      left_ready_ = false;
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
void StereoCamera::configure(Config& newconfig, uint32_t level) {
  Config left_config = newconfig;
  Config right_config = newconfig;

  // Left camera is considered MASTER and right SLAVE
  left_config.sync_out_source = master_out_source_;
  right_config.trigger_source = slave_trigger_source_;
  right_config.sync_in_selector = slave_in_source_;

  try {
    // resolve frame ID using tf_prefix parameter
    left_config.frame_id  = "left_optical";
    right_config.frame_id = "right_optical";

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
    left_cam_.updateConfig(left_config);
    right_cam_.updateConfig(right_config);
    updateCameraInfo(newconfig);
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Error reconfiguring avt_vimba_camera node : " << e.what());
  }
}

void StereoCamera::updateCameraInfo(const Config& config) {
  // Get camera_info from the manager
  sensor_msgs::CameraInfo left_ci = left_info_man_->getCameraInfo();
  sensor_msgs::CameraInfo right_ci = right_info_man_->getCameraInfo();

  // Set the frame id
  left_ci.header.frame_id = config.frame_id;
  right_ci.header.frame_id = config.frame_id;

  // Set the operational parameters in CameraInfo (binning, ROI)
  left_ci.height    = config.height;
  left_ci.width     = config.width;
  left_ci.binning_x = config.binning_x;
  left_ci.binning_y = config.binning_y;

  right_ci.height    = config.height;
  right_ci.width     = config.width;
  right_ci.binning_x = config.binning_x;
  right_ci.binning_y = config.binning_y;

  // ROI in CameraInfo is in unbinned coordinates, need to scale up
  left_ci.roi.x_offset = config.roi_offset_x;
  left_ci.roi.y_offset = config.roi_offset_y;
  left_ci.roi.height   = config.roi_height;
  left_ci.roi.width    = config.roi_width;

  right_ci.roi.x_offset = config.roi_offset_x;
  right_ci.roi.y_offset = config.roi_offset_y;
  right_ci.roi.height   = config.roi_height;
  right_ci.roi.width    = config.roi_width;

  std::string left_camera_info_url, right_camera_info_url;
  nhp_.getParam("left_camera_info_url", left_camera_info_url);
  nhp_.getParam("right_camera_info_url", right_camera_info_url);

  // set the new URL and load CameraInfo (if any) from it
  if (left_camera_info_url != left_camera_info_url_) {
    left_info_man_->setCameraName(config.frame_id);
    if (left_info_man_->validateURL(left_camera_info_url)) {
      left_info_man_->loadCameraInfo(left_camera_info_url);
      left_ci = left_info_man_->getCameraInfo();
    } else {
      ROS_WARN_STREAM("Camera info URL not valid: " << left_camera_info_url);
    }
  }

  if (right_camera_info_url != right_camera_info_url_) {
    right_info_man_->setCameraName(config.frame_id);
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
  left_ci.roi.do_rectify = lRoiMatchesCalibration ||
    lResolutionMatchesCalibration;

  bool rRoiMatchesCalibration = (right_ci.height == config.roi_height
                              && right_ci.width == config.roi_width);
  bool rResolutionMatchesCalibration = (right_ci.width == config.width
                                   && right_ci.height == config.height);
  // check
  right_ci.roi.do_rectify = rRoiMatchesCalibration ||
    rResolutionMatchesCalibration;

  // push the changes to manager
  left_info_man_->setCameraInfo(left_ci);
  right_info_man_->setCameraInfo(right_ci);
}

void StereoCamera::syncDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("Number of left frames", left_frames_);
  stat.add("Number of right frames", right_frames_);
  stat.add("Number of sync'd frames", synced_frames_);
  stat.add("Dropped % sync'd frames", 2 * (double)synced_frames_ / ((double)left_frames_ + (double)right_frames_));

  if (synced_frames_ <= left_frames_)
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Sync'd images.");
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Wrong synchronization");
}
};
