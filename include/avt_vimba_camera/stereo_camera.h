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

#ifndef STEREO_CAMERA_H
#define STEREO_CAMERA_H

#include <avt_vimba_camera/avt_vimba_camera.h>
#include <avt_vimba_camera/AvtVimbaCameraConfig.h>
#include <avt_vimba_camera/avt_vimba_api.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <string>

namespace avt_vimba_camera {
class StereoCamera {
 public:
  StereoCamera(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~StereoCamera(void);

 private:
  AvtVimbaApi api_;
  AvtVimbaCamera left_cam_;
  AvtVimbaCamera right_cam_;

  diagnostic_updater::Updater updater_;
  diagnostic_updater::TopicDiagnostic* pub_freq_;
  diagnostic_updater::FunctionDiagnosticTask* sync_check_;

  bool left_ready_;
  long long unsigned int left_frames_;
  long long unsigned int right_frames_;
  long long unsigned int synced_frames_;
  bool right_ready_;
  bool show_debug_prints_;

  long long unsigned int left_timestamp_;
  long long unsigned int right_timestamp_;

  // Params
  std::string left_ip_;
  std::string right_ip_;
  std::string left_guid_;
  std::string right_guid_;
  std::string left_camera_info_url_;
  std::string right_camera_info_url_;
  std::string master_out_source_;
  std::string slave_trigger_source_;
  std::string slave_in_source_;

  ros::Time left_time_;
  ros::Time right_time_;
  double max_nsec_sync_error_;

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::NodeHandle left_nhp_;
  ros::NodeHandle right_nhp_;

  // Image transport
  image_transport::ImageTransport it_;
  //image_transport::ImageTransport right_it_;

  // ROS Camera publisher
  image_transport::CameraPublisher left_pub_;
  image_transport::CameraPublisher right_pub_;

  sensor_msgs::Image left_img_;
  sensor_msgs::Image right_img_;

  // sensor_msgs::CameraInfo left_info_;
  // sensor_msgs::CameraInfo right_info_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> left_info_man_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> right_info_man_;

  // Dynamic reconfigure
  typedef avt_vimba_camera::AvtVimbaCameraConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  ReconfigureServer reconfigure_server_;

  // Camera configuration
  Config camera_config_;

  void leftFrameCallback(const FramePtr& vimba_frame_ptr);
  void rightFrameCallback(const FramePtr& vimba_frame_ptr);
  void configure(Config& newconfig, uint32_t level);
  void updateCameraInfo(const Config& config);
  void sync(void);
  void syncDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);
};
}
#endif
