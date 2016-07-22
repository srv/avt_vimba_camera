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

#include <avt_vimba_camera/mono_camera.h>

#define DEBUG_PRINTS 1

namespace avt_vimba_camera {

MonoCamera::MonoCamera(ros::NodeHandle nh, ros::NodeHandle nhp) : nh_(nh), nhp_(nhp), it_(nhp), cam_(ros::this_node::getName()) {
  // Prepare node handle for the camera
  // TODO use nodelets with getMTNodeHandle()

  // Start Vimba & list all available cameras
  api_.start();

  // Set the image publisher before the streaming
  pub_  = it_.advertiseCamera("image_raw",  1);

  memory_loaded_ = false;

  // Set the frame callback
  cam_.setCallback(boost::bind(&avt_vimba_camera::MonoCamera::frameCallback, this, _1));

  // Set the params
  nhp_.param("ip", ip_, std::string(""));
  nhp_.param("guid", guid_, std::string(""));
  nhp_.param("camera_info_url", camera_info_url_, std::string(""));
  std::string frame_id;
  nhp_.param("frame_id", frame_id, std::string(""));
  nhp_.param("show_debug_prints", show_debug_prints_, false);

  // Set camera info manager
  info_man_  = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(nh_, frame_id, camera_info_url_));

  // Service call for setting calibration.
  set_camera_info_srv_ = nhp_.advertiseService("set_camera_info",
                                               &MonoCamera::setCameraInfo,
                                               this);

  // Start dynamic_reconfigure & run configure()
  reconfigure_server_.setCallback(boost::bind(&avt_vimba_camera::MonoCamera::configure, this, _1, _2));
}

MonoCamera::~MonoCamera(void) {
  cam_.stop();
  pub_.shutdown();
}

void MonoCamera::frameCallback(const FramePtr& vimba_frame_ptr) {
  ros::Time ros_time = ros::Time::now();
  if (pub_.getNumSubscribers() > 0) {
    sensor_msgs::Image img;
    if (api_.frameToImage(vimba_frame_ptr, img)) {
      sensor_msgs::CameraInfo ci = info_man_->getCameraInfo();
      ci.header.stamp = img.header.stamp = ros_time;
      img.header.frame_id = ci.header.frame_id;
      pub_.publish(img, ci);
    } else {
      ROS_WARN_STREAM("Function frameToImage returned 0. No image published.");
    }
  }
  // updater_.update();
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
void MonoCamera::configure(Config& newconfig, uint32_t level) {
  try {
    // resolve frame ID using tf_prefix parameter
    if (newconfig.frame_id == "") {
      newconfig.frame_id = "camera";
    }
    // The camera already stops & starts acquisition
    // so there's no problem on changing any feature.
    if (!cam_.isOpened()) {
      cam_.start(ip_, guid_, show_debug_prints_);
    }

    Config config = newconfig;
    cam_.updateConfig(newconfig);
    updateCameraInfo(config);
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Error reconfiguring mono_camera node : " << e.what());
  }
}

void MonoCamera::updateCameraInfo(const avt_vimba_camera::AvtVimbaCameraConfig& config) {

  // Get camera_info from the manager
  sensor_msgs::CameraInfo ci = info_man_->getCameraInfo();

  // Set the frame id
  ci.header.frame_id = config.frame_id;

  // Set the operational parameters in CameraInfo (binning, ROI)
  int binning_or_decimation_x = std::max(config.binning_x, config.decimation_x);
  int binning_or_decimation_y = std::max(config.binning_y, config.decimation_y);

  // Set the operational parameters in CameraInfo (binning, ROI)
  ci.height    = config.height; 
  ci.width     = config.width;
  ci.binning_x = binning_or_decimation_x;
  ci.binning_y = binning_or_decimation_y;

  // ROI in CameraInfo is in unbinned coordinates, need to scale up
  ci.roi.x_offset = config.roi_offset_x;
  ci.roi.y_offset = config.roi_offset_y;
  ci.roi.height   = config.roi_height;
  ci.roi.width    = config.roi_width;

  // Load camera_info from camera memory
  if(!info_man_->isCalibrated() && !memory_loaded_) {
    ROS_WARN("Failed to load camera_info from file. Trying to load camera_info from camera memory ...");

	UcharVector loaded_data;
	VmbErrorType result = cam_.loadCameraMemory(loaded_data);
	if(result == VmbErrorSuccess) {
	  std::string calibration_data;
	  for(int i = 0 ; i < loaded_data.size() ; i++)
	    calibration_data += loaded_data[i];

      std::string cam_name = cam_.getCameraName();

      std::stringstream yml_stream;
      yml_stream << calibration_data;
      if(camera_calibration_parsers::readCalibrationYml(yml_stream, cam_name, ci)) {
        ROS_INFO("Loaded camera_info from camera memory for camera '%s'.", cam_name.c_str());
        memory_loaded_ = true;
      }
      else
        ROS_WARN("Failed to parse camera_info from camera memory.");
    }
    else
      ROS_WARN("Failed to load camera_info from camera memory.");
  }

  // set the new URL and load CameraInfo (if any) from it
  std::string camera_info_url;
  nhp_.getParam("camera_info_url", camera_info_url);
  if (camera_info_url != camera_info_url_) {
    info_man_->setCameraName(config.frame_id);
    if (info_man_->validateURL(camera_info_url)) {
      info_man_->loadCameraInfo(camera_info_url);
      ci = info_man_->getCameraInfo();
    } else {
      ROS_WARN_STREAM("Camera info URL not valid: " << camera_info_url);
    }
  }

  bool roiMatchesCalibration = (ci.height == config.roi_height
                              && ci.width == config.roi_width);
  bool resolutionMatchesCalibration = (ci.width == config.width
                                   && ci.height == config.height);
  // check
  ci.roi.do_rectify = roiMatchesCalibration || resolutionMatchesCalibration;

  // push the changes to manager
  info_man_->setCameraInfo(ci);
}

bool MonoCamera::setCameraInfo(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& rsp) {
  ROS_INFO("New camera_info received.");

  sensor_msgs::CameraInfo &info = req.camera_info;

  if(info.width != info_man_->getCameraInfo().width || info.height != info_man_->getCameraInfo().height)
  {
    rsp.success = false;
    rsp.status_message = (boost::format("camera_info resolution %ix%i does not match current video setting, camera is running at resolution %ix%i.")
                                        % info.width % info.height % info_man_->getCameraInfo().width % info_man_->getCameraInfo().height).str();
    ROS_ERROR("%s", rsp.status_message.c_str());
    return true;
  }

  std::string cam_name = cam_.getCameraName();

  std::stringstream yml_stream;
  if(!camera_calibration_parsers::writeCalibrationYml(yml_stream, cam_name, info)) {
    rsp.status_message = "Error formatting camera_info for storage.";
    rsp.success = false;
  }
  else {
    std::string yml = yml_stream.str();

    VmbInt64_t lut_size = cam_.getLutMemorySize();

    if(yml.size() > lut_size / 2) {
      rsp.success = false;
      rsp.status_message = "Unable to write camera_info to camera memory, exceeded storage capacity.";
    }
    else {
      VmbErrorType status;
      UcharVector data;
      data.resize(static_cast<size_t>(lut_size / 2), ' ');

      int index = 0;
      for(std::string::iterator it = yml.begin() ; it != yml.end() ; it++) {
        data[index] = *it;
        index++;
      }

      status = cam_.saveCameraMemory(data);
      if(status != VmbErrorSuccess) {
        rsp.success = false;
        rsp.status_message = "Undefinded write to memory problem.";
      }
      else {
        rsp.success = true;
        info_man_->setCameraInfo(info);
      }
    }
  }

  if(!rsp.success)
    ROS_ERROR("%s", rsp.status_message.c_str());

  return true;
}


};
