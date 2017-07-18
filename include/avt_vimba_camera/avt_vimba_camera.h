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

#ifndef AVT_VIMBA_CAMERA_H
#define AVT_VIMBA_CAMERA_H

#include <VimbaCPP/Include/VimbaCPP.h>

#include <avt_vimba_camera/AvtVimbaCameraConfig.h>
#include <avt_vimba_camera/frame_observer.h>
#include <avt_vimba_camera/avt_vimba_api.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <boost/function.hpp>
#include <boost/thread.hpp>

#include <string>

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

enum CameraState {
  OPENING,
  IDLE,
  CAMERA_NOT_FOUND,
  FORMAT_ERROR,
  ERROR,
  OK
};

class AvtVimbaCamera {
 public:
  AvtVimbaCamera();
  AvtVimbaCamera(std::string name);
  void start(std::string ip_str, std::string guid_str, bool debug_prints = true);
  void stop();
  double getTimestamp(void);
  bool resetTimestamp(void);
  double getDeviceTemp(void);

  CameraPtr getCameraPtr(void) {
    return vimba_camera_ptr_;
  }

  int getWidth();
  int getHeight();
  int getMaxWidth();
  int getMaxHeight();

  // Pass callback function pointer
  typedef boost::function<void (const FramePtr)> frameCallbackFunc;
  void setCallback(frameCallbackFunc callback = &avt_vimba_camera::AvtVimbaCamera::defaultFrameCallback) {
    userFrameCallback = callback;
  }

  typedef avt_vimba_camera::AvtVimbaCameraConfig Config;
  void updateConfig(Config& config);
  void startImaging(void);
  void stopImaging(void);
  bool isOpened(void) { return opened_; }

 private:
  Config config_;

  AvtVimbaApi api_;
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

  // Mutex
  boost::mutex config_mutex_;

  bool opened_;
  bool streaming_;
  bool on_init_;
  bool show_debug_prints_;
  std::string name_;

  diagnostic_updater::Updater updater_;
  CameraState camera_state_;
  std::string diagnostic_msg_;

  // ROS params
  int num_frames_;
  std::string guid_;
  std::string frame_id_;
  std::string trigger_source_;
  int trigger_source_int_;

  CameraPtr openCamera(std::string id_str);

  frameCallbackFunc userFrameCallback;
  void frameCallback(const FramePtr vimba_frame_ptr);
  void defaultFrameCallback(const FramePtr vimba_frame_ptr) {
    std::cout << "[AvtVimbaCamera] No frame callback provided." << std::endl;
  }

  template<typename T>
  bool setFeatureValue(const std::string& feature_str, const T& val);
  template<typename T>
  bool getFeatureValue(const std::string& feature_str, T& val);
  bool getFeatureValue(const std::string& feature_str, std::string& val);
  bool runCommand(const std::string& command_str);
  std::string interfaceToString(VmbInterfaceType interfaceType);
  std::string accessModeToString(VmbAccessModeType modeType);
  int getTriggerModeInt(std::string mode_str);
  void printAllCameraFeatures(const CameraPtr& camera);

  void updateAcquisitionConfig(Config& config);
  void updateExposureConfig(Config& config);
  void updateGainConfig(Config& config);
  void updateWhiteBalanceConfig(Config& config);
  void updateImageModeConfig(Config& config);
  void updateROIConfig(Config& config);
  void updateBandwidthConfig(Config& config);
  void updatePixelFormatConfig(Config& config);
  void updatePtpModeConfig(Config& config);
  void updateGPIOConfig(Config& config);
  void updateIrisConfig(Config& config);

  void getCurrentState(diagnostic_updater::DiagnosticStatusWrapper &stat);

};
}
#endif
