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

#include <avt_vimba_camera/avt_vimba_camera.h>
#include <avt_vimba_camera/avt_vimba_api.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <boost/lexical_cast.hpp>
#include <sstream>

#define DEBUG_PRINTS 1

namespace avt_vimba_camera {

static const char* AutoMode[] = {
  "Off",
  "Once",
  "Continuous"};
static const char* TriggerMode[] = {
  "Freerun",
  "FixedRate",
  "Software",
  "Line1",
  "Line2",
  "Line3",
  "Line4" };
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
static const char* BalanceRatioMode[] = {
  "Red",
  "Blue"};
static const char* FeatureDataType[] = {
  "Unknown",
  "int",
  "float",
  "enum",
  "string",
  "bool"};

AvtVimbaCamera::AvtVimbaCamera() {
  // Init global variables
  running_ = false;
  first_run_ = true;
}

void AvtVimbaCamera::start(std::string ip_str, std::string guid_str) {
  if (running_) return;

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

  std::string trigger_source;
  getFeatureValue("TriggerSource", trigger_source);
  ROS_INFO_STREAM("[" << ros::this_node::getName()
                  << "]: Trigger mode is " << trigger_source);

  int trigger_source_int = getTriggerModeInt(trigger_source);

  if (trigger_source_int == Freerun   ||
      trigger_source_int == FixedRate ||
      trigger_source_int == SyncIn1) {
    // Create a frame observer for this camera
    vimba_frame_observer_ptr_ = new FrameObserver(vimba_camera_ptr_,
      boost::bind(&avt_vimba_camera::AvtVimbaCamera::frameCallback, this, _1));
    // Start streaming
    VmbErrorType err =
      vimba_camera_ptr_->StartContinuousImageAcquisition(1,  // num_frames_,
      IFrameObserverPtr(vimba_frame_observer_ptr_));
    if (VmbErrorSuccess == err) {
      ROS_INFO_STREAM("[" << ros::this_node::getName()
        << "]: Starting continuous image acquisition...");
    } else {
      ROS_ERROR_STREAM("[" << ros::this_node::getName()
        << "]: Could not start continuous image acquisition. "
        << "\n Error: " << api_.errorCodeToMessage(err));
    }
  } else {
    ROS_ERROR_STREAM("Trigger mode " <<
                    TriggerMode[trigger_source_int] <<
                    " not implemented.");
  }
}

void AvtVimbaCamera::updateConfig(Config& config) {
  updateExposureConfig(config);
  updateGainConfig(config);
  updateWhiteBalanceConfig(config);
  updateImageModeConfig(config);
  updateROIConfig(config);
  updateBandwidthConfig(config);
  updateGPIOConfig(config);
  updatePtpModeConfig(config);
  config_ = config;
}

void AvtVimbaCamera::stop() {
  if (!running_) return;
  vimba_camera_ptr_->Close();
  running_ = false;
}

CameraPtr AvtVimbaCamera::openCamera(std::string id_str) {
  // Details:   The ID might be one of the following:
  //            "IP:169.254.12.13",
  //            "MAC:000f31000001",
  //            or a plain serial number: "1234567890".

  CameraPtr camera;
  VimbaSystem& vimba_system(VimbaSystem::GetInstance());
  VmbErrorType err = vimba_system.GetCameraByID(id_str.c_str(), camera);
  if (VmbErrorSuccess == err) {
    std::string cam_id, cam_name, cam_model, cam_sn, cam_int_id;
    VmbInterfaceType cam_int_type;
    VmbAccessModeType accessMode;  // = VmbAccessModeNone;
    camera->GetID(cam_id);
    camera->GetName(cam_name);
    camera->GetModel(cam_model);
    camera->GetSerialNumber(cam_sn);
    camera->GetInterfaceID(cam_int_id);
    camera->GetInterfaceType(cam_int_type);
    err = camera->GetPermittedAccess(accessMode);

    ROS_INFO_STREAM("[" << ros::this_node::getName() << "]: Opened camera with"
    << "\n\t\t * Name     : " << cam_name
    << "\n\t\t * Model    : " << cam_model
    << "\n\t\t * ID       : " << cam_id
    << "\n\t\t * S/N      : " << cam_sn
    << "\n\t\t * Itf. ID  : " << cam_int_id
    << "\n\t\t * Itf. Type: " << interfaceToString(cam_int_type)
    << "\n\t\t * Access   : " << accessModeToString(accessMode));

    err = camera->Open(VmbAccessModeFull);
    if (VmbErrorSuccess == err) {
      // printAllCameraFeatures(camera);
    } else {
      ROS_ERROR_STREAM("[" << ros::this_node::getName()
        << "]: Could not get camera " << id_str
        << "\n Error: " << api_.errorCodeToMessage(err));
    }
  } else {
    ROS_ERROR_STREAM("[" << ros::this_node::getName()
      << "]: Could not get camera " << id_str
      << "\n Error: " << api_.errorCodeToMessage(err));
  }
  running_ = true;
  return camera;
}

void AvtVimbaCamera::frameCallback(const FramePtr vimba_frame_ptr) {
  // Call the callback implemented by other classes
  userFrameCallback(vimba_frame_ptr);

  // Queue the frame so that we can receive a new one.
  vimba_camera_ptr_->QueueFrame(vimba_frame_ptr);
}

// Template function to GET a feature value from the camera
template<typename T>
bool AvtVimbaCamera::getFeatureValue(const std::string& feature_str, T& val) {
  VmbErrorType err;
  FeaturePtr vimba_feature_ptr;
  VmbFeatureDataType data_type;
  err = vimba_camera_ptr_->GetFeatureByName(feature_str.c_str(),
                                            vimba_feature_ptr);
  if (VmbErrorSuccess == err) {
    bool readable;
    vimba_feature_ptr->IsReadable(readable);
    if (readable) {
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
                    << api_.errorCodeToMessage(err));
        }
      }
    } else {
      ROS_ERROR_STREAM("[" << ros::this_node::getName() << "]: Feature "
                       << feature_str
                       << " is not readable.");
    }
  } else {
    ROS_ERROR_STREAM("[" << ros::this_node::getName()
      << "]: Could not get feature " << feature_str);
  }
  if (DEBUG_PRINTS)
    ROS_INFO_STREAM("Asking for feature "
      << feature_str << " with datatype "
      << FeatureDataType[data_type]
      << " and value " << val);
  return (VmbErrorSuccess == err);
}

// Function to GET a feature value from the camera, overloaded to strings
bool AvtVimbaCamera::getFeatureValue(const std::string& feature_str,
                                     std::string& val) {
  VmbErrorType err;
  FeaturePtr vimba_feature_ptr;
  VmbFeatureDataType data_type;
  err = vimba_camera_ptr_->GetFeatureByName(feature_str.c_str(),
                                            vimba_feature_ptr);
  if (VmbErrorSuccess == err) {
    bool readable;
    vimba_feature_ptr->IsReadable(readable);
    if (readable) {
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
                    << api_.errorCodeToMessage(err));
        }
      }
    } else {
      ROS_ERROR_STREAM("[" << ros::this_node::getName() << "]: Feature "
                       << feature_str
                       << " is not readable.");
    }
  } else {
    ROS_ERROR_STREAM("[" << ros::this_node::getName()
      << "]: Could not get feature " << feature_str);
  }
  ROS_DEBUG_STREAM("Asking for feature " << feature_str
    << " with datatype " << FeatureDataType[data_type]
    << " and value " << val);
  return (VmbErrorSuccess == err);
}

// Template function to SET a feature value from the camera
template<typename T>
bool AvtVimbaCamera::setFeatureValue(const std::string& feature_str,
                                     const T& val) {
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
              if (available) {
                err = vimba_feature_ptr->SetValue(val);
              } else {
                ROS_ERROR_STREAM("[" << ros::this_node::getName()
                  << "]: Feature " << feature_str << " is available now.");
              }
            } else {
              ROS_ERROR_STREAM("[" << ros::this_node::getName() << "]: Feature "
                << feature_str << ": value unavailable\n\tERROR "
                << api_.errorCodeToMessage(err));
            }
          } else {
            err = vimba_feature_ptr->SetValue(val);
          }
        } else {
          ROS_ERROR_STREAM("[" << ros::this_node::getName() << "]: Feature "
            << feature_str << ": Bad data type\n\tERROR "
            << api_.errorCodeToMessage(err));
        }
      } else {
        ROS_ERROR_STREAM("[" << ros::this_node::getName() << "]: Feature "
                         << feature_str
                         << " is not writable.");
      }
    } else {
      ROS_ERROR_STREAM("[" << ros::this_node::getName() << "]: Feature "
        << feature_str << ": ERROR " << api_.errorCodeToMessage(err));
    }
  } else {
    ROS_ERROR_STREAM("[" << ros::this_node::getName()
      << "]: Could not get feature " << feature_str
      << "\n Error: " << api_.errorCodeToMessage(err));
  }
  return (VmbErrorSuccess == err);
}


// Template function to RUN a command
bool AvtVimbaCamera::runCommand(const std::string& command_str) {
  FeaturePtr feature_ptr;
  if (VmbErrorSuccess ==
    vimba_camera_ptr_->GetFeatureByName(command_str.c_str(), feature_ptr)) {
    if ( VmbErrorSuccess == feature_ptr->RunCommand() ) {
      bool is_command_done = false;
      do {
        if ( VmbErrorSuccess != feature_ptr->IsCommandDone(is_command_done) ) {
          break;
        }
        ROS_INFO_STREAM_THROTTLE(1, "Waiting for command "
          << command_str.c_str() << "...");
      } while ( false == is_command_done );
      ROS_INFO_STREAM("Command " << command_str.c_str() << " done!");
    }
  }
}

std::string AvtVimbaCamera::interfaceToString(VmbInterfaceType interfaceType) {
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

std::string AvtVimbaCamera::accessModeToString(VmbAccessModeType modeType) {
  std::string s;
  if (modeType & VmbAccessModeFull)
    s = std::string("Read and write access");
  else if (modeType & VmbAccessModeRead)
    s = std::string("Only read access");
  else if (modeType & VmbAccessModeConfig)
    s = std::string("Device configuration access");
  else if (modeType & VmbAccessModeLite)
    s = std::string("Device read/write access without feature access (only addresses)");
  else if (modeType & VmbAccessModeNone)
    s = std::string("No access");
  return s;
}

int AvtVimbaCamera::getTriggerModeInt(std::string mode_str) {
  int mode;
  if (mode_str == TriggerMode[Freerun]) {
    mode = Freerun;
  } else if (mode_str == TriggerMode[FixedRate]) {
    mode = FixedRate;
  } else if (mode_str == TriggerMode[Software]) {
    mode = Software;
  } else if (mode_str == TriggerMode[SyncIn1]) {
    mode = SyncIn1;
  } else if (mode_str == TriggerMode[SyncIn2]) {
    mode = SyncIn2;
  } else if (mode_str == TriggerMode[SyncIn3]) {
    mode = SyncIn3;
  } else if (mode_str == TriggerMode[SyncIn4]) {
    mode = SyncIn4;
  }
  return mode;
}

void AvtVimbaCamera::printAllCameraFeatures(CameraPtr camera) {
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

/** Change the Trigger configuration */
void AvtVimbaCamera::updateAcquisitionConfig(Config& config) {
  bool changed = false;
  if (config.acquisition_mode != config_.acquisition_mode || first_run_) {
    changed = true;
    setFeatureValue("AcquisitionMode", config.acquisition_mode.c_str());
  }
  if (config.acquisition_rate != config_.acquisition_rate || first_run_) {
    changed = true;
    setFeatureValue("AcquisitionFrameRateAbs",
                    static_cast<float>(config.acquisition_rate));
  }
  if (config.trigger_mode != config_.trigger_mode || first_run_) {
    changed = true;
    setFeatureValue("TriggerMode", config.trigger_mode.c_str());
  }
  if (config.trigger_selector != config_.trigger_selector || first_run_) {
    changed = true;
    setFeatureValue("TriggerSelector", config.trigger_selector.c_str());
  }
  if (config.trigger_source != config_.trigger_source || first_run_) {
    changed = true;
    setFeatureValue("TriggerSource", config.trigger_source.c_str());
  }
  if (config.trigger_activation != config_.trigger_activation || first_run_) {
    changed = true;
    setFeatureValue("TriggerActivation", config.trigger_activation.c_str());
  }
  if (config.trigger_delay != config_.trigger_delay || first_run_) {
    changed = true;
    setFeatureValue("TriggerDelayAbs", config.trigger_delay);
  }
  if(changed && DEBUG_PRINTS){
    ROS_INFO_STREAM("New Trigger config: "
      << "\n\tAcquisitionMode         : " << config.acquisition_mode   << " was " << config_.acquisition_mode
      << "\n\tAcquisitionFrameRateAbs : " << config.acquisition_rate   << " was " << config_.acquisition_rate
      << "\n\tTriggerMode             : " << config.trigger_mode       << " was " << config_.trigger_mode
      << "\n\tTriggerSource           : " << config.trigger_source     << " was " << config_.trigger_source
      << "\n\tTriggerSelector         : " << config.trigger_selector   << " was " << config_.trigger_selector
      << "\n\tTriggerActivation       : " << config.trigger_activation << " was " << config_.trigger_activation
      << "\n\tTriggerDelayAbs         : " << config.trigger_delay      << " was " << config_.trigger_delay);
  }
}

/** Change the Exposure configuration */
void AvtVimbaCamera::updateExposureConfig(Config& config) {
  bool changed = false;
  if (config.exposure != config_.exposure || first_run_) {
    changed = true;
    setFeatureValue("ExposureTimeAbs", static_cast<float>(config.exposure));
  }
  if (config.exposure_auto != config_.exposure_auto || first_run_) {
    changed = true;
    setFeatureValue("ExposureAuto", config.exposure_auto.c_str());
  }
  if (config.exposure_auto_tol != config_.exposure_auto_tol || first_run_) {
    changed = true;
    setFeatureValue("ExposureAutoAdjustTol",
                    static_cast<VmbInt64_t>(config.exposure_auto_tol));
  }
  if (config.exposure_auto_max != config_.exposure_auto_max || first_run_) {
    changed = true;
    setFeatureValue("ExposureAutoMax",
                    static_cast<VmbInt64_t>(config.exposure_auto_max));
  }
  if (config.exposure_auto_min != config_.exposure_auto_min || first_run_) {
    changed = true;
    setFeatureValue("ExposureAutoMin",
                    static_cast<VmbInt64_t>(config.exposure_auto_min));
  }
  if (config.exposure_auto_outliers != config_.exposure_auto_outliers || first_run_) {
    changed = true;
    setFeatureValue("ExposureAutoOutliers",
                    static_cast<VmbInt64_t>(config.exposure_auto_outliers));
  }
  if (config.exposure_auto_rate != config_.exposure_auto_rate || first_run_) {
    changed = true;
    setFeatureValue("ExposureAutoRate",
                    static_cast<VmbInt64_t>(config.exposure_auto_rate));
  }
  if (config.exposure_auto_target != config_.exposure_auto_target || first_run_) {
    changed = true;
    setFeatureValue("ExposureAutoTarget",
                    static_cast<VmbInt64_t>(config.exposure_auto_target));
  }
  if(changed && DEBUG_PRINTS){
    ROS_INFO_STREAM("New Exposure config: "
      << "\n\tExposureTimeAbs       : " << config.exposure               << " was " << config_.exposure
      << "\n\tExposureAuto          : " << config.exposure_auto          << " was " << config_.exposure_auto
      << "\n\tExposureAutoAdjustTol : " << config.exposure_auto_tol      << " was " << config_.exposure_auto_tol
      << "\n\tExposureAutoMax       : " << config.exposure_auto_max      << " was " << config_.exposure_auto_max
      << "\n\tExposureAutoMin       : " << config.exposure_auto_min      << " was " << config_.exposure_auto_min
      << "\n\tExposureAutoOutliers  : " << config.exposure_auto_outliers << " was " << config_.exposure_auto_outliers
      << "\n\tExposureAutoRate      : " << config.exposure_auto_rate     << " was " << config_.exposure_auto_rate
      << "\n\tExposureAutoTarget    : " << config.exposure_auto_target   << " was " << config_.exposure_auto_target);
  }
}

/** Change the Gain configuration */
void AvtVimbaCamera::updateGainConfig(Config& config) {
  bool changed = false;
  if (config.gain != config_.gain || first_run_) {
    changed = true;
    setFeatureValue("Gain", static_cast<float>(config.gain));
  }
  if (config.gain_auto != config_.gain_auto || first_run_) {
    changed = true;
    setFeatureValue("GainAuto", config.gain_auto.c_str());
  }
  if (config.gain_auto_tol != config_.gain_auto_tol || first_run_) {
    changed = true;
    setFeatureValue("GainAutoAdjustTol",
                    static_cast<VmbInt64_t>(config.gain_auto_tol));
  }
  if (config.gain_auto_max != config_.gain_auto_max || first_run_) {
    changed = true;
    setFeatureValue("GainAutoMax", static_cast<float>(config.gain_auto_max));
  }
  if (config.gain_auto_min != config_.gain_auto_min || first_run_) {
    changed = true;
    setFeatureValue("GainAutoMin",
                    static_cast<VmbInt64_t>(config.gain_auto_min));
  }
  if (config.gain_auto_outliers != config_.gain_auto_outliers || first_run_) {
    changed = true;
    setFeatureValue("GainAutoMin",
                    static_cast<VmbInt64_t>(config.gain_auto_outliers));
  }
  if (config.gain_auto_rate != config_.gain_auto_rate || first_run_) {
    changed = true;
    setFeatureValue("GainAutoOutliers",
                    static_cast<VmbInt64_t>(config.gain_auto_rate));
  }
  if (config.gain_auto_target != config_.gain_auto_target || first_run_) {
    changed = true;
    setFeatureValue("GainAutoRate", static_cast<VmbInt64_t>(config.gain_auto_target));
  }
  if(changed && DEBUG_PRINTS){
    ROS_INFO_STREAM("New Exposure config: "
      << "\n\tGain              : " << config.gain               << " was " << config_.gain
      << "\n\tGainAuto          : " << config.gain_auto          << " was " << config_.gain_auto
      << "\n\tGainAutoAdjustTol : " << config.gain_auto_tol      << " was " << config_.gain_auto_tol
      << "\n\tGainAutoMax       : " << config.gain_auto_max      << " was " << config_.gain_auto_max
      << "\n\tGainAutoMin       : " << config.gain_auto_min      << " was " << config_.gain_auto_min
      << "\n\tGainAutoOutliers  : " << config.gain_auto_outliers << " was " << config_.gain_auto_outliers
      << "\n\tGainAutoRate      : " << config.gain_auto_rate     << " was " << config_.gain_auto_rate
      << "\n\tGainAutoTarget    : " << config.gain_auto_target   << " was " << config_.gain_auto_target);
  }
}

/** Change the White Balance configuration */
void AvtVimbaCamera::updateWhiteBalanceConfig(Config& config){
  bool changed = false;
  if (config.balance_ratio_abs != config_.balance_ratio_abs || first_run_) {
    changed = true;
    setFeatureValue("BalanceRatioAbs", static_cast<float>(config.balance_ratio_abs));
  }
  if (config.balance_ratio_selector != config_.balance_ratio_selector || first_run_) {
    changed = true;
    setFeatureValue("BalanceRatioSelector", config.balance_ratio_selector.c_str());
  }
  if (config.whitebalance_auto != config_.whitebalance_auto || first_run_) {
    changed = true;
    setFeatureValue("BalanceWhiteAuto", config.whitebalance_auto.c_str());
  }
  if (config.whitebalance_auto_tol != config_.whitebalance_auto_tol || first_run_) {
    changed = true;
    setFeatureValue("BalanceWhiteAutoAdjustTol", static_cast<VmbInt64_t>(config.whitebalance_auto_tol));
  }
  if (config.whitebalance_auto_rate != config_.whitebalance_auto_rate || first_run_) {
    changed = true;
    setFeatureValue("BalanceWhiteAutoRate", static_cast<VmbInt64_t>(config.whitebalance_auto_rate));
  }
  if(changed && DEBUG_PRINTS){
    ROS_INFO_STREAM("New ROI config: "
      << "\n\tBalanceRatioAbs           : " << config.balance_ratio_abs      << " was " << config_.balance_ratio_abs
      << "\n\tBalanceRatioSelector      : " << config.balance_ratio_selector << " was " << config_.balance_ratio_selector
      << "\n\tBalanceWhiteAuto          : " << config.whitebalance_auto      << " was " << config_.whitebalance_auto
      << "\n\tBalanceWhiteAutoAdjustTol : " << config.whitebalance_auto_tol  << " was " << config_.whitebalance_auto_tol
      << "\n\tBalanceWhiteAutoRate      : " << config.whitebalance_auto_rate << " was " << config_.whitebalance_auto_rate);
  }
}

/** Change the Binning and Decimation configuration */
void AvtVimbaCamera::updatePtpModeConfig(Config& config) {
  bool changed = false;
  if (config.ptp_mode != config_.ptp_mode || first_run_) {
    changed = true;
    setFeatureValue("PtpMode", config.ptp_mode.c_str());
  }

  if(changed && DEBUG_PRINTS){
    ROS_INFO_STREAM("New PTP config: "
      << "\n\tPtpMode                   : " << config.ptp_mode << " was " << config_.ptp_mode);
  }

}

/** Change the Binning and Decimation configuration */
void AvtVimbaCamera::updateImageModeConfig(Config& config) {
  bool changed = false;
  if (config.decimation_x != config_.decimation_x || first_run_) {
    changed = true;
    setFeatureValue("DecimationHorizontal",
                    static_cast<VmbInt64_t>(config.decimation_x));
  }
  if (config.decimation_y != config_.decimation_y || first_run_) {
    changed = true;
    setFeatureValue("DecimationVertical", static_cast<VmbInt64_t>(config.decimation_y));
  }
  if (config.binning_x != config_.binning_x || first_run_) {
    changed = true;
    setFeatureValue("BinningHorizontal", static_cast<VmbInt64_t>(config.binning_x));
  }
  if (config.binning_y != config_.binning_y || first_run_) {
    changed = true;
    setFeatureValue("BinningVertical", static_cast<VmbInt64_t>(config.binning_y));
  }
  if(changed && DEBUG_PRINTS){
    ROS_INFO_STREAM("New Image config: "
      << "\n\tDecimationHorizontal : " << config.decimation_x << " was " << config_.decimation_x
      << "\n\tDecimationVertical   : " << config.decimation_y << " was " << config_.decimation_y
      << "\n\tBinningHorizontal    : " << config.binning_x    << " was " << config_.binning_x
      << "\n\tBinningVertical      : " << config.binning_y    << " was " << config_.binning_y);
  }
}

/** Change the ROI configuration */
void AvtVimbaCamera::updateROIConfig(Config& config) {
  bool changed = false;

  // Region of interest configuration
  // Make sure ROI fits in image

  int max_width, max_height;
  getFeatureValue("WidthMax",max_width);
  getFeatureValue("HeightMax",max_height);

  config.width        = std::min(config.width,  (int)max_width);
  config.height       = std::min(config.height, (int)max_height);
  config.roi_offset_x = std::min(config.roi_offset_x, config.width - 1);
  config.roi_offset_y = std::min(config.roi_offset_y, config.height - 1);
  config.roi_width    = std::min(config.roi_width,  config.width  - config.roi_offset_x);
  config.roi_height   = std::min(config.roi_height, config.height - config.roi_offset_y);
  // If width or height is 0, set it as large as possible
  int width  = config.roi_width  ? config.roi_width  : max_width  - config.roi_offset_x;
  int height = config.roi_height ? config.roi_height : max_height - config.roi_offset_y;

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

  if (config.roi_offset_x != config_.roi_offset_x || first_run_) {
    changed = true;
    setFeatureValue("OffsetX", static_cast<VmbInt64_t>(config.roi_offset_x));
  }
  if (config.roi_offset_y != config_.roi_offset_y || first_run_) {
    changed = true;
    setFeatureValue("OffsetY", static_cast<VmbInt64_t>(config.roi_offset_y));
  }
  if (config.width != config_.width || first_run_) {
    changed = true;
    setFeatureValue("Width", static_cast<VmbInt64_t>(config.width));
  }
  if (config.height != config_.height || first_run_) {
    changed = true;
    setFeatureValue("Height", static_cast<VmbInt64_t>(config.height));
  }

  if(changed && DEBUG_PRINTS){
    ROS_INFO_STREAM("New ROI config: "
      << "\n\tOffsetX : " << config.roi_offset_x << " was " << config_.roi_offset_x
      << "\n\tOffsetY : " << config.roi_offset_y << " was " << config_.roi_offset_y
      << "\n\tWidth   : " << config.width        << " was " << config_.width
      << "\n\tHeight  : " << config.height       << " was " << config_.height);
  }
}

/** Change the Bandwidth configuration */
void AvtVimbaCamera::updateBandwidthConfig(Config& config) {
  bool changed = false;
  if (config.stream_bytes_per_second
      != config_.stream_bytes_per_second || first_run_) {
    changed = true;
    setFeatureValue("StreamBytesPerSecond",
                    static_cast<VmbInt64_t>(config.stream_bytes_per_second));
  }
  if(changed && DEBUG_PRINTS){
    ROS_INFO_STREAM("New Bandwidth config: "
      << "\n\tStreamBytesPerSecond : " << config.stream_bytes_per_second << " was " << config_.stream_bytes_per_second);
  }
}

/** Change the Pixel Format configuration */
void AvtVimbaCamera::updatePixelFormatConfig(Config& config) {
  bool changed = false;
  if (config.pixel_format != config_.pixel_format || first_run_) {
    changed = true;
    setFeatureValue("PixelFormat", config.pixel_format.c_str());
  }
  if(changed && DEBUG_PRINTS){
    ROS_INFO_STREAM("New PixelFormat config: "
      << "\n\tPixelFormat : " << config.pixel_format << " was " << config_.pixel_format);
  }
}

/** Change the GPIO configuration */
void AvtVimbaCamera::updateGPIOConfig(Config& config) {
  bool changed = false;
  if (config.sync_in_selector
      != config_.sync_in_selector || first_run_) {
    changed = true;
    setFeatureValue("SyncInSelector", config.sync_in_selector.c_str());
  }
  if (config.sync_out_polarity
      != config_.sync_out_polarity || first_run_) {
    changed = true;
    setFeatureValue("SyncOutPolarity", config.sync_out_polarity.c_str());
  }
  if (config.sync_out_selector
      != config_.sync_out_selector || first_run_) {
    changed = true;
    setFeatureValue("SyncOutSelector", config.sync_out_selector.c_str());
  }
  if (config.sync_out_source
      != config_.sync_out_source || first_run_) {
    changed = true;
    setFeatureValue("SyncOutSource", config.sync_out_source.c_str());
  }
  if(changed && DEBUG_PRINTS){
    ROS_INFO_STREAM("New Bandwidth config: "
      << "\n\tSyncInSelector  : " << config.sync_in_selector  << " was " << config_.sync_in_selector
      << "\n\tSyncOutPolarity : " << config.sync_out_polarity << " was " << config_.sync_out_polarity
      << "\n\tSyncOutSelector : " << config.sync_out_selector << " was " << config_.sync_out_selector
      << "\n\tSyncOutSource   : " << config.sync_out_source   << " was " << config_.sync_out_source);
  }
}
}
