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

#ifndef AVT_VIMBA_API_H
#define AVT_VIMBA_API_H

#include <VimbaCPP/Include/VimbaCPP.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

#include <string>
#include <map>

using AVT::VmbAPI::VimbaSystem;
using AVT::VmbAPI::CameraPtr;
using AVT::VmbAPI::FramePtr;

namespace avt_vimba_camera {
class AvtVimbaApi {
 public:
  AvtVimbaApi(void) : vs(VimbaSystem::GetInstance()) {}

  void start() {
    VmbErrorType err = vs.Startup();
    if (VmbErrorSuccess == err) {
      ROS_INFO_STREAM("[Vimba System]: AVT Vimba System initialized successfully");
      listAvailableCameras();
    } else {
      ROS_ERROR_STREAM("[Vimba System]: Could not start Vimba system: "
        << errorCodeToMessage(err));
    }
  }

  /** Translates Vimba error codes to readable error messages
    *
    * @param error Vimba error tyme
    * @return readable string error
    *
    **/
  std::string errorCodeToMessage(VmbErrorType error) {
    std::map<VmbErrorType, std::string> error_msg;
    error_msg[VmbErrorSuccess]        = "Success.";
    error_msg[VmbErrorApiNotStarted]  = "API not started.";
    error_msg[VmbErrorNotFound]       = "Not found.";
    error_msg[VmbErrorBadHandle]      = "Invalid handle ";
    error_msg[VmbErrorDeviceNotOpen]  = "Device not open.";
    error_msg[VmbErrorInvalidAccess]  = "Invalid access.";
    error_msg[VmbErrorBadParameter]   = "Bad parameter.";
    error_msg[VmbErrorStructSize]     = "Wrong DLL version.";
    error_msg[VmbErrorWrongType]      = "Wrong type.";
    error_msg[VmbErrorInvalidValue]   = "Invalid value.";
    error_msg[VmbErrorTimeout]        = "Timeout.";
    error_msg[VmbErrorOther]          = "TL error.";
    error_msg[VmbErrorInvalidCall]    = "Invalid call.";
    error_msg[VmbErrorNoTL]           = "TL not loaded.";
    error_msg[VmbErrorNotImplemented] = "Not implemented.";
    error_msg[VmbErrorNotSupported]   = "Not supported.";
    error_msg[VmbErrorResources]      = "Resource not available.";
    error_msg[VmbErrorInternalFault]  = "Unexpected fault in VmbApi or driver.";
    error_msg[VmbErrorMoreData]       = "More data returned than memory provided.";

    std::map<VmbErrorType, std::string>::const_iterator iter =
      error_msg.find(error);
    if ( error_msg.end() != iter ) {
      return iter->second;
    }
    return "Unsupported error code passed.";
  }

  bool frameToImage(const FramePtr vimba_frame_ptr, sensor_msgs::Image& image) {
    VmbPixelFormatType pixel_format;
    VmbUint32_t width, height, nSize;

    vimba_frame_ptr->GetWidth(width);
    vimba_frame_ptr->GetHeight(height);
    vimba_frame_ptr->GetPixelFormat(pixel_format);
    vimba_frame_ptr->GetImageSize(nSize);

    VmbUint32_t step = nSize / height;

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
      ROS_ERROR_STREAM("[" << ros::this_node::getName()
        << "]: Could not GetImage. "
        << "\n Error: " << errorCodeToMessage(err));
    }
    return res;
  }

 private:
  VimbaSystem& vs;

  void listAvailableCameras(void) {
    std::string name;
    CameraPtrVector cameras;
    if (VmbErrorSuccess == vs.Startup()) {
      if (VmbErrorSuccess == vs.GetCameras(cameras)) {
        for (CameraPtrVector::iterator iter = cameras.begin();
        cameras.end() != iter;
        ++iter) {
          if (VmbErrorSuccess == (*iter)->GetName(name)) {
            ROS_DEBUG_STREAM("[" << ros::this_node::getName() << "]: Found camera: ");
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
          ROS_INFO_STREAM("\t/// Camera Name: " << strName);
          ROS_INFO_STREAM("\t/// Model Name: " << strModelname);
          ROS_INFO_STREAM("\t/// Camera ID: " << strID);
          ROS_INFO_STREAM("\t/// Serial Number: " << strSerialNumber);
          ROS_INFO_STREAM("\t/// @ Interface ID: " << strInterfaceID);

        }
      } else {
        ROS_WARN("Could not get cameras from Vimba System");
      }
    } else {
      ROS_WARN("Could not start Vimba System");
    }
  }
};
}
#endif
