#ifndef VIMBA_ROS_H
#define VIMBA_ROS_H

#include <VimbaCPP/Include/VimbaCPP.h>

namespace avt_vimba_ros {
class VimbaROS{
public:
	VimbaROS(ros::NodeHandle nh, ros::NodeHandle nhp);
	int getWidth();
  int getHeight();
  VmbPixelFormatType getPixelFormat();
private:

	// Vimba singleton
	AVT::VmbAPI::VimbaSystem& vimba_;
	// The currently streaming camera
  AVT::VmbAPI::CameraPtr camera_ptr_;
  // The current pixel format
  VmbInt64_t pixel_format_;
  // The current width
  VmbInt64_t width_;
  // The current height
  VmbInt64_t height_;

	// A readable value for every Vimba error code
  std::map<VmbErrorType, std::string> error_code_to_message_;
  // Translates Vimba error codes to readable error messages
	std::string errorCodeToMessage( VmbErrorType eErr );

	void initApi(void);
	void listAvailableCameras(void);
	std::string interfaceToString( VmbInterfaceType interfaceType );
	AVT::VmbAPI::CameraPtr openCamera(std::string id);
	void printAllCameraFeatures(AVT::VmbAPI::CameraPtr camera);
};
}

#endif
