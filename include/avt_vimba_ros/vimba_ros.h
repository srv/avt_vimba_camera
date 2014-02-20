#ifndef VIMBA_ROS_H
#define VIMBA_ROS_H

#include <VimbaCPP/Include/VimbaCPP.h>

namespace avt_vimba_ros {
class VimbaROS{
public:
	VimbaROS(ros::NodeHandle nh, ros::NodeHandle nhp);
private:
	void init(void);
	void listAvailableCameras(void);
	std::string interfaceToString( VmbInterfaceType interfaceType );
	AVT::VmbAPI::CameraPtr openCamera(std::string id);
	void printAllCameraFeatures(AVT::VmbAPI::CameraPtr camera);
};
}

#endif
