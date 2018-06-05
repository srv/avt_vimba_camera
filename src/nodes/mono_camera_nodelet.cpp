#include <pluginlib/class_list_macros.h>
#include "avt_vimba_camera/mono_camera_nodelet.h"

namespace avt_vimba_camera
{
    void MonoCameraNodelet::onInit()
    {
        NODELET_DEBUG("Initializing nodelet...");
				camera_ = new MonoCamera(getMTNodeHandle(), getMTPrivateNodeHandle());
    }

    MonoCameraNodelet::~MonoCameraNodelet()
    {
      	delete camera_;
    }
}

PLUGINLIB_EXPORT_CLASS(avt_vimba_camera::MonoCameraNodelet, nodelet::Nodelet)
