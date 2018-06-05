#include <pluginlib/class_list_macros.h>
#include "avt_vimba_camera/stereo_camera_nodelet.h"
#include <boost/thread.hpp>

namespace avt_vimba_camera
{
    void StereoCameraNodelet::onInit()
    {
        NODELET_DEBUG("Initializing nodelet...");
				camera_ = new StereoCamera(getMTNodeHandle(), getMTPrivateNodeHandle());
        boost::thread stereoThread(&avt_vimba_camera::StereoCamera::run, camera_);
    }

    StereoCameraNodelet::~StereoCameraNodelet()
    {
        delete camera_;
    }
}

PLUGINLIB_EXPORT_CLASS(avt_vimba_camera::StereoCameraNodelet, nodelet::Nodelet)
