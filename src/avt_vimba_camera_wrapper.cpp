#include <avt_vimba_camera/avt_vimba_camera_wrapper.h>


    AVTVimbaCameraWrapper::AVTVimbaCameraWrapper(int argc, char **argv, const std::string &name = "avt_vimba_camera_wrapper"){}


    void AVTVimbaCameraWrapper::initialize()
    {
        camera_status_pub = nh_->advertise<cav_msgs::DriverStatus>("avt_vimba_camera_wrapper", 1);


    }

    void AVTVimbaCameraWrapper::pre_spin()
    {
        //Assign publisher for camera's status
        status.pre_camera(camera_status_pub);


    }



    cav_msgs::DriverStatus AVTVimbaCameraWrapper::getCameraStatus()
    {
        return(status.status_);
    }

    void AVTVimbaCameraWrapper::shutdown(){}

    void AVTVimbaCameraWrapper::post_spin(){}




