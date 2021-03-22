#include <avt_vimba_camera/avt_vimba_camera.h>
#include <avt_vimba_camera/AvtVimbaCameraConfig.h>
#include <avt_vimba_camera/avt_vimba_api.h>
#include<avt_vimba_camera/mono_camera.h>
#include <cav_driver_utils/driver_wrapper/driver_wrapper.h>

#include<avt_vimba_camera/status_camera.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <cav_msgs/SystemAlert.h>
#include <cav_msgs/DriverStatus.h>
#include <string>

namespace avt_vimba_camera{
    class AVTVimbaCameraWrapper : public cav::DriverWrapper
    {

       public:
        AVTVimbaCameraWrapper(int argc, char **argv, const std::string &name = "avt_vimba_camera_wrapper");
        virtual ~AVTVimbaCameraWrapper();

        cav_msgs::DriverStatus getCameraStatus();

        private:
            //ros::Subscriber camera_status_sub;
            ros::Publisher camera_status_pub;

            // cav::DriverWrapper members
            virtual void initialize();
            virtual void pre_spin();
            virtual void post_spin();
            virtual void shutdown();
            void publish_status(cav_msgs::DriverStatus stat);
            StatusCamera status;
            


    };



}


