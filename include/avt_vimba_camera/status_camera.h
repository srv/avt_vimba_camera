#include <ros/ros.h>
#include <avt_vimba_camera/mono_camera.h>
#include <cav_msgs/SystemAlert.h>
#include <cav_msgs/DriverStatus.h>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>

class StatusCamera
{
    cav_msgs::DriverStatus status_;
    ros::Publisher status_pub_ ;
    ros::Subscriber alert_sub_;
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    boost::thread* cam_thread_;
public:
    short int status_cam;
    //Constructor
    StatusCamera(ros::NodeHandle,ros::NodeHandle);
    //Destructor to interrupt the cam_thread
    ~StatusCamera();
    void alertCallback(const cav_msgs::SystemAlertConstPtr &msg);
    void publish_status();
    void pre_camera();
    void post_camera();
};

