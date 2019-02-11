#include <ros/ros.h>
#include <avt_vimba_camera/mono_camera.h>
#include <cav_msgs/SystemAlert.h>
#include <cav_msgs/DriverStatus.h>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>

class Status_camera
{
    cav_msgs::DriverStatus status_;
    ros::Publisher status_pub_ ;
    ros::Subscriber alert_sub_;
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    boost::thread* my_thread_;
public:
    int status1;
    Status_camera(ros::NodeHandle,ros::NodeHandle);
    ~Status_camera();
    void alertCallback(const cav_msgs::SystemAlertConstPtr &msg);
    void publish_status();
    void pre_camera();
    void post_camera();
};

