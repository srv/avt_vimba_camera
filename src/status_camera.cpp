#include <avt_vimba_camera/status_camera.h>

Status_camera::Status_camera(ros::NodeHandle nh, ros::NodeHandle nhp)
{
    status_.name=ros::this_node::getName();
    status_.camera=true;
    nh_=nh;
    nhp_=nhp;
    //Subscriber object for system alert
    status_pub_ = nh_.advertise<cav_msgs::DriverStatus>("driver_discovery", 1);
}

Status_camera::~Status_camera() {
    my_thread_->interrupt();
}

void Status_camera::alertCallback(const cav_msgs::SystemAlertConstPtr &msg)
{
if( msg->type==cav_msgs::SystemAlert::FATAL || msg->type==cav_msgs::SystemAlert::SHUTDOWN)
{
ros::shutdown();
}
}

void Status_camera::publish_status()
{
    while(true){
        if (status1==cav_msgs::DriverStatus::OFF)
       {
        status_.status=cav_msgs::DriverStatus::OFF;
       }
       else if (status1==cav_msgs::DriverStatus::OPERATIONAL)
       {
       status_.status=cav_msgs::DriverStatus::OPERATIONAL;
       }
       else if (status1==cav_msgs::DriverStatus::FAULT)
       {
       status_.status=cav_msgs::DriverStatus::FAULT;
       }
       status_pub_.publish(status_);
       ros::Duration(0.1).sleep();
    }
}

void Status_camera::pre_camera()
{
    my_thread_ = new boost::thread(boost::bind(&Status_camera::publish_status,this));
    alert_sub_ = nh_.subscribe<cav_msgs::SystemAlert>("system_alert",10,&Status_camera::alertCallback, this);
}

void Status_camera::post_camera()
{
    my_thread_->interrupt();

}
