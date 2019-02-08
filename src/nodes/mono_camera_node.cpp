#include <ros/ros.h>
#include <avt_vimba_camera/mono_camera.h>
#include <cav_msgs/SystemAlert.h>
#include <cav_msgs/DriverStatus.h>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>
cav_msgs::DriverStatus status_;
void alertCallback(const cav_msgs::SystemAlertConstPtr &msg);
void publish_status();
ros::Publisher status_pub_ ;
int flag;
int main(int argc, char** argv)
{

  ros::init(argc, argv, "mono_camera_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::Rate loop_rate(10);
  ros::Subscriber alert_sub_;
  //Publisher object for driver status
  status_.name=ros::this_node::getName();
  status_.camera=true;
  //Subscriber object for system alert
  status_pub_ = nh.advertise<cav_msgs::DriverStatus>("driver_discovery", 1);
  alert_sub_ = nh.subscribe("system_alert",10,alertCallback);

  boost::thread* my_thread;
  my_thread = new boost::thread(boost::bind(publish_status));
  avt_vimba_camera::MonoCamera mc(nh,nhp);
  my_thread->interrupt();
 
 // ros::spin();
while(ros::ok())
{
mc.time_compare();
ros::spinOnce();
loop_rate.sleep();
}
  return 0;
}

void alertCallback(const cav_msgs::SystemAlertConstPtr &msg)
{
if( msg->type==cav_msgs::SystemAlert::FATAL || msg->type==cav_msgs::SystemAlert::SHUTDOWN)
{
ros::shutdown();
}
}

void publish_status()
{
    while(true){
        if (flag==1)
       {
        status_.status=cav_msgs::DriverStatus::OFF;
       }
       else if (flag==2)
       {
       status_.status=cav_msgs::DriverStatus::OPERATIONAL;
       }
       else if (flag==3)
       {
       status_.status=cav_msgs::DriverStatus::FAULT;
       }
       status_pub_.publish(status_);
       ros::Duration(0.1).sleep();
    }
}
