#include <ros/ros.h>
#include <boost/thread.hpp>
#include <avt_vimba_camera/sync.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sync_node");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  avt_vimba_camera::Sync sync(nh,nhp);

  boost::thread syncThread(&avt_vimba_camera::Sync::run, &sync);

  // ROS spin
  ros::Rate r(10);
  while (ros::ok())
    r.sleep();

  ros::shutdown();
  return 0;
}
