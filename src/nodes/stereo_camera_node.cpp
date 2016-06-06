#include <ros/ros.h>
#include <boost/thread.hpp>
#include <avt_vimba_camera/stereo_camera.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "stereo_camera_node");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  avt_vimba_camera::StereoCamera sc(nh,nhp);
  boost::thread stereoThread(&avt_vimba_camera::StereoCamera::run, &sc);

  // ROS spin
  ros::Rate r(10);
  while (ros::ok())
    r.sleep();
  ros::shutdown();

  return 0;
}
