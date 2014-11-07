#include <ros/ros.h>
#include <avt_vimba_camera/stereo_camera.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "stereo_camera_node");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  avt_vimba_camera::StereoCamera sc(nh,nhp);
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
  return 0;
}
