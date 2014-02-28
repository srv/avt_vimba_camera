#include <ros/ros.h>
#include <avt_vimba_camera/vimba_ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "avt_vimba_camera_node");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  avt_vimba_camera::VimbaROS vimba_ros(nh,nhp);

  ros::spin();
  return 0;
}