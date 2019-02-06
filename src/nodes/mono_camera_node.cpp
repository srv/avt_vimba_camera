#include <ros/ros.h>
#include <avt_vimba_camera/mono_camera.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mono_camera_node");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::Rate loop_rate(19);
  avt_vimba_camera::MonoCamera mc(nh,nhp);
 
 // ros::spin();
while(ros::ok)
{
mc.publish_status();
ros::spinOnce();
loop_rate.sleep();
}
  return 0;
}
