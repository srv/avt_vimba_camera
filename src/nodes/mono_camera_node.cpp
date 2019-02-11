#include <ros/ros.h>
#include <avt_vimba_camera/mono_camera.h>
#include <avt_vimba_camera/status_camera.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mono_camera_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::Rate loop_rate(10);
  Status_camera h(nh,nhp);
  h.pre_camera();
  avt_vimba_camera::MonoCamera mc(nh,nhp);
  h.post_camera();

while(ros::ok())
{

h.status1=mc.flag;
mc.time_compare();
ros::spinOnce();
loop_rate.sleep();
}
  return 0;
}

