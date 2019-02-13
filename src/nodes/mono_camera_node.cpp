#include <ros/ros.h>
#include <avt_vimba_camera/mono_camera.h>
#include <avt_vimba_camera/status_camera.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mono_camera_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::Rate loop_rate(19); //Camera max frame rate is 19Hz
  avt_vimba_camera::StatusCamera hc(nh,nhp);
  hc.pre_camera();
  avt_vimba_camera::MonoCamera mc(nh,nhp);
  hc.post_camera();

while(ros::ok())
{
mc.updateCameraStatus();
ros::spinOnce();
loop_rate.sleep();
hc.status_cam=mc.cam_status;
hc.publish_status();
}
  return 0;
}


