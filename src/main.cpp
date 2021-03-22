#include <avt_vimba_camera/avt_vimba_camera_wrapper.h>

int main(int argc, char**argv)
{
   avt_vimba_camera::AVTVimbaCameraWrapper wrapper(argc,argv);
   return  wrapper.run();
}