#include <nodelet/nodelet.h>
#include "avt_vimba_camera/stereo_camera.h"

namespace avt_vimba_camera
{

    class StereoCameraNodelet : public nodelet::Nodelet
    {
        public:
            virtual void onInit();
            virtual ~StereoCameraNodelet();
        private:
            StereoCamera* camera_;
    };

}
