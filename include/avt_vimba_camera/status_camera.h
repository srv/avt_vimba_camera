/*
 * Copyright (C) 2019-2021 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <ros/ros.h>
#include <avt_vimba_camera/mono_camera.h>
#include <cav_msgs/SystemAlert.h>
#include <cav_msgs/DriverStatus.h>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>

namespace avt_vimba_camera {
class StatusCamera
{
    ros::Publisher status_pub_ ;
    boost::thread* cam_thread_;

public:
    uint8_t status_cam;
    cav_msgs::DriverStatus status_;
    //Destructor to interrupt the cam_thread
    ~StatusCamera();
    void alertCallback(const cav_msgs::SystemAlertConstPtr &msg);
    void publish_status();
    void publish_off_status();
    void pre_camera(ros::Publisher status_pub);
    void post_camera();
};
}

