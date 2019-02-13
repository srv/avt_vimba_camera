/*
 * Copyright (C) 2019 LEIDOS.
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

#include <avt_vimba_camera/status_camera.h>

namespace avt_vimba_camera {
StatusCamera::StatusCamera(ros::NodeHandle nh, ros::NodeHandle nhp)
{
    status_.name=ros::this_node::getName();
    status_.camera=true;
    nh_=nh;
    nhp_=nhp;
    //Publisher object for driver_discovery
    status_pub_ = nh_.advertise<cav_msgs::DriverStatus>("driver_discovery", 1);
}
//Destructor to interrupt the cam_thread
StatusCamera::~StatusCamera()
 {
    cam_thread_->interrupt();
    cam_thread_->detach();
 }
//System alert function definition
void StatusCamera::alertCallback(const cav_msgs::SystemAlertConstPtr &msg)
{
  if( msg->type==cav_msgs::SystemAlert::FATAL || msg->type==cav_msgs::SystemAlert::SHUTDOWN)
    {
     ros::shutdown();
    }
}

void StatusCamera::publish_status()
{  //Various driver status conditions
    while(true){
        if (status_cam==cav_msgs::DriverStatus::OFF)
       {
        status_.status=cav_msgs::DriverStatus::OFF;
       }
       else if (status_cam==cav_msgs::DriverStatus::OPERATIONAL)
       {
       status_.status=cav_msgs::DriverStatus::OPERATIONAL;
       }
       else if (status_cam==cav_msgs::DriverStatus::FAULT)
       {
       status_.status=cav_msgs::DriverStatus::FAULT;
       }
       //Publisher for driver_status
       status_pub_.publish(status_);
       ros::Duration(0.1).sleep();
    }
}
//Start the thread and subscriber declaration
void StatusCamera::pre_camera()
{
    cam_thread_ = new boost::thread(boost::bind(&StatusCamera::publish_status,this));
    alert_sub_ = nh_.subscribe<cav_msgs::SystemAlert>("system_alert",10,&StatusCamera::alertCallback, this);
}
//Interrupt the cam_thread
void StatusCamera::post_camera()
{
    cam_thread_->interrupt();
    cam_thread_->detach();

}
}
