/// Copyright (c) 2014,
/// Systems, Robotics and Vision Group
/// University of the Balearic Islands
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///     * Redistributions of source code must retain the above copyright
///       notice, this list of conditions and the following disclaimer.
///     * Redistributions in binary form must reproduce the above copyright
///       notice, this list of conditions and the following disclaimer in the
///       documentation and/or other materials provided with the distribution.
///     * All advertising materials mentioning features or use of this software
///       must display the following acknowledgement:
///       This product includes software developed by
///       Systems, Robotics and Vision Group, Univ. of the Balearic Islands
///     * Neither the name of Systems, Robotics and Vision Group, University of
///       the Balearic Islands nor the names of its contributors may be used
///       to endorse or promote products derived from this software without
///       specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
/// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
/// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef SYNC_H
#define SYNC_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <std_msgs/String.h>
#include <boost/lexical_cast.hpp>

using namespace std;

namespace avt_vimba_camera {
class Sync {

  public:
    Sync(ros::NodeHandle nh, ros::NodeHandle nhp);
    void run();

  protected:

    void msgsCallback(const sensor_msgs::ImageConstPtr& l_img_msg,
                      const sensor_msgs::ImageConstPtr& r_img_msg,
                      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                      const sensor_msgs::CameraInfoConstPtr& r_info_msg);

    void syncCallback(const ros::TimerEvent&);

  private:

    // Node handles
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    bool init_; //!> True when node is initialized.
    double last_ros_sync_; //!> Last ros time sync
    double timer_period_; //!> Timer period
    double max_unsync_time_; //!> Maximum time without sync
    ros::Timer sync_timer_; //!> Timer to check the image sync
    string camera_; //!> Camera name

    // Topic sync
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                            sensor_msgs::Image,
                                                            sensor_msgs::CameraInfo,
                                                            sensor_msgs::CameraInfo> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> SyncType;

    // Image transport
    image_transport::ImageTransport it_;

    ros::Publisher pub_info_; //!> Publish reset info
};
}
#endif
