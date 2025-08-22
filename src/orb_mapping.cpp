/*
    This file is part of ORB-SLAM3-ROS
    Created by xubobo on 2025/08/22.
*/

#include "orb_mapping.h"

namespace orb_slam3_ros
{

bool ORBMapping::InitROS()
{
    SubAndPubToROS();
    return true;
}

void ORBMapping::SubAndPubToROS()
{
    sub_image_ = nh.subscribe<sensor_msgs::Image>(camera_image_topic_, 100, &ORBMapping::image_callback, this);
}

void ORBMapping::image_callback(const sensor_msgs::ImageConstPtr &imgMsg)
{
    image_queue_.push(imgMsg);

}
}