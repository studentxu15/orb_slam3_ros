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
    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;
    else if(mSensor==IMU_MONOCULAR)
        cout << "Monocular-Inertial" << endl;
    else if(mSensor==IMU_STEREO)
        cout << "Stereo-Inertial" << endl;
    else if(mSensor==IMU_RGBD)
        cout << "RGB-D-Inertial" << endl;
    return true;
}

void ORBMapping::SubAndPubToROS()
{
    sub_image_ = nh.subscribe<sensor_msgs::Image>(camera_image_topic_, 100, &ORBMapping::image_callback, this);
}

void ORBMapping::image_callback(const sensor_msgs::ImageConstPtr &imgMsg)
{
    image_queue_.push_back(imgMsg);
    if (image_queue_.size() >= 200) {
        image_queue_.pop_front();
    }
}

void ORBMapping::Run()
{
    if (!SyncPackages())
    {
        return;
    }

    
}

bool ORBMapping::SyncPackages()
{
    if (!initial_camera_info_ || image_queue_.empty())
    {
        return false; 
    }
    return true;
}





} // namespace orb_slam3_ros