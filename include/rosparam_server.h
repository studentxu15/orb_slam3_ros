#ifndef ORBSLAM_ROS_ROSPARAM_SERVER_H
#define ORBSLAM_ROS_ROSPARAM_SERVER_H

#include "header.h"

class ParamServer
{
public:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    std::string camera_image_topic_;
    std::string camera_info_topic_;

    int mSensor;

    ParamServer()
        : nh(), private_nh("~")
    {
        private_nh.param<string>("camera_image_topic", camera_image_topic_, "/camera/color/image_raw");
        private_nh.param<string>("camera_info_topic", camera_info_topic_, "/camera/color/camera_info");

        private_nh.param<int>("mSensor", mSensor, 0);

    }

};



#endif  // ORBSLAM_ROS_ROSPARAM_SERVER_H