#ifndef ORBSLAM_ROS_ORB_MAPPING_H
#define ORBSLAM_ROS_ORB_MAPPING_H

#include "header.h"
#include "rosparam_server.h"

namespace orb_slam3_ros
{

class ORBMapping : public ParamServer
{
public:
    ORBMapping();
    ~ORBMapping()
    {
        LOG(INFO) << "orb mapping deconstruct";
    }

    // init with ros
    bool InitROS();

    void Run();

    void image_callback(const sensor_msgs::ImageConstPtr &imgMsg);

public:
    enum eSensor
    {
        MONOCULAR = 0,
        STEREO = 1,
        RGBD = 2,
        IMU_MONOCULAR = 3,
        IMU_STEREO = 4,
        IMU_RGBD = 5,
    };

private:
    void SubAndPubToROS();

    std::deque<sensor_msgs::ImageConstPtr> image_queue_;
    bool initial_camera_info_ = false;

private:
    ros::Subscriber sub_image_;
    ros::Subscriber sub_camera_info_;

};

}


#endif  // ORBSLAM_ROS_ORB_MAPPING_H