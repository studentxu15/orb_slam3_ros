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

    void image_callback(const sensor_msgs::ImageConstPtr &imgMsg);

private:
    void SubAndPubToROS();

    std::queue<sensor_msgs::ImageConstPtr> image_queue_;

private:
    ros::Subscriber sub_image_;
    ros::Subscriber sub_camera_info_;

};

}


#endif  // ORBSLAM_ROS_ORB_MAPPING_H