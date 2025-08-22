/*
    This file is part of ORB-SLAM3-ROS
    Created by xubobo on 2025/08/22.
*/

#include "header.h"
#include "orb_mapping.h"
#include "rosparam_server.h"

volatile sig_atomic_t FLAG_EXIT = false;

void SigHandle(int sig) {
    FLAG_EXIT = true;
    ROS_WARN("catch sig %d", sig);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb_slam3_ros");
    ros::NodeHandle nh;

    auto orb_mapping = std::make_shared<orb_slam3_ros::ORBMapping>();
    orb_mapping->InitROS();

    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    while (ros::ok())
    {
        if (FLAG_EXIT) 
        {
            break;
        }
        ros::spinOnce();
        orb_mapping->Run();
        rate.sleep();
    }

    LOG(INFO) << "finishing mapping";
    orb_mapping->Finish();

    return 0;
}