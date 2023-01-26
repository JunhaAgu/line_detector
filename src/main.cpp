#include <ros/ros.h>

#include <iostream>
#include <string>

#include "mono_line_detector_ros.h"

// #include

int main(int argc, char **argv)
{
    ros::init(argc, argv, "line_detector_node");
    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("START: \"line_detector_node\".\n");

    // Init. monocular line detector
    MonoLineDetectorROS mono_line_detector(nh);

    ROS_INFO_STREAM("TERMINATE: \"line_detector_node\". ");
    return -1;
}