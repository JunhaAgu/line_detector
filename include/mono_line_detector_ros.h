#ifndef _MONO_LINE_DETECTOR_ROS_H_
#define _MONO_LINE_DETECTOR_ROS_H_

#include <iostream>

#include "ros/ros.h"
#include "sensor_msgs/Image.h" // for 'image'\

#include "cv_bridge/cv_bridge.h"
#include "opencv2/core.hpp"

#include "opencv2/ximgproc/fast_line_detector.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include <opencv2/video/tracking.hpp> // calcOpticalFlow
#include <opencv2/calib3d.hpp> // findHomography

#include "timer.h"

class MonoLineDetectorROS
{
private:
    ros::NodeHandle nh_;

private:
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;

    cv::Mat img_color_undist_;
    cv::Mat map1_;
    cv::Mat map2_;

private:
    cv::Mat prev_img_;

private:
    std::vector<cv::Point2f> feature_init_;
    bool flag_feature_init_success_; 
    bool flag_track_go_;


// Subscribers
private:
    ros::Subscriber sub_image_;
    std::string topicname_image_;

public:
    MonoLineDetectorROS(const ros::NodeHandle& nh);

// CLASS of the YOUR OWN MODULE
private:
    // YourLibrary yl_;
    cv::Ptr<cv::ximgproc::FastLineDetector> fast_line_detector_;

private:
    void callbackImage(const sensor_msgs::ImageConstPtr& msg);

private:
    void run();

};

#endif