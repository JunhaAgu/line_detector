#ifndef _MONO_LINE_DETECTOR_ROS_H_
#define _MONO_LINE_DETECTOR_ROS_H_

#include <iostream>
#include <fstream>
#include <dirent.h>

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

struct param_RANSAC
{
    int iter;
    float thr;
    int mini_inlier;
};

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

// launch params
private:
    bool flag_cam_live_;
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

// for test
private:
    std::string image_dir_;
    std::string image_type_;
    std::vector<cv::Mat> img_vec_;
    std::vector<std::string> file_lists_;
    int n_test_data_;

    // bwlabel
    std::vector<int> object_area_row_;
    std::vector<int> object_area_col_;
    
    //points
    std::vector<int> points_x_;
    std::vector<int> points_y_;
    std::vector<float> line_a_;
    std::vector<float> line_b_;

    param_RANSAC param_RANSAC_;
    

// for test
private:
    void test();
    void readImage(std::string& image_dir, std::string& image_type);
    void sort_filelists(std::vector<std::string>& filists, std::string& image_type);
    void ransacLine(std::vector<int>& points_x, std::vector<int>& points_y, 
                                    /*output*/ bool mask_inlier[], std::vector<float>& line_a, std::vector<float>& line_b);

    void reset_vector();

private: 

    void callbackImage(const sensor_msgs::ImageConstPtr& msg);

private:
    void run();

};

#endif