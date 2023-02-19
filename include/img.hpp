
#ifndef _IMG_HPP_
#define _IMG_HPP_

#include "ros/ros.h"
#include "sensor_msgs/Image.h"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/core.hpp"

#include "opencv2/ximgproc/fast_line_detector.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include "user_param.hpp"

class IMG
{
    friend class MonoLineDetectorROS;

    public:
        cv::Mat img_gray_;
        cv::Mat img_distort_;

        cv::Mat cameraMatrix_;
        cv::Mat distCoeffs_;

        cv::Mat map1_;
        cv::Mat map2_;

        cv::Mat img_undist_;
        cv::Mat img_visual_;
        cv::Mat img_gray_original_;
        
        cv::Mat img_dilate_;
        cv::Mat img_erode_;

        cv::Mat img_clone_;

        cv::Mat skel_;

        int n_col_;
        int n_row_;

        CameraIntrinsicParam camera_intrinsic_param_;
        FastLineDetectorParam fast_line_detector_param_;
        cv::Ptr<cv::ximgproc::FastLineDetector> fast_line_detector_;
        
    public:
        IMG(const std::unique_ptr<UserParam>& user_param);
        ~IMG();

        public:
        void receiveImageMsg(const sensor_msgs::ImageConstPtr& msg);
        void undistImage();

        void detectLines();
        void dilateImage();
        void erodeImage();

        void connectBinImage();
        void skeletonizeImage();
        void Thinning(cv::Mat input, int row, int col);

        void visualizeImage(std::string& window_name, cv::Mat& image_name);
};

#endif