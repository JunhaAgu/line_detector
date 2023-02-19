#include "user_param.hpp"

UserParam::UserParam()
{

};

UserParam::~UserParam()
{

};

void UserParam::getUserSettingParameters()
{
    line_detector_param_.canny_thr_h_ = 40;
    line_detector_param_.canny_thr_l_ = 20;
    line_detector_param_.line_length_ = 20;

    ransac_param_.iter_ = 30;
    ransac_param_.thr_ = 3;
    ransac_param_.mini_inlier_ = 30;

    camera_intrinsic_param_.fx_ = 5.145008560635781e+02;
    camera_intrinsic_param_.fy_ = 5.145495171417294e+02;
    camera_intrinsic_param_.cx_ = 3.704036341964502e+02;
    camera_intrinsic_param_.cy_ = 2.247394340321797e+02;

    camera_intrinsic_param_.k1_ = -0.356639678263282;
    camera_intrinsic_param_.k2_ = 0.120685709311234;
    camera_intrinsic_param_.p1_ = 0.0;
    camera_intrinsic_param_.p2_ = 0.0;
    camera_intrinsic_param_.k3_ = 0.0;

    fast_line_detector_param_.length_threshold_     = 20;
    fast_line_detector_param_.distance_threshold_   = 1.41421356f;
    fast_line_detector_param_.canny_th1_            = 20.0;
    fast_line_detector_param_.canny_th2_            = 40.0;
    fast_line_detector_param_.canny_aperture_size_  = 3; // sobel filter size
    fast_line_detector_param_.do_merge_             = true;

};