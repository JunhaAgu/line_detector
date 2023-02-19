
#ifndef _USER_PARAM_HPP_
#define _USER_PARAM_HPP_

struct LineDetectorParam
{
    int canny_thr_h_;
    int canny_thr_l_;
    int line_length_;
};

struct RansacParam
{
    int iter_;
    float thr_;
    int mini_inlier_;
};

struct CameraIntrinsicParam
{
    double fx_;
    double fy_;
    double cx_;
    double cy_;

    double k1_;
    double k2_;
    double p1_;
    double p2_;
    double k3_;
};

struct FastLineDetectorParam
{
    int   length_threshold_;
    float distance_threshold_;
    double canny_th1_;
    double canny_th2_;
    int canny_aperture_size_; // sobel filter size
    bool do_merge_;
};

class UserParam
{
    friend class MonoLineDetectorROS;

    public:
        LineDetectorParam line_detector_param_;
        RansacParam ransac_param_;
        CameraIntrinsicParam camera_intrinsic_param_;
        FastLineDetectorParam fast_line_detector_param_;

    public:
        UserParam();
        ~UserParam();
        void getUserSettingParameters();
};

#endif