
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

class UserParam
{
    friend class MonoLineDetectorROS;

    public:
        LineDetectorParam line_detector_param_;
        RansacParam ransac_param_;

    public:
        UserParam();
        ~UserParam();
        void getUserSettingParameters();
};

#endif