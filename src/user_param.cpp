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
    ransac_param_.mini_inlier- = 30;
};