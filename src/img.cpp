#include "img.hpp"

IMG::IMG(const std::unique_ptr<UserParam>& user_param)
{
    camera_intrinsic_param_.fx_ = user_param->camera_intrinsic_param_.fx_;
    camera_intrinsic_param_.fy_ = user_param->camera_intrinsic_param_.fy_;
    camera_intrinsic_param_.cx_ = user_param->camera_intrinsic_param_.cx_;
    camera_intrinsic_param_.cy_ = user_param->camera_intrinsic_param_.cy_;

    camera_intrinsic_param_.k1_ = user_param->camera_intrinsic_param_.k1_;
    camera_intrinsic_param_.k2_ = user_param->camera_intrinsic_param_.k2_;
    camera_intrinsic_param_.p1_ = user_param->camera_intrinsic_param_.p1_;
    camera_intrinsic_param_.p2_ = user_param->camera_intrinsic_param_.p2_;
    camera_intrinsic_param_.k3_ = user_param->camera_intrinsic_param_.k3_;

    fast_line_detector_param_.length_threshold_     = user_param->fast_line_detector_param_.length_threshold_;
    fast_line_detector_param_.distance_threshold_   = user_param->fast_line_detector_param_.distance_threshold_;
    fast_line_detector_param_.canny_th1_            = user_param->fast_line_detector_param_.canny_th1_;
    fast_line_detector_param_.canny_th2_            = user_param->fast_line_detector_param_.canny_th2_;
    fast_line_detector_param_.canny_aperture_size_  = user_param->fast_line_detector_param_.canny_aperture_size_;
    fast_line_detector_param_.do_merge_             = user_param->fast_line_detector_param_.do_merge_;

    cameraMatrix_= cv::Mat::eye(3, 3, CV_64FC1);
    distCoeffs_ = cv::Mat::zeros(1, 5, CV_64FC1);

    cameraMatrix_=(cv::Mat1d(3, 3) <<  camera_intrinsic_param_.fx_, 0.0, camera_intrinsic_param_.cx_,
                                        0.0, camera_intrinsic_param_.fy_, camera_intrinsic_param_.cy_, 
                                        0.0, 0.0, 1.0);
    distCoeffs_ = (cv::Mat1d(1, 5) << camera_intrinsic_param_.k1_, camera_intrinsic_param_.k2_, 
                                        camera_intrinsic_param_.p1_, camera_intrinsic_param_.p2_, 
                                        camera_intrinsic_param_.k3_);
    cv::initUndistortRectifyMap(cameraMatrix_, distCoeffs_, cv::Mat(), cameraMatrix_, cv::Size(752, 480), CV_32FC1, map1_, map2_);

    this->fast_line_detector_ = cv::ximgproc::createFastLineDetector(fast_line_detector_param_.length_threshold_,
            fast_line_detector_param_.distance_threshold_, fast_line_detector_param_.canny_th1_, fast_line_detector_param_.canny_th2_,
            fast_line_detector_param_.canny_aperture_size_, fast_line_detector_param_.do_merge_);
};

IMG::~IMG()
{

};

void IMG::receiveImageMsg(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    cv_ptr->image.copyTo(img_gray_);

    //for calibration
    img_gray_.copyTo(img_distort_);

    n_col_ = img_gray_.cols;
    n_row_ = img_gray_.rows;
};

void IMG::undistImage()
{
    cv::remap(img_gray_, img_undist_, map1_, map2_, cv::INTER_LINEAR);

    img_undist_.copyTo(img_gray_);

    //cvtColor 없으면 안되는걸 봐서 img_gray가 CV_GRAY가 아니다?
    // cv::cvtColor(img_color, img_gray, CV_BGR2GRAY);
    img_gray_.convertTo(img_gray_, CV_8UC1);
    img_gray_.convertTo(img_visual_, CV_8UC1);
    img_gray_.convertTo(img_gray_original_, CV_8UC1);

};

void IMG::detectLines()
{
    cv::Mat img_channels[3];
    
    std::vector<cv::Vec4f> lines;
    
    // line detection
    fast_line_detector_->detect(img_gray_, lines);

    // draw lines on image
    cv::Mat img_zero = cv::Mat::zeros(n_row_, n_col_, CV_8UC1);
    fast_line_detector_->drawSegments(img_zero, lines);

    // BGR중 3번째
    cv::split(img_zero, img_channels);
    img_channels[2].copyTo(img_gray_);
};

void IMG::dilateImage()
{
    // cv::threshold(img_gray, img_threshold, 180, 255, cv::THRESH_BINARY);
    cv::dilate(img_gray_, img_dilate_, cv::Mat::ones(cv::Size(10, 10), CV_8UC1));
};

void IMG::erodeImage()
{
    // cv::threshold(img_dilate, img_threshold, 180, 255, cv::THRESH_BINARY);
    cv::erode(img_dilate_, img_erode_, cv::Mat::ones(cv::Size(10, 10), CV_8UC1));
};

void IMG::connectBinImage()
{
    // bwlabel
    cv::Mat object_label = cv::Mat::zeros(n_row_, n_col_, CV_32SC1);
    int *ptr_object_label = object_label.ptr<int>(0);
    cv::Mat stats, centroids;
    std::vector<int> sum_object;
    img_clone_ = cv::Mat::zeros(n_row_, n_col_, CV_8UC1);
    uchar *ptr_img_clone = img_clone_.ptr<uchar>(0);

    // bwlabel
    int n_label = cv::connectedComponentsWithStats(img_erode_, object_label, stats, centroids, 8);
    if (n_label == 0)
    {
        ROS_INFO_STREAM("There is no connectivity");
        return;
    }

    int obj_left;
    int obj_top;
    int obj_width;
    int obj_height;

    for (int object_idx = 0; object_idx < n_label; ++object_idx)
    {
        // object_idx=0 -> background
        if (object_idx == 0)
        {
            sum_object.push_back(0);
            continue;
        }
        int cnt_obj_pixel = 0;
        // object_area_row_.resize(0);
        // object_area_col_.resize(0);
        obj_left    = stats.at<int>(object_idx, cv::CC_STAT_LEFT);
        obj_top     = stats.at<int>(object_idx, cv::CC_STAT_TOP);
        obj_width   = stats.at<int>(object_idx, cv::CC_STAT_WIDTH);
        obj_height  = stats.at<int>(object_idx, cv::CC_STAT_HEIGHT);

        for (int i = obj_top; i < obj_top + obj_height; ++i)
        {
            int i_ncols = i * n_col_;
            for (int j = obj_left; j < obj_left + obj_width; ++j)
            {
                if (*(ptr_object_label + i_ncols + j) == object_idx)
                {
                    // object_area_row_.push_back(i);
                    // object_area_col_.push_back(j);
                    cnt_obj_pixel += 1;
                }
            }
        }
        sum_object.push_back(cnt_obj_pixel);
    }
    int max_obj_pixel_idx = max_element(sum_object.begin(), sum_object.end()) - sum_object.begin();

    obj_left    = stats.at<int>(max_obj_pixel_idx, cv::CC_STAT_LEFT);
    obj_top     = stats.at<int>(max_obj_pixel_idx, cv::CC_STAT_TOP);
    obj_width   = stats.at<int>(max_obj_pixel_idx, cv::CC_STAT_WIDTH);
    obj_height  = stats.at<int>(max_obj_pixel_idx, cv::CC_STAT_HEIGHT);
    for (int i = obj_top; i < obj_top + obj_height; ++i)
    {
        int i_ncols = i * n_col_;
        for (int j = obj_left; j < obj_left + obj_width; ++j)
        {
            if (*(ptr_object_label + i_ncols + j) == max_obj_pixel_idx)
            {
                *(ptr_img_clone + i_ncols + j) = 255;
            }
        }
    }

    ROS_INFO_STREAM("sum_obj: " << sum_object[0] << " " << sum_object[1] << " " << sum_object[2] << " " << sum_object[3]);
    ROS_INFO_STREAM("max_obj_pixel_idx: " << max_obj_pixel_idx);
};

void IMG::skeletonizeImage()
{
    // skel
    skel_ = cv::Mat::zeros(n_row_, n_col_, CV_8UC1); //(img_clone.size(), CV_8UC1, cv::Scalar(0));

    // cv::ximgproc::thinning(img_clone, skel);
    Thinning(img_clone_, n_row_, n_col_);
    img_clone_.copyTo(skel_);
};

void IMG::visualizeImage(std::string& window_name, cv::Mat& image_name)
{
    cv::imshow(window_name, image_name);
    cv::waitKey(5);
};

void IMG::Thinning(cv::Mat input, int row, int col)
{
    int x, y, p, q, xp, yp, xm, ym, cc, cd;
    int np1, sp1, hv;
    int cnt=0, chk=0, flag=0;
 
    unsigned char *m_BufImage;
    unsigned char *m_ResultImg;
    m_BufImage =    (unsigned char*)malloc(sizeof(unsigned char)*row*col);
    m_ResultImg =    (unsigned char*)malloc(sizeof(unsigned char)*row*col);
 
    // Result image에 Mat format의 input image Copy
    for( y = 0 ; y < row ; y ++ ){
        for( x = 0 ; x < col ; x++ ){
            *(m_ResultImg+(col*y)+x) = input.at<uchar>(y,x);
        }
    }
 
    do{
        // Image Buffer를 0으로 셋팅
        for( y = 1 ; y < row-1 ; y ++ ){
            for( x = 1 ; x < col-1 ; x++ ){
                *(m_BufImage+(col*y)+x) = 0;
            }
        }
 
        // 천이 추출
        if(chk == 0) flag = 0;
        chk = cnt % 2;
        cnt ++;
 
        for( y = 1 ; y < row-1 ; y ++ ){
            ym = y - 1;
            yp = y + 1;
            for( x = 1 ; x < col-1 ; x ++ ){
                if(*(m_ResultImg+(col*y)+x) == 0) continue;
 
                np1 = 0;
                for(q = y-1 ; q <= y+1; q ++ ){
                    for(p = x-1 ; p <= x+1; p ++ ){
                        if(*(m_ResultImg+(col*q)+p) != 0) np1++;
                    }
                }
 
                if(np1 < 3 || np1 > 7){
                    *(m_BufImage+(col*y)+x) = 255;
                    continue;                    
                }
 
                xm = x - 1;
                xp = x + 1;
                sp1 = 0;
 
                if(*(m_ResultImg+(col*ym)+x) == 0 && *(m_ResultImg+(col*ym)+xp) != 0) sp1++;
                if(*(m_ResultImg+(col*ym)+xp) == 0 && *(m_ResultImg+(col*y)+xp) != 0) sp1++;
                if(*(m_ResultImg+(col*y)+xp) == 0 && *(m_ResultImg+(col*yp)+xp) != 0) sp1++;
                if(*(m_ResultImg+(col*yp)+xp) == 0 && *(m_ResultImg+(col*yp)+x) != 0) sp1++;
                if(*(m_ResultImg+(col*yp)+x) == 0 && *(m_ResultImg+(col*yp)+xm) != 0) sp1++;
                if(*(m_ResultImg+(col*yp)+xm) == 0 && *(m_ResultImg+(col*y)+xm) != 0) sp1++;
                if(*(m_ResultImg+(col*y)+xm) == 0 && *(m_ResultImg+(col*ym)+xm) != 0) sp1++;
                if(*(m_ResultImg+(col*ym)+xm) == 0 && *(m_ResultImg+(col*ym)+x) != 0) sp1++;
 
                if(sp1 != 1){
                    *(m_BufImage+(col*y)+x) = 255;
                    continue;
                }
 
                if(chk == 0){
                    cc = *(m_ResultImg+(col*ym)+x) * *(m_ResultImg+(col*y)+xp);
                    cc = cc * *(m_ResultImg+(col*yp)+x);
 
                    cd = *(m_ResultImg+(col*y)+xp) * *(m_ResultImg+(col*yp)+x);
                    cd = cd * *(m_ResultImg+(col*y)+xm);
                }
                else{
                    cc = *(m_ResultImg+(col*ym)+x) * *(m_ResultImg+(col*y)+xp);
                    cc = cc * *(m_ResultImg+(col*y)+xm);
 
                    cd = *(m_ResultImg+(col*ym)+x) * *(m_ResultImg+(col*yp)+x);
                    cd = cd * *(m_ResultImg+(col*y)+xm);                
                }
 
                if(cc != 0 || cd != 0){
                    *(m_BufImage+(col*y)+x) = 255;
                    continue;
                }
                flag = 1;
            }
        }
 
        for( y = 1 ; y < row-1 ; y ++ ){
            for( x = 1 ; x < col-1 ; x ++ ){
                *(m_ResultImg+(col*y)+x) = *(m_BufImage+(col*y)+x);
            }
        }
    }while(!(chk == 1 && flag == 0));
 
    // 4연결점 처리
    for( y = 1 ; y < row-1 ; y ++ ){
        yp = y + 1;
        ym = y - 1;
        for( x = 1 ; x < col-1 ; x ++ ){
            if(*(m_ResultImg+(col*y)+x) == 0) continue;
 
            xm = x - 1;
            xp = x + 1;
            sp1 = 0;
            if(*(m_ResultImg+(col*ym)+x) == 0 && *(m_ResultImg+(col*ym)+xp) != 0) sp1++;
            if(*(m_ResultImg+(col*ym)+xp) == 0 && *(m_ResultImg+(col*y)+xp) != 0) sp1++;
            if(*(m_ResultImg+(col*y)+xp) == 0 && *(m_ResultImg+(col*yp)+xp) != 0) sp1++;
            if(*(m_ResultImg+(col*yp)+xp) == 0 && *(m_ResultImg+(col*yp)+x) != 0) sp1++;
            if(*(m_ResultImg+(col*yp)+x) == 0 && *(m_ResultImg+(col*yp)+xm) != 0) sp1++;
            if(*(m_ResultImg+(col*yp)+xm) == 0 && *(m_ResultImg+(col*y)+xm) != 0) sp1++;
            if(*(m_ResultImg+(col*y)+xm) == 0 && *(m_ResultImg+(col*ym)+xm) != 0) sp1++;
            if(*(m_ResultImg+(col*ym)+xm) == 0 && *(m_ResultImg+(col*ym)+x) != 0) sp1++;
 
            hv = 0;
            if(sp1 == 2){
                if        ((*(m_ResultImg+(col*ym)+x) & *(m_ResultImg+(col*y)+xp)) != 0) hv++;
                else if    ((*(m_ResultImg+(col*y)+xp) & *(m_ResultImg+(col*yp)+x)) != 0) hv++;
                else if    ((*(m_ResultImg+(col*yp)+x) & *(m_ResultImg+(col*y)+xm)) != 0) hv++;
                else if    ((*(m_ResultImg+(col*y)+xm) & *(m_ResultImg+(col*ym)+x)) != 0) hv++;
 
                if(hv == 1) *(m_ResultImg+(col*y)+x) = 0;
            }
            else if(sp1 == 3){
                if        ((*(m_ResultImg+(col*ym)+x) & *(m_ResultImg+(col*y)+xm) & *(m_ResultImg+(col*y)+xp)) != 0) hv++;
                else if    ((*(m_ResultImg+(col*yp)+x) & *(m_ResultImg+(col*y)+xm) & *(m_ResultImg+(col*y)+xp)) != 0) hv++;
                else if ((*(m_ResultImg+(col*ym)+x) & *(m_ResultImg+(col*yp)+x) & *(m_ResultImg+(col*y)+xm)) != 0) hv++;
                else if ((*(m_ResultImg+(col*ym)+x) & *(m_ResultImg+(col*yp)+x) & *(m_ResultImg+(col*y)+xp)) != 0) hv++;
 
                if(hv == 1) *(m_ResultImg+(col*y)+x) = 0;
            }
        }
    }
 
    // 들어온 배열에 재복사
    for( y = 0 ; y < row ; y ++ ){
        for( x = 0 ; x < col ; x++ ){
            input.at<uchar>(y,x) = *(m_ResultImg+(col*y)+x);
        }
    }
 
    free(m_BufImage);
    free(m_ResultImg);
};