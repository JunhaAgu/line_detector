#include "mono_line_detector_ros.h"

MonoLineDetectorROS::MonoLineDetectorROS(const ros::NodeHandle& nh)
: nh_(nh), topicname_image_("")
{
    // Check whether needed parameters exist or not.
    if(ros::param::has("~topicname_image"))
        ros::param::get("~topicname_image", topicname_image_);
    else
        throw std::runtime_error("There is no 'topicname_image'.");
    
    // Check live data or not
    ros::param::get("~flag_cam_live", flag_cam_live_);
    ros::param::get("~image_dir", image_dir_);
    ros::param::get("~image_type", image_type_);

    // Subscribe
    sub_image_ = nh_.subscribe<sensor_msgs::Image>(topicname_image_, 1, 
                    &MonoLineDetectorROS::callbackImage, this);

    if (flag_cam_live_==false)
    {
        readImage(image_dir_, image_type_);
        
        img_vec_.reserve(10000);
        // for(int i=0;)
    }

    //bwlabel
    object_area_row_.reserve(20000);
    object_area_col_.reserve(20000);

    //points
    points_x_.reserve(10000);
    points_y_.reserve(10000);
    line_a_.reserve(10);
    line_b_.reserve(10);

    //RANSAC
    param_RANSAC_.iter = 25;
    param_RANSAC_.thr = 2;
    param_RANSAC_.mini_inlier = 30;

    // Create Fast Line Detector Object
    // Param               Default value   Description
    // length_threshold    10            - Segments shorter than this will be discarded
    // distance_threshold  1.41421356    - A point placed from a hypothesis line
    //                                     segment farther than this will be
    //                                     regarded as an outlier
    // canny_th1           50            - First threshold for
    //                                     hysteresis procedure in Canny()
    // canny_th2           50            - Second threshold for
    //                                     hysteresis procedure in Canny()
    // canny_aperture_size 3            - Aperturesize for the sobel operator in Canny().
    //                                     If zero, Canny() is not applied and the input
    //                                     image is taken as an edge image.
    // do_merge            false         - If true, incremental merging of segments
    //                                     will be performed
    int   length_threshold   = 20;
    float distance_threshold = 1.41421356f;
    double canny_th1         = 40.0;
    double canny_th2         = 80.0;
    int canny_aperture_size  = 3; // sobel filter size
    bool do_merge            = true;

    this->fast_line_detector_ = cv::ximgproc::createFastLineDetector(length_threshold,
            distance_threshold, canny_th1, canny_th2, canny_aperture_size,
            do_merge);

    prev_img_ = cv::Mat::zeros(752, 480, CV_8UC1);

    flag_feature_init_success_ = false;
    flag_track_go_ = false;
    feature_init_.reserve(12);

    cameraMatrix_= cv::Mat::eye(3, 3, CV_64FC1);
    distCoeffs_ = cv::Mat::zeros(1, 5, CV_64FC1);
    cameraMatrix_=(cv::Mat1d(3, 3) <<  5.158077490877873e+02, 0.0, 3.645846749659401e+02, 0.0, 5.158399101987746e+02, 2.097769119587788e+02, 0.0, 0.0, 1.0);
    distCoeffs_ = (cv::Mat1d(1, 5) << -0.368520007660487, 0.142043301312520, 0.0, 0.0, 0.0);
    cv::initUndistortRectifyMap(cameraMatrix_, distCoeffs_, cv::Mat(), cameraMatrix_, cv::Size(752, 480), CV_32FC1, map1_, map2_);

    this->run();

    ROS_INFO_STREAM("MonocularLineDetectorROS is constructed.");
};

void MonoLineDetectorROS::run()
{   
    if (flag_cam_live_ == true)
    {
        ros::Rate rate(500);
        while (ros::ok())
        {
            // ROS_INFO_STREAM("RUN!");
            ros::spinOnce();
            rate.sleep();
        }
    }
    else // flag_cam_live_ == false
    {
        ros::Rate rate(20);
        while (ros::ok())
        {
            test();
            ros::spinOnce();
            rate.sleep();
        }
    }

};

bool computePairNum(std::string pair1, std::string pair2)
{
    // return pair1 < pair2;

    int num1 = std::stoi(pair1);
    int num2 = std::stoi(pair2);
    return num1 < num2;
}

void MonoLineDetectorROS::sort_filelists(std::vector<std::string>& filists, std::string& image_type)
{
    if (filists.empty())
    {
        return;
    }
    std::sort(filists.begin(), filists.end(), computePairNum);
}

void MonoLineDetectorROS::readImage(std::string& image_dir, std::string& image_type)
{
    struct dirent *ptr;
    DIR *dir;
    dir = opendir(image_dir.c_str());
    file_lists_.clear();

    while((ptr = readdir(dir)) != NULL)
    {
        std::string tmp_file = ptr->d_name;
        if (tmp_file[0] == '.') continue;
        if (image_type.size() <= 0)
        {
            file_lists_.push_back(ptr->d_name);
        }
        else
        {
            if (tmp_file.size() < image_type.size()) continue;
            std::string tmp_cut_type = tmp_file.substr(tmp_file.size() - image_type.size(), image_type.size());
            if (tmp_cut_type == image_type)
            {
                file_lists_.push_back(ptr->d_name);
            }
        }
    }

    n_test_data_ = file_lists_.size();

    ROS_INFO_STREAM("# of files in folder: " <<n_test_data_);
    

    sort_filelists(file_lists_, image_type);

    for (int i=0; i< n_test_data_; ++i)
    {
        std::string img_file = image_dir + file_lists_[i];
        // std::cout << img_file <<std::endl;
        cv::Mat img = cv::imread(img_file);
        img_vec_.push_back(img);
    }
};

void MonoLineDetectorROS::test()
{
    static int iter_test = 0;
    // cv::imshow("img input", img_vec_[iter_test]);
    // cv::waitKey(0);

    cv::Mat img_gray;
    cv::Mat img_channels[3];

    // dilate & erode
    cv::Mat img_threshold;
    cv::Mat img_dilate;
    cv::Mat img_erode;

    //cvtColor 없으면 안되는걸 봐서 img_gray가 CV_GRAY가 아니다?
    cv::cvtColor(img_vec_[iter_test], img_gray, CV_BGR2GRAY);
    img_gray.convertTo(img_gray, CV_8UC1);

    int n_row = img_gray.rows;
    int n_col = img_gray.cols;

    //bwlabel
    cv::Mat object_label = cv::Mat::zeros(n_row, n_col, CV_32SC1);
    int *ptr_object_label = object_label.ptr<int>(0);
    cv::Mat stats, centroids;
    std::vector<int> sum_object;
    cv::Mat img_clone = cv::Mat::zeros(n_row, n_col, CV_8UC1);
    uchar *ptr_img_clone = img_clone.ptr<uchar>(0);

    //skel
    // cv::Mat skel(img_gray.size(), CV_8UC1, cv::Scalar(0));
    
    std::vector<cv::Vec4f> lines;
    if (iter_test == 0)
    {
        fast_line_detector_->detect(img_gray, lines);
        fast_line_detector_->drawSegments(img_gray, lines);

        // std:: cout << img_gray.type() << std::endl;

        cv::split(img_gray, img_channels);
        img_channels[2].copyTo(img_gray);
        uchar* ptr_img_gray = img_gray.ptr<uchar>(0);  
        // cv::imshow("B", img_channels[0]);
        // cv::imshow("G", img_channels[1]);
        // cv::imshow("R", img_channels[2]);
        for (int i=0; i<n_row; ++i)
        {
            int i_ncols = i * n_col;
            for (int j=0; j<n_col; ++j)
            {
                if (*(ptr_img_gray + i_ncols + j) < 255)
                {
                    *(ptr_img_gray + i_ncols + j) = 0;
                }
            }
        }
        cv::threshold(img_gray, img_threshold, 180, 255, cv::THRESH_BINARY);
        cv::dilate(img_threshold, img_dilate, cv::Mat::ones(cv::Size(7, 7), CV_8UC1));

        cv::threshold(img_dilate, img_threshold, 180, 255, cv::THRESH_BINARY);
        cv::erode(img_threshold, img_erode, cv::Mat::ones(cv::Size(7,7),CV_8UC1));

        //bwlabel
        int n_label = cv::connectedComponentsWithStats(img_erode, object_label, stats, centroids, 8);
        if (n_label == 0)
        {
            ROS_INFO_STREAM("There is no connectivity");
            return;
        }
        
        for (int object_idx = 0; object_idx < n_label; ++object_idx) 
        {
            // object_idx=0 -> background
            if (object_idx == 0)
            {
                sum_object.push_back(0);
                continue;
            }
            int cnt_obj_pixel = 0;
            object_area_row_.resize(0);
            object_area_col_.resize(0);
            int obj_left    = stats.at<int>(object_idx, cv::CC_STAT_LEFT); 
            int obj_top     = stats.at<int>(object_idx, cv::CC_STAT_TOP);
            int obj_width   = stats.at<int>(object_idx, cv::CC_STAT_WIDTH);
            int obj_height  = stats.at<int>(object_idx, cv::CC_STAT_HEIGHT);

            for (int i = obj_top; i < obj_top + obj_height; ++i)
            {
                int i_ncols = i*n_col;
                for (int j=obj_left; j<obj_left + obj_width; ++j)
                {
                    if(*(ptr_object_label+i_ncols+j) == object_idx)
                    {
                        object_area_row_.push_back(i);
                        object_area_col_.push_back(j);
                        cnt_obj_pixel += 1;
                    }
                }
            }
            sum_object.push_back(cnt_obj_pixel);
        }
        int max_obj_pixel_idx = max_element(sum_object.begin(), sum_object.end()) - sum_object.begin();

        int obj_left    = stats.at<int>(max_obj_pixel_idx, cv::CC_STAT_LEFT); 
        int obj_top     = stats.at<int>(max_obj_pixel_idx, cv::CC_STAT_TOP);
        int obj_width   = stats.at<int>(max_obj_pixel_idx, cv::CC_STAT_WIDTH);
        int obj_height  = stats.at<int>(max_obj_pixel_idx, cv::CC_STAT_HEIGHT);
        for (int i = obj_top; i < obj_top + obj_height; ++i)
        {
            int i_ncols = i * n_col;
            for (int j = obj_left; j < obj_left + obj_width; ++j)
            {
                if (*(ptr_object_label + i_ncols + j) == max_obj_pixel_idx)
                {
                    *(ptr_img_clone + i_ncols + j) = 255;
                }
            }
        }

        std::cout << sum_object[0] << " " << sum_object[1] << " "  << sum_object[2] << " "  << sum_object[3] <<std::endl;
        std::cout << "max_obj_pixel_idx: " << max_obj_pixel_idx << std::endl;

        // skel
        cv::Mat skel(img_clone.size(), CV_8UC1, cv::Scalar(0));
        uchar* ptr_skel = skel.ptr<uchar>(0);
        cv::Mat temp;
        cv::Mat eroded;
        cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

        bool done;
        do
        {
            cv::erode(img_clone, eroded, element);
            cv::dilate(eroded, temp, element); // temp = open(img)
            cv::subtract(img_clone, temp, temp);
            cv::bitwise_or(skel, temp, skel);
            eroded.copyTo(img_clone);

            done = (cv::countNonZero(img_clone) == 0);
        } while (!done);

        for (int i=0; i<n_row; ++i)
        {
            int i_ncols = i * n_col;
            for (int j=0; j<n_col; ++j)
            {
                if (*(ptr_skel + i_ncols + j) == 255)
                {
                    // std::cout << static_cast<int>(*(ptr_skel + i_ncols + j)) << std::endl;
                    points_x_.push_back(j);
                    points_y_.push_back(i);
                }
            }
        }

        int n_points = points_x_.size();
        bool mask_inlier[n_points];
        
        // 4 line detection
        for (int i=0; i<4; ++i)
        {
            int param_RANSAC_iter = 25;
            int param_RANSAC_thr = 2;
            int param_RANSAC_mini_inlier = 30;

            ransacLine(points_x_, points_y_, /*output*/ mask_inlier, line_a_, line_b_);
        }

        cv::imshow("img input", skel);
        // cv::imshow("img input", img_gray);
        cv::waitKey(0);
    }
    iter_test += 1;

    reset_vector();
};

void MonoLineDetectorROS::reset_vector()
{
    points_x_.resize(0);
    points_y_.resize(0);
    line_a_.resize(0);
    line_b_.resize(0);
};

void MonoLineDetectorROS::ransacLine(std::vector<int>& points_x, std::vector<int>& points_y, 
                                    /*output*/ bool mask_inlier[], std::vector<float>& line_a, std::vector<float>& line_b)
{
    
};

void MonoLineDetectorROS::callbackImage(const sensor_msgs::ImageConstPtr& msg)
{
    static int cnt_callback = 0;
    // msg -> cv::Mat img
    ROS_INFO_STREAM("callback image");
    timer::tic();

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

    cv::Mat img_gray;
    cv::Mat img_gray_vis;
    cv::Mat img_color;
    cv_ptr->image.copyTo(img_color);
    // cv::cvtColor(img_color, img_gray, CV_BGR2GRAY);

    // cv::Mat map1;
    // cv::Mat map2;
    // cv::initUndistortRectifyMap(cameraMatrix_, distCoeffs_, cv::Mat(), cameraMatrix_, cv::Size(752, 480), CV_32FC1, map1_, map2_);
    
    // cv::Mat img_color_undist_;
    // img_color_undist = img_color.clone();
    cv::remap(img_color, img_color_undist_, map1_, map2_, cv::INTER_LINEAR);

    // cv_ptr->image.copyTo(img_gray);
    cv::cvtColor(img_color_undist_, img_gray, CV_BGR2GRAY);
    img_gray.convertTo(img_gray, CV_8UC1);
    img_gray.copyTo(img_gray_vis);

    // cv::namedWindow("img input");
    // cv::imshow("img input", img_color);

    // cv::namedWindow("img input undist");
    // cv::imshow("img input undist", img_color_undist);
    // cv::waitKey(0);

    // ROS_INFO_STREAM("Size : " << img_gray.cols << ", " << img_gray.rows);
    // // 752 X 480

    // if (flag_feature_init_success_ == false)
    // {
    //     // Detect lines on image.
    //     std::vector<cv::Vec4f> lines;
    //     fast_line_detector_->detect(img_gray, lines);

    //     // for(const cv::Vec4f& line : lines)
    //     // {
    //     //     cv::Point2f p0(line(0),line(1));
    //     //     cv::Point2f p1(line(2),line(3));
    //     // }

    //     // timer::toc(1); // 2~9ms varying

    //     ROS_INFO_STREAM("# of detected lines: " << lines.size());

    //     // Image showing
    //     for(const cv::Vec4f& line : lines)
    //     {
    //         // cv::Point2f p0(line(0),line(1));
    //         // cv::Point2f p1(line(2),line(3));
    //         // cv::line(img_gray, p0,p1, cv::Scalar(255,0,255),2);
    //         // cv::circle(img_gray_vis, p0, 4, cv::Scalar(255, 0, 255), 1, 8, 0);
    //         // cv::circle(img_gray_vis, p1, 4, cv::Scalar(255, 0, 255), 1, 8, 0);
    //     }
    //     std::vector<cv::Point2f> corners;
    //     cv::goodFeaturesToTrack(img_gray, corners, 50, 0.1, 20);
    //     ROS_INFO_STREAM("# of detected corners: " << corners.size());

    //     // for(const cv::Point2f& corner : corners)
    //     // {
    //     //     // cv::line(img_gray, p0,p1, cv::Scalar(255,0,255),2);
    //     //     cv::circle(img_gray, corner, 1, cv::Scalar(255, 0, 255), 1, 8, 0);
    //     //     // cv::circle(img_gray, p1, 1, cv::Scalar(255, 0, 255), 1, 8, 0);
    //     // }

    //     // for(int i=0; i<corners.size(); ++i)
    //     // {
    //     //     float corner_x = corners[i].x;
    //     //     float corner_y = corners[i].y;
    //     //     for(int j=0; j<lines.size(); ++j)
    //     //     {
    //     //         if (std::abs(lines[j](0) - corner_x) < 20)
    //     //         {
    //     //             if (std::abs(lines[j](1) - corner_y) < 20)
    //     //             {
    //     //                 cv::circle(img_gray, corners[i], 5, cv::Scalar(255, 0, 255), 1, 8, 0);
    //     //             }
    //     //         }

    //     //         if (std::abs(lines[j](2) - corner_x) < 20)
    //     //         {
    //     //             if (std::abs(lines[j](3) - corner_y) < 20)
    //     //             {
    //     //                 cv::circle(img_gray, corners[i], 5, cv::Scalar(255, 0, 255), 1, 8, 0);
    //     //             }
    //     //         }
    //     //     }
    //     // }

    //     std::vector<cv::Point2f> corners_init;
    //     std::vector<int> valid_corner_idx;
    //     int cnt_corner = 0;
    //     int cnt_desired_feature = 0;
    //     for (const cv::Point2f &corner : corners)
    //     {
    //         float p0 = corner.x;
    //         float p1 = corner.y;
    //         for (const cv::Vec4f &line : lines)
    //         {
    //             float l0 = line(0);
    //             float l1 = line(1);
    //             float l2 = line(2);
    //             float l3 = line(3);

    //             float left_ = (l2 - l0) * (p0 - l0) + (l3 - l1) * (p1 - l1);
    //             float l2p = std::sqrt((p0 - l0) * (p0 - l0) + (p1 - l1) * (p1 - l1));
    //             float l2l = std::sqrt((l2 - l0) * (l2 - l0) + (l3 - l1) * (l3 - l1));
    //             float right_ = 0.9 * (l2p + l2l);

    //             if (left_ > right_)
    //             {
    //                 // cv::circle(img_gray, corner, 10, cv::Scalar(255, 0, 255), 1, 8, 0);
    //                 valid_corner_idx.push_back(cnt_corner);
    //                 cnt_desired_feature += 1;
    //                 break;
    //             }
    //         }
    //         cnt_corner += 1;
    //     }
    //     ROS_INFO_STREAM("# of feature point: " << cnt_desired_feature);

    //     if (cnt_desired_feature == 12)
    //     {
    //         ROS_INFO_STREAM("SUCCESS >> 12 feature point");
    //         feature_init_.resize(0);
    //         for (int i = 0; i < valid_corner_idx.size(); ++i)
    //         {
    //             feature_init_.push_back(corners[valid_corner_idx[i]]);
    //             cv::circle(img_gray_vis, feature_init_[i], 10, cv::Scalar(255, 0, 255), 1, 8, 0);
    //         }
    //         flag_feature_init_success_ = true;
    //     }
    // }

    // if (flag_track_go_==true)
    // {   
    //     // std::vector<cv::Mat> prevPyr, nextPyr;
    //     // cv::Size winSize(31,31);
    //     // int maxLevel = 3;
    //     // cv::buildOpticalFlowPyramid(prev_img_, prevPyr, winSize, maxLevel, true);
    //     // cv::buildOpticalFlowPyramid(img_gray, nextPyr, winSize, maxLevel, true);

    //     std::vector<cv::Point2f> feature_tracked;
    //     std::vector<unsigned char> status;
    //     std::vector<float> err;
    //     ROS_INFO_STREAM("before tracking");
    //     cv::calcOpticalFlowPyrLK(prev_img_, img_gray, feature_init_, feature_tracked, status, err, cv::Size(21,21), 0,
    //      cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01));
    //     // for (int i=0; i<12; ++i)
    //     // {
    //     //     // std::cout << static_cast<int>(status[i]) << " " << err[i] << std::endl;
    //     //     if (static_cast<int>(status[i]) ==0)
    //     //     {
    //     //         flag_feature_init_success_ = false;
    //     //         flag_track_go_ = false;
    //     //         return;
    //     //     }
    //     // }

    //     // homography
    //     std::vector<unsigned char> h_mask;
    //     cv::Mat H = cv::findHomography(feature_init_, feature_tracked, h_mask);

    //     // std::cout << H << std::endl;
    //     for (int i=0; i<12; ++i)
    //     {
    //         std::cout <<"h_mask: " << static_cast<int>(h_mask[i]) << std::endl;
    //     }

    //     for (int i=0; i<12; ++i)
    //     {
    //         // std::cout << static_cast<int>(status[i]) << " " << err[i] << std::endl;
    //         if (static_cast<int>(status[i]) ==0)
    //         {
    //             flag_feature_init_success_ = false;
    //             flag_track_go_ = false;
    //             // return;
    //         }
    //     }


    //     //////////////////// homography

    //     ROS_INFO_STREAM(feature_tracked.size());
    //     for (int i = 0; i < feature_tracked.size(); ++i)
    //     {
    //         cv::circle(img_gray_vis, feature_tracked[i], 10, cv::Scalar(255, 0, 255), 1, 8, 0);
    //     }
    //     feature_init_.resize(0);
        
    //     for (int i = 0; i < feature_tracked.size(); ++i)
    //     {
    //         feature_init_.push_back(feature_tracked[i]);
    //     }
    // }
    // std::cout << flag_feature_init_success_<< std::endl;

    // /* for calibration (save image)
    std::string image_folder = "/home/junhakim/mono_calibration/";
    std::string image_name = std::to_string(cnt_callback);
    std::string save_path = image_folder + image_name + ".png";
    std::cout << save_path <<std::endl;
    cv::imwrite(save_path, img_gray);
    // */

    

    timer::toc(1); // 2~9ms varying

    cv::namedWindow("img with lines");
    cv::imshow("img with lines", img_gray_vis);
    cv::waitKey(0);

    cnt_callback += 1;
    // img_gray.copyTo(prev_img_);
    if (flag_feature_init_success_ == true)
    {
        img_gray.copyTo(prev_img_);
        flag_track_go_ = true;
    }
    
    // Done.
};