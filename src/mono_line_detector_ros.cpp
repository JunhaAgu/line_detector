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
    ros::param::get("~flag_cam_stream",flag_cam_stream_);
    ros::param::get("~image_dir", image_dir_);
    ros::param::get("~image_type", image_type_);
    ros::param::get("~image_hz", image_hz_);

    // Subscribe
    sub_image_ = nh_.subscribe<sensor_msgs::Image>(topicname_image_, 1, 
                    &MonoLineDetectorROS::callbackImage, this);

    // Pusblish
    pub_projected_points_ = nh_.advertise<geometry_msgs::PoseArray>("/line_detector_node/projected_points",1);

    if (flag_cam_live_==false)
    {
        readImage(image_dir_, image_type_);
        
        img_vec_.reserve(10000);
        // for(int i=0;)
    }

    flag_init_ = false;

    //bwlabel
    object_area_row_.reserve(20000);
    object_area_col_.reserve(20000);

    //points
    points_x_.reserve(10000);
    points_y_.reserve(10000);
    line_a_.reserve(10);
    line_b_.reserve(10);

    //RANSAC
    param_RANSAC_.iter = 30;
    param_RANSAC_.thr = 1;
    param_RANSAC_.mini_inlier = 30;
    points_x_tmp_.reserve(10000);
    points_y_tmp_.reserve(10000);
    points_x_tmp2_.reserve(10000);
    points_y_tmp2_.reserve(10000);
    std::mt19937 gen_(rd());
    inlier_result_x_.reserve(10000);
    inlier_result_y_.reserve(10000);
    // points_result_.reserve(10);
    prev_feat_.reserve(10);
    next_feat_.reserve(10);
    help_feat_.reserve(10);

    //permutation
    perm_.resize(4 * 3 * 2 * 1);
    for (int i=0; i<24; ++i)
    {
        perm_[i].reserve(4);
    }

    std::vector<int> v;
    v.push_back(0);
    v.push_back(1);
    v.push_back(2);
    v.push_back(3);
    permutation(v, 0, 4, 4); // P(4,4)
    for (int i = 0; i < 24; ++i)
    {
        std::cout << perm_[i][0] << " " << perm_[i][1] << " " << perm_[i][2] << " " << perm_[i][3] << std::endl;
    }
    // for (int i=0; i<24; ++i)
    // {
    //     for (in)
    // }


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
    double canny_th1         = 20.0;
    double canny_th2         = 40.0;
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
    // cameraMatrix_=(cv::Mat1d(3, 3) <<  5.158077490877873e+02, 0.0, 3.645846749659401e+02, 0.0, 5.158399101987746e+02, 2.097769119587788e+02, 0.0, 0.0, 1.0);
    // distCoeffs_ = (cv::Mat1d(1, 5) << -0.368520007660487, 0.142043301312520, 0.0, 0.0, 0.0);
    cameraMatrix_=(cv::Mat1d(3, 3) <<  5.145008560635781e+02, 0.0, 3.704036341964502e+02, 0.0, 5.145495171417294e+02, 2.247394340321797e+02, 0.0, 0.0, 1.0);
    distCoeffs_ = (cv::Mat1d(1, 5) << -0.356639678263282, 0.120685709311234, 0.0, 0.0, 0.0);
    cv::initUndistortRectifyMap(cameraMatrix_, distCoeffs_, cv::Mat(), cameraMatrix_, cv::Size(752, 480), CV_32FC1, map1_, map2_);

    this->run();

    ROS_INFO_STREAM("MonocularLineDetectorROS is constructed.");
};

void MonoLineDetectorROS::run()
{   
    if (flag_cam_live_ == true)
    {
        ros::Rate rate(30);
        while (ros::ok())
        {
            // ROS_INFO_STREAM("RUN!");
            ros::spinOnce();
            rate.sleep();
        }
    }
    else // flag_cam_live_ == false
    {
        ros::Rate rate(image_hz_);
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
    // timer::tic();
    timer::tic();
    static int iter_test = 0;
    // bool flag_initialization = false ;

    ROS_INFO_STREAM(">>>>>>>>>> " << "Iter " << iter_test <<" Start <<<<<<<<<");

    // input image
    cv::Mat img_gray;
    cv::Mat img_channels[3];
    cv::Mat img_visual;
    cv::Mat img_gray_original;
    
    // dilate & erode
    cv::Mat img_threshold;
    cv::Mat img_dilate;
    cv::Mat img_erode;

    //cvtColor 없으면 안되는걸 봐서 img_gray가 CV_GRAY가 아니다?
    cv::cvtColor(img_vec_[iter_test], img_gray, CV_BGR2GRAY);
    img_gray.convertTo(img_gray, CV_8UC1);
    img_gray.convertTo(img_visual, CV_8UC1);
    img_gray.convertTo(img_gray_original, CV_8UC1);

    int n_row = img_gray.rows;
    int n_col = img_gray.cols;

    //bwlabel
    cv::Mat object_label = cv::Mat::zeros(n_row, n_col, CV_32SC1);
    int *ptr_object_label = object_label.ptr<int>(0);
    cv::Mat stats, centroids;
    std::vector<int> sum_object;
    cv::Mat img_clone = cv::Mat::zeros(n_row, n_col, CV_8UC1);
    uchar *ptr_img_clone = img_clone.ptr<uchar>(0);

    bool flag_line_detect = false;
    
    std::vector<cv::Vec4f> lines;

    fast_line_detector_->detect(img_gray, lines);

    cv::Mat img_zero = cv::Mat::zeros(n_row, n_col, CV_8UC1);
    fast_line_detector_->drawSegments(img_zero, lines);

    // cv::imshow("img input", img_zero);
    // cv::waitKey(0);

    cv::split(img_zero, img_channels);
    img_channels[2].copyTo(img_gray);
    uchar *ptr_img_gray = img_gray.ptr<uchar>(0);
    // cv::imshow("B", img_channels[0]);
    // cv::imshow("G", img_channels[1]);
    // cv::imshow("R", img_channels[2]);

    // for (int i = 0; i < n_row; ++i)
    // {
    //     int i_ncols = i * n_col;
    //     for (int j = 0; j < n_col; ++j)
    //     {
    //         if (*(ptr_img_gray + i_ncols + j) < 255)
    //         {
    //             *(ptr_img_gray + i_ncols + j) = 0;
    //         }
    //     }
    // }

    // cv::threshold(img_gray, img_threshold, 180, 255, cv::THRESH_BINARY);
    cv::dilate(img_gray, img_dilate, cv::Mat::ones(cv::Size(15, 15), CV_8UC1));

    // cv::threshold(img_dilate, img_threshold, 180, 255, cv::THRESH_BINARY);
    cv::erode(img_dilate, img_erode, cv::Mat::ones(cv::Size(15, 15), CV_8UC1));

    // bwlabel
    int n_label = cv::connectedComponentsWithStats(img_erode, object_label, stats, centroids, 8);
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
            int i_ncols = i * n_col;
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
        int i_ncols = i * n_col;
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
    
    // skel
    cv::Mat skel = cv::Mat::zeros(n_row, n_col, CV_8UC1); //(img_clone.size(), CV_8UC1, cv::Scalar(0));
    // if (sum_object[max_obj_pixel_idx] > 5000)
    // {
    //     cv::erode(img_clone, img_clone, cv::Mat::ones(cv::Size(3, 3), CV_8UC1));
    // }

    // cv::ximgproc::thinning(img_clone, skel);
    Thinning(img_clone, n_row, n_col);
    img_clone.copyTo(skel);
    uchar *ptr_skel = skel.ptr<uchar>(0);

    double dt_toc = timer::toc(1); // milliseconds
    ROS_INFO_STREAM("total time :" << dt_toc << " [ms]");

    // if (dt_toc>30)
    // {
    //     cv::imshow("img input", skel);
    //     cv::waitKey(0);
    //     // exit(0);
    // }

    // cv::imshow("img input", skel);
    // cv::waitKey(0);

    for (int i = 0; i < n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j)
        {
            if (*(ptr_skel + i_ncols + j) != 0)
            {
                // std::cout << static_cast<int>(*(ptr_skel + i_ncols + j)) << std::endl;
                points_x_.push_back(j);
                points_y_.push_back(i);
            }
        }
    }
    // std::cout << points_x_.size() <<std::endl;
    // std::cout << points_y_.size() <<std::endl;
    // cv::imshow("img input", skel);
    // cv::waitKey(0);

    int n_points = points_x_.size();
    // bool mask_inlier[n_points];
    // std::vector<int> points_x_tmp_;
    // std::vector<int> points_y_tmp_;

    points_x_tmp_.resize(0);
    points_y_tmp_.resize(0);
    // std::copy(points_x_.begin(), points_x_.end(), points_x_tmp_.begin());
    // std::copy(points_y_.begin(), points_y_.end(), points_y_tmp_.begin());
    for (int i = 0; i < n_points; ++i)
    {
        points_x_tmp_.push_back(points_x_[i]);
        points_y_tmp_.push_back(points_y_[i]);
    }
    // for (int i=0; i<points_x_tmp_.size(); ++i)
    // {
    //     std::cout<< "points_x_tmp_: " << points_x_tmp_[i]<<std::endl;
    // }

    // std::vector<int> points_x_tmp2_;
    // std::vector<int> points_y_tmp2_;

    // 4 line detection
    for (int i = 0; i < 4; ++i)
    {
        points_x_tmp2_.resize(0);
        points_y_tmp2_.resize(0);
        inlier_result_x_.resize(0);
        inlier_result_y_.resize(0);

        int n_pts_tmp = points_x_tmp_.size();
        std::cout << "# of remaining pts: " << n_pts_tmp << std::endl;
        bool mask_inlier[n_pts_tmp];
        for (int q = 0; q < n_pts_tmp; ++q)
        {
            mask_inlier[q] = false;
        }
        int param_RANSAC_iter = 25;
        int param_RANSAC_thr = 2;
        int param_RANSAC_mini_inlier = 30;

        // std::cout << points_x_tmp_.size() << " " << points_y_tmp_.size() << std::endl;
        ransacLine(points_x_tmp_, points_y_tmp_, /*output*/ mask_inlier, line_a_, line_b_, inlier_result_x_, inlier_result_y_);
        // std::cout<< "line_a_: " << line_a_[0]<< "line_b_: " << line_b_[0]<<std::endl;

        ///////// visualization (from)
        float points_x_tmp_min = *std::min_element(inlier_result_x_.begin(), inlier_result_x_.end());
        float points_x_tmp_max = *std::max_element(inlier_result_x_.begin(), inlier_result_x_.end());
        float points_y_tmp_min = line_a_[i] * points_x_tmp_min + line_b_[i];
        float points_y_tmp_max = line_a_[i] * points_x_tmp_max + line_b_[i];
        // cv::Point2f p0(points_x_tmp_min, points_y_tmp_min);
        // cv::Point2f p1(points_x_tmp_max, points_y_tmp_max);
        // cv::line(img_visual, p0,p1, cv::Scalar(255,0,255),1);
        ///////// visualization (to)

        for (int q = 0; q < n_pts_tmp; ++q)
        {
            if (mask_inlier[q] == false)
            {
                points_x_tmp2_.push_back(points_x_tmp_[q]);
                points_y_tmp2_.push_back(points_y_tmp_[q]);
            }
        }
        points_x_tmp_.resize(0);
        points_y_tmp_.resize(0);
        for (int i = 0; i < points_x_tmp2_.size(); ++i)
        {
            points_x_tmp_.push_back(points_x_tmp2_[i]);
            points_y_tmp_.push_back(points_y_tmp2_[i]);
        }

        flag_line_detect = true;

        if (points_x_tmp_.size() < 30 && i < 3)
        {
            flag_line_detect = false;
            break;
        }
    }

    if (flag_line_detect == true)
    {
        for (int i = 0; i < line_a_.size(); ++i)
        {
            std::cout << "line_a_[" << i << "]: " << line_a_[i] << "  "
                      << "line_b_[" << i << "]: " << line_b_[i] << std::endl;
        }

        std::vector<float> line_a_abs;
        for (int i = 0; i < line_a_.size(); ++i)
        {
            line_a_abs.push_back(SQUARE(line_a_[i]));
        }

        // intersection points
        int min_a_abs_idx = std::min_element(line_a_abs.begin(), line_a_abs.end()) - line_a_abs.begin();
        
        ROS_INFO_STREAM("min_a_abs_idx: " << min_a_abs_idx);

        std::vector<float> diff_b;
        float b_min_a_idx = line_b_[min_a_abs_idx];

        for (int i = 0; i < line_b_.size(); ++i)
        {
            if (i == min_a_abs_idx)
            {
                diff_b.push_back(1e7);
            }
            else
            {
                diff_b.push_back(std::abs(line_b_[i] - b_min_a_idx));
            }
            std::cout << diff_b[i] << std::endl;
        }
        int min_diff_b_idx = std::min_element(diff_b.begin(), diff_b.end()) - diff_b.begin();
        std::cout << min_diff_b_idx << std::endl;

        std::vector<float> dir1_line_a;
        std::vector<float> dir1_line_b;

        std::vector<float> dir2_line_a;
        std::vector<float> dir2_line_b;

        for (int i = 0; i < line_a_.size(); ++i)
        {
            if (i == min_a_abs_idx)
            {
                dir1_line_a.push_back(line_a_[i]);
                dir1_line_b.push_back(line_b_[i]);
            }
            else if (i == min_diff_b_idx)
            {
                dir1_line_a.push_back(line_a_[i]);
                dir1_line_b.push_back(line_b_[i]);
            }
            else
            {
                dir2_line_a.push_back(line_a_[i]);
                dir2_line_b.push_back(line_b_[i]);
            }
        }
        
        for (int i = 0; i < dir1_line_a.size(); ++i)
        {
            float dir1_a = dir1_line_a[i];
            float dir1_b = dir1_line_b[i];

            for (int j = 0; j < dir2_line_a.size(); ++j)
            {
                float dir2_a = dir2_line_a[j];
                float dir2_b = dir2_line_b[j];
                float px_tmp = 0.0;
                float py_tmp = 0.0;
                calcLineIntersection(dir1_a, dir1_b, dir2_a, dir2_b, px_tmp, py_tmp);
                cv::Point2f points_tmp(px_tmp, py_tmp);
                next_feat_.push_back(points_tmp);
                // px_.push_back(px_tmp);
                // py_.push_back(py_tmp);
                std::cout << "dir1_a: " << dir1_a << ", "
                          << "dir1_b: " << dir1_b << std::endl;
                std::cout << "dir2_a: " << dir2_a << ", "
                          << "dir2_b: " << dir2_b << std::endl;
                std::cout << "points_tmp: "
                          << "(" << points_tmp.x << ", " << points_tmp.y << ")" << std::endl;
            }
        }
        // for (int i = 0; i < next_feat_.size(); ++i)
        // {
        //     cv::circle(img_visual, next_feat_[i], 10, cv::Scalar(255, 0, 255), 1, 8, 0);
        // }

        if (flag_init_ == false)
        {
            flag_init_ = true;

            std::cout << "next_feat_[0]" << " " << next_feat_[0] << std::endl;
            std::cout << "next_feat_[1]" << " " << next_feat_[1] << std::endl;
            std::cout << "next_feat_[2]" << " " << next_feat_[2] << std::endl;
            std::cout << "next_feat_[3]" << " " << next_feat_[3] << std::endl;

            // 왼쪽 위 -> 왼쪽 아래 -> 오른쪽 아래 -> 오른쪽 위 : 순서대로 id0 id1 id2 id3
            std::vector<float> pts_x;
            std::vector<float> pts_y;
            for (int i=0; i<4; ++i)
            {
                pts_x.push_back(next_feat_[i].x);
                pts_y.push_back(next_feat_[i].y);
            }
            int pts_x_max_idx = std::max_element(pts_x.begin(),pts_x.end()) - pts_x.begin();
            pts_x.erase(pts_x.begin()+pts_x_max_idx);
            pts_y.erase(pts_y.begin()+pts_x_max_idx);
            pts_x_max_idx = std::max_element(pts_x.begin(),pts_x.end()) - pts_x.begin();
            pts_x.erase(pts_x.begin()+pts_x_max_idx);
            pts_y.erase(pts_y.begin()+pts_x_max_idx);

            int pts_y_min_idx = std::min_element(pts_y.begin(),pts_y.end()) - pts_y.begin();
            cv::Point2f pts_id0_tmp(pts_x[pts_y_min_idx], pts_y[pts_y_min_idx]);
            help_feat_.push_back(pts_id0_tmp);

            int pts_y_max_idx = std::max_element(pts_y.begin(),pts_y.end()) - pts_y.begin();
            cv::Point2f pts_id1_tmp(pts_x[pts_y_max_idx], pts_y[pts_y_max_idx]);
            help_feat_.push_back(pts_id1_tmp);

            pts_x.resize(0);
            pts_y.resize(0);
            for (int i=0; i<4; ++i)
            {
                pts_x.push_back(next_feat_[i].x);
                pts_y.push_back(next_feat_[i].y);
            }

            int pts_x_min_idx = std::min_element(pts_x.begin(),pts_x.end()) - pts_x.begin();
            pts_x.erase(pts_x.begin()+pts_x_min_idx);
            pts_y.erase(pts_y.begin()+pts_x_min_idx);
            pts_x_min_idx = std::min_element(pts_x.begin(),pts_x.end()) - pts_x.begin();
            pts_x.erase(pts_x.begin()+pts_x_min_idx);
            pts_y.erase(pts_y.begin()+pts_x_min_idx);

            pts_y_max_idx = std::max_element(pts_y.begin(),pts_y.end()) - pts_y.begin();
            cv::Point2f pts_id2_tmp(pts_x[pts_y_max_idx], pts_y[pts_y_max_idx]);
            help_feat_.push_back(pts_id2_tmp);

            pts_y_min_idx = std::min_element(pts_y.begin(),pts_y.end()) - pts_y.begin();
            cv::Point2f pts_id3_tmp(pts_x[pts_y_min_idx], pts_y[pts_y_min_idx]);
            help_feat_.push_back(pts_id3_tmp);

            std::cout << "help_feat_[0]" << " " << help_feat_[0] << std::endl;
            std::cout << "help_feat_[1]" << " " << help_feat_[1] << std::endl;
            std::cout << "help_feat_[2]" << " " << help_feat_[2] << std::endl;
            std::cout << "help_feat_[3]" << " " << help_feat_[3] << std::endl;

            next_feat_.resize(0);
            next_feat_.push_back(help_feat_[0]);
            next_feat_.push_back(help_feat_[1]);
            next_feat_.push_back(help_feat_[2]);
            next_feat_.push_back(help_feat_[3]);
            ///////////////////////////////////////////////////////////////////

            img_gray_original.copyTo(img0_);
            prev_feat_.resize(0);
            for (int i = 0; i < next_feat_.size(); ++i)
            {
                prev_feat_.push_back(next_feat_[i]);
            }
            ROS_INFO_STREAM(">>>>>>>>>> Initialization Complete <<<<<<<<<");

            // cv::imshow("img input", img_visual);
            // cv::imshow("img input", img_gray);
            // cv::waitKey(0);
        }
        else
        {
            
            help_feat_.resize(0);
            // check prev_feat vs. help_feat

            std::vector<float> diff_prev_and_help;
            
            // 이건 matching 문제가 있다.
            // for (int i = 0; i < prev_feat_.size(); ++i)
            // {
            //     diff_prev_and_help.resize(0);
            //     for (int j = 0; j < next_feat_.size(); ++j)
            //     {
            //         float diff_norm = std::sqrt( SQUARE(prev_feat_[i].x - next_feat_[j].x) + SQUARE(prev_feat_[i].y - next_feat_[j].y) );
            //         diff_prev_and_help.push_back(diff_norm);
            //         std::cout << diff_norm << " ";
            //     }
            //     std::cout << std::endl;
            //     int diff_min_idx = std::min_element(diff_prev_and_help.begin(), diff_prev_and_help.end()) - diff_prev_and_help.begin();
            //     //check 할당도 안했는데 [~]로 대입해주려고 하면 안됨.
            //     help_feat_.push_back(next_feat_[diff_min_idx]);
            //     std::cout << diff_min_idx <<std::endl;
            //     std::cout << next_feat_[diff_min_idx] <<std::endl;
            //     std::cout << help_feat_[i] <<std::endl;
            // }


            std::vector<float> diff_norm;
            
            for (int k = 0; k < perm_.size(); ++k)
            {
                float diff_norm_sum_tmp = 0;
                for (int p=0; p<4 ; ++p)
                {
                    int perm_k_p = perm_[k][p];
                    diff_norm_sum_tmp += std::sqrt( SQUARE(prev_feat_[p].x - next_feat_[perm_k_p].x) + SQUARE(prev_feat_[p].y - next_feat_[perm_k_p].y) );
                }
                diff_norm.push_back(diff_norm_sum_tmp);
            }
            int diff_min_idx = std::min_element(diff_norm.begin(), diff_norm.end()) - diff_norm.begin();

            for (int i=0; i<4; ++i)
            {
                help_feat_.push_back(next_feat_[perm_[diff_min_idx][i]]);
            }
            // exit(0);
            

            std::vector<unsigned char> status;
            std::vector<float> err;
            ROS_INFO_STREAM("before tracking");
            // std::cout << prev_feat_.size() << " " << help_feat_.size() << std::endl;
            std::cout << "help_feat_[0]" << " " << help_feat_[0] << std::endl;
            std::cout << "help_feat_[1]" << " " << help_feat_[1] << std::endl;
            std::cout << "help_feat_[2]" << " " << help_feat_[2] << std::endl;
            std::cout << "help_feat_[3]" << " " << help_feat_[3] << std::endl;

            int size_rec_half1 = 5;
            int size_rec_half2 = 10;
            int size_rec_half3 = 15;
            int size_rec_half4 = 20;

            cv::rectangle(img_visual, cv::Rect(cv::Point(prev_feat_[0].x-size_rec_half1, prev_feat_[0].y-size_rec_half1), cv::Point(prev_feat_[0].x+size_rec_half1, prev_feat_[0].y+size_rec_half1)), cv::Scalar(255, 0, 255), 1, 8, 0);
            cv::rectangle(img_visual, cv::Rect(cv::Point(prev_feat_[1].x-size_rec_half2, prev_feat_[1].y-size_rec_half2), cv::Point(prev_feat_[1].x+size_rec_half2, prev_feat_[1].y+size_rec_half2)), cv::Scalar(255, 0, 255), 1, 8, 0);
            cv::rectangle(img_visual, cv::Rect(cv::Point(prev_feat_[2].x-size_rec_half3, prev_feat_[2].y-size_rec_half3), cv::Point(prev_feat_[2].x+size_rec_half3, prev_feat_[2].y+size_rec_half3)), cv::Scalar(255, 0, 255), 1, 8, 0);
            cv::rectangle(img_visual, cv::Rect(cv::Point(prev_feat_[3].x-size_rec_half4, prev_feat_[3].y-size_rec_half4), cv::Point(prev_feat_[3].x+size_rec_half4, prev_feat_[3].y+size_rec_half4)), cv::Scalar(255, 0, 255), 1, 8, 0);


            cv::calcOpticalFlowPyrLK(img0_, img_gray_original, prev_feat_, help_feat_, status, err, cv::Size(51, 51), 0,
                                     {}, 4); // use OPTFLOW_USE_INITIAL_FLOW

            std::cout << "help_feat_[0]" << " " << help_feat_[0] << std::endl;
            std::cout << "help_feat_[1]" << " " << help_feat_[1] << std::endl;
            std::cout << "help_feat_[2]" << " " << help_feat_[2] << std::endl;
            std::cout << "help_feat_[3]" << " " << help_feat_[3] << std::endl;

            // help_feat_ == next_feat_

            // cv::imshow("img input", img_visual);
            // cv::imshow("img input", img_gray);
            // cv::waitKey(0);

            img_gray_original.copyTo(img0_);
            prev_feat_.resize(0);
            for (int i = 0; i < help_feat_.size(); ++i)
            {
                prev_feat_.push_back(help_feat_[i]);
            }

            // ROS_INFO_STREAM(">>>>>>>>>> " << "Iter " << iter_test <<" Complete <<<<<<<<<");
        }
    }
    else if (flag_line_detect == false)
    {
        ROS_INFO_STREAM("4 line detection fail");
    }

    // cv::imshow("img input", img_visual);
    // // cv::imshow("img input", img_gray);
    // cv::waitKey(0);

    // double dt_toc = timer::toc(1); // milliseconds
    // ROS_INFO_STREAM("total time :" << dt_toc << " [ms]");

    // if (dt_toc>30)
    // {
    //     cv::imshow("img input", skel);
    //     cv::waitKey(0);
    //     // exit(0);
    // }

    ROS_INFO_STREAM(">>>>>>>>>> "
                    << "Iter " << iter_test << " End <<<<<<<<<");
    iter_test += 1;
    if (iter_test == n_test_data_)
    {
        ROS_INFO_STREAM(" >>>>>>>>>> All data is done <<<<<<<<<");
        exit(0);
    }

    // for (int i = 0; i < help_feat_.size(); ++i)
    // {
    cv::circle(img_visual, help_feat_[0], 4, cv::Scalar(255, 0, 255), 1, 8, 0);
    cv::circle(img_visual, help_feat_[1], 6, cv::Scalar(255, 0, 255), 1, 8, 0);
    cv::circle(img_visual, help_feat_[2], 8, cv::Scalar(255, 0, 255), 1, 8, 0);
    cv::circle(img_visual, help_feat_[3], 10, cv::Scalar(255, 0, 255), 1, 8, 0);
    // }
    // cv::imshow("img input", img_visual);
    // if (iter_test>143)
    // cv::waitKey(0);
    // else
    cv::waitKey(5);

    reset_vector();
};

void MonoLineDetectorROS::reset_vector()
{
    points_x_.resize(0);
    points_y_.resize(0);
    line_a_.resize(0);
    line_b_.resize(0);

    // px_.resize(0);
    // py_.resize(0);
    next_feat_.resize(0);
    help_feat_.resize(0);
    // for (int i=0; i<24; ++i)
    // {
    //     perm_[i].resize(0);
    // }
};

void MonoLineDetectorROS::calcLineIntersection(float dir1_a, float dir1_b, float dir2_a, float dir2_b, float& px_tmp, float& py_tmp)
{
    px_tmp = (dir2_b - dir1_b) / (dir1_a - dir2_a);
    py_tmp = dir1_a * px_tmp + dir1_b;
};

void MonoLineDetectorROS::ransacLine(std::vector<int>& points_x, std::vector<int>& points_y, 
                                    /*output*/ bool mask_inlier[], std::vector<float>& line_a, std::vector<float>& line_b,
                                                std::vector<int>& inlier_result_x, std::vector<int>& inlier_result_y)
{
    float* ptr_line_a = line_a.data();
    float* ptr_line_b = line_b.data();

    int* ptr_points_x = points_x.data();
    int* ptr_points_y = points_y.data();

    int iter = param_RANSAC_.iter;
    float thr = param_RANSAC_.thr;
    int mini_inlier = param_RANSAC_.mini_inlier;

    int n_pts = points_x.size();

    // ROS_INFO_STREAM("here here here here " << n_pts);

    // bool id_good_fit[iter];
    // bool ini_inlier[iter][n_pts];
    // bool mask[iter][n_pts];
    float residual[iter][n_pts];
    ROS_INFO_STREAM("here here here here " << n_pts);
    int inlier_cnt[iter];

    std::vector<std::vector<int>> inlier_x;
    std::vector<std::vector<int>> inlier_y;
    
    inlier_x.resize(iter); // reserve 아니고 resize
    inlier_y.resize(iter);

    // std::vector<int> inlier_result_x;
    // std::vector<int> inlier_result_y;
    // inlier_result_x.reserve(n_pts);
    // inlier_result_y.reserve(n_pts);

    float line_A[iter];
    float line_B[iter];
    
    for (int i = 0; i < iter; ++i)
    {
        // id_good_fit[i] = false;
        inlier_cnt[i] = 0;
        for (int j = 0; j < n_pts; ++j)
        {
            // ini_inlier[i][j] = false;
            // mask[i][j] = false;
            residual[i][j] = 0.0;
        }

        inlier_x[i].reserve(n_pts);
        inlier_y[i].reserve(n_pts);
        line_A[i] = 0.0;
        line_B[i] = 0.0;
    }
    
    int n1 = 0;
    int n2 = 0;
    float x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0;

    // std::random_device rd;
    // std::mt19937 gen_(rd());
    std::uniform_int_distribution<int> dis(0, n_pts - 1);
    
    for (int m = 0; m < iter; ++m)
    {
        inlier_cnt[m] = 1;
    //     std::cout<< n_pts<<std::endl;
    //     cv::imshow("s",img_vec_[0]);
    // cv::waitKey(0);
        while(1)
        { 
            n1 = dis(gen_);
            n2 = dis(gen_);
            if (n1 != n2)
            {
                // std::cout<< 0 << "~" <<n_pts-1<<": "<<n1<<", "<<n2<<std::endl;
                break;
            }
        }
        x1 = ptr_points_x[n1];
        y1 = ptr_points_y[n1];
        x2 = ptr_points_x[n2];
        y2 = ptr_points_y[n2];

        line_A[m] = (y2 - y1) / (x2 - x1);
        std::cout << "line_A[m]: " << line_A[m] <<std::endl;
        if ( !isinf(line_A[m]) )
        {
            line_B[m] = -line_A[m] * x1 + y1;

            double den = 1.0 / std::sqrt(line_A[m] * line_A[m] + 1.0);

            for (int i = 0; i < n_pts; ++i)
            {
                residual[m][i] = std::abs(line_A[m] * ptr_points_x[i] - ptr_points_y[i] + line_B[m]) * den;

                if (residual[m][i] < thr)
                {
                    // ini_inlier[m][i] = true;
                    // mask[m][i] = true;
                    inlier_x[m].push_back(ptr_points_x[i]);
                    inlier_y[m].push_back(ptr_points_y[i]);
                    inlier_cnt[m] += 1;
                }
            }
        }
        else // isinf(line_A[m]
        {
            line_B[m] = INFINITY; //posible?

            for (int i = 0; i < n_pts; ++i)
            {
                residual[m][i] = std::abs( x2 - ptr_points_x[i] ); //x2 와 x1이 같으니까 slope가 inf 겠지?

                if (residual[m][i] < thr)
                {
                    // ini_inlier[m][i] = true;
                    // mask[m][i] = true;
                    inlier_x[m].push_back(ptr_points_x[i]);
                    inlier_y[m].push_back(ptr_points_y[i]);
                    inlier_cnt[m] += 1;
                }
            }
        }
        std::cout << "line_B[m]: " << line_B[m] <<std::endl;
        
        // if ((inlier_cnt[m] - 1) > mini_inlier_)
        // {
        //     id_good_fit[m] = true;
        // }
    }
    
    

    int max_inlier_cnt = 0;
    std::vector<int> max_inlier_cnt_index;
    max_inlier_cnt_index.reserve(100);

    max_inlier_cnt = *std::max_element(inlier_cnt, inlier_cnt + iter);

    for (int i=0; i<iter; ++i)
    {
        if (inlier_cnt[i] == max_inlier_cnt)
        {
            max_inlier_cnt_index.push_back(i);
        }
    }

    float mini_pre = 1e3;
    
    int id_mini = 0;
    int max_inlier_cnt_index_1 = 0;
    float inv_n_pts = 1 / n_pts;
    if (max_inlier_cnt_index.size() > 1)
    {
        int n_candi = max_inlier_cnt_index.size();
        for (int i_candi = 0; i_candi < n_candi; ++i_candi)
        {
            float mean_residual = 0.0;
            for (int k=0; k<n_pts; ++k)
            {
                mean_residual += residual[max_inlier_cnt_index[i_candi]][k];
            }

            mean_residual *= inv_n_pts;

            float mini = std::min(mean_residual, mini_pre);
            
            if (mini < mini_pre)
            {
                id_mini = i_candi;
            }
            mini_pre = mini;
        }
        max_inlier_cnt_index_1 = max_inlier_cnt_index[id_mini];
    }
    else
    {
        max_inlier_cnt_index_1 = max_inlier_cnt_index[0];
    }
    int best_n_inlier = inlier_cnt[max_inlier_cnt_index_1] - 1;

    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(best_n_inlier,3);
    for (int i=0; i<best_n_inlier; ++i)
    {
        A(i,0) = inlier_x[max_inlier_cnt_index_1][i];
        A(i,1) = inlier_y[max_inlier_cnt_index_1][i];
        A(i,2) = 1;

        inlier_result_x.push_back(A(i,0));
        inlier_result_y.push_back(A(i,1));
    }
    std::cout << "before ransac svd" <<std::endl;
    std::cout << "inlier_x.size(): " << inlier_x.size() << std::endl;
    std::cout << "inlier_y.size(): " << inlier_y.size() << std::endl;
    std::cout << "best_n_inlier: " << best_n_inlier << std::endl;
    Eigen::Vector3f t;
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    std::cout << "after ransac svd" <<std::endl;
    t = svd.matrixV().col(2);
        // std::cout << svd.matrixU() << std::endl;
        // std::cout << "***************" << std::endl;
        // std::cout << svd.matrixV() << std::endl;
        // std::cout << "***************" << std::endl;
        // std::cout << svd.singularValues() << std::endl;
        // exit(0);

    // Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // // cout << "Its singular values are:" << endl << svd.singularValues() << endl;
    // // cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
    // // cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;

    t = -t/t(1,0);
    std::cout << t <<std::endl;
    // Eigen::Vector2f line;
    // line << t(0), t(2); //y = ax + b
    
    line_a.push_back(t(0));
    line_b.push_back(t(2));

    for (int i=0; i<line_a.size(); ++i)
    {
        std::cout << "line a[" << i << "]" <<": " << line_a[i] <<std::endl;
        std::cout << "line b[" << i << "]" <<": " << line_b[i] <<std::endl;
    }

    float residual_leastsquare = 0.0;
    // int cnt_inlier = 0;

    double den_ls = 1.0 / std::sqrt(t(0)*t(0)+t(1)*t(1));
    for (int i=0; i<n_pts; ++i)
    {
        residual_leastsquare = std::abs(t(0)*ptr_points_x[i] - ptr_points_y[i] + t(2))* den_ls; //t(1)=-1

        if ((residual_leastsquare < thr))
        {
            mask_inlier[i] = true;
            // cnt_inlier += 1;
        }
    }
    // output: mask_inlier
    // std::cout << t(0) << " " << t(2) << " # of inlier: " << cnt_inlier <<std::endl;
    // double dt_toc2 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("AA :" << dt_toc2 << " [ms]");

};

void MonoLineDetectorROS::callbackImage(const sensor_msgs::ImageConstPtr& msg)
{
    // msg -> cv::Mat img
    ROS_INFO_STREAM("callback image");

    
    // cv::cvtColor(img_color, img_gray, CV_BGR2GRAY);

    // timer::tic();
    timer::tic();
    static int image_seq = 0;
    // bool flag_initialization = false ;

    ROS_INFO_STREAM(">>>>>>>>>> " << "Iter " << image_seq <<" Start <<<<<<<<<");

    // input image
    cv::Mat img_color;
    cv::Mat img_gray;
    cv::Mat img_channels[3];
    cv::Mat img_visual;
    cv::Mat img_gray_original;
    cv::Mat img_distort;
    
    // dilate & erode
    cv::Mat img_threshold;
    cv::Mat img_dilate;
    cv::Mat img_erode;

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

    cv_ptr->image.copyTo(img_gray);
    //for calibration
    img_gray.copyTo(img_distort);
    
    cv::remap(img_gray, img_color_undist_, map1_, map2_, cv::INTER_LINEAR);
    // cv_ptr->image.copyTo(img_gray);
    // cv::cvtColor(img_color_undist_, img_gray, CV_BGR2GRAY);

    img_color_undist_.copyTo(img_gray);

    //cvtColor 없으면 안되는걸 봐서 img_gray가 CV_GRAY가 아니다?
    // cv::cvtColor(img_color, img_gray, CV_BGR2GRAY);
    img_gray.convertTo(img_gray, CV_8UC1);
    img_gray.convertTo(img_visual, CV_8UC1);
    img_gray.convertTo(img_gray_original, CV_8UC1);

    int n_row = img_gray.rows;
    int n_col = img_gray.cols;

    //bwlabel
    cv::Mat object_label = cv::Mat::zeros(n_row, n_col, CV_32SC1);
    int *ptr_object_label = object_label.ptr<int>(0);
    cv::Mat stats, centroids;
    std::vector<int> sum_object;
    cv::Mat img_clone = cv::Mat::zeros(n_row, n_col, CV_8UC1);
    uchar *ptr_img_clone = img_clone.ptr<uchar>(0);

    bool flag_line_detect = false;
    
    std::vector<cv::Vec4f> lines;
    
    fast_line_detector_->detect(img_gray, lines);


    cv::Mat img_zero = cv::Mat::zeros(n_row, n_col, CV_8UC1);
    fast_line_detector_->drawSegments(img_zero, lines);

    // cv::imshow("img input", img_zero);
    // cv::waitKey(0);

    cv::split(img_zero, img_channels);
    img_channels[2].copyTo(img_gray);
    uchar *ptr_img_gray = img_gray.ptr<uchar>(0);
    // cv::imshow("B", img_channels[0]);
    // cv::imshow("G", img_channels[1]);
    // cv::imshow("R", img_channels[2]);

    // for (int i = 0; i < n_row; ++i)
    // {
    //     int i_ncols = i * n_col;
    //     for (int j = 0; j < n_col; ++j)
    //     {
    //         if (*(ptr_img_gray + i_ncols + j) < 255)
    //         {
    //             *(ptr_img_gray + i_ncols + j) = 0;
    //         }
    //     }
    // }

    for (int i = 360; i < n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j)
        {
                *(ptr_img_gray + i_ncols + j) = 0;
        }
    }

    // cv::threshold(img_gray, img_threshold, 180, 255, cv::THRESH_BINARY);
    cv::dilate(img_gray, img_dilate, cv::Mat::ones(cv::Size(10, 10), CV_8UC1));

    // cv::threshold(img_dilate, img_threshold, 180, 255, cv::THRESH_BINARY);
    cv::erode(img_dilate, img_erode, cv::Mat::ones(cv::Size(10, 10), CV_8UC1));

    // bwlabel
    int n_label = cv::connectedComponentsWithStats(img_erode, object_label, stats, centroids, 8);
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
            int i_ncols = i * n_col;
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
        int i_ncols = i * n_col;
        for (int j = obj_left; j < obj_left + obj_width; ++j)
        {
            if (*(ptr_object_label + i_ncols + j) == max_obj_pixel_idx)
            {
                *(ptr_img_clone + i_ncols + j) = 255;
            }
        }
    }
    // for (int i=360; i<img_clone.rows; ++i)
    // {
    //     int i_ncols = i * img_clone.rows;
    //     for (int j = 0; j < img_clone.rows; ++j)
    //     {
    //         *(ptr_img_clone + i_ncols + j) = 0;
    //     }
    // }
    ROS_INFO_STREAM("sum_obj: " << sum_object[0] << " " << sum_object[1] << " " << sum_object[2] << " " << sum_object[3]);
    ROS_INFO_STREAM("max_obj_pixel_idx: " << max_obj_pixel_idx);
    
    // skel
    cv::Mat skel = cv::Mat::zeros(n_row, n_col, CV_8UC1); //(img_clone.size(), CV_8UC1, cv::Scalar(0));
    // if (sum_object[max_obj_pixel_idx] > 5000)
    // {
    //     cv::erode(img_clone, img_clone, cv::Mat::ones(cv::Size(3, 3), CV_8UC1));
    // }

    // cv::ximgproc::thinning(img_clone, skel);
    Thinning(img_clone, n_row, n_col);
    img_clone.copyTo(skel);
    uchar *ptr_skel = skel.ptr<uchar>(0);

    // for (int i=360; i<skel.rows; ++i)
    // {
    //     int i_ncols = i * skel.cols;
    //     for (int j = 0; j < skel.cols; ++j)
    //     {
    //         *(ptr_skel + i_ncols + j) = 0;
    //     }
    // }

    // double dt_toc = timer::toc(1); // milliseconds
    // ROS_INFO_STREAM("total time :" << dt_toc << " [ms]");

    // if (dt_toc>30)
    // {
    //     cv::imshow("img input", skel);
    //     cv::waitKey(0);
    //     // exit(0);
    // }

    // cv::imshow("img input", skel);
    // cv::waitKey(0);

    for (int i = 0; i < n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j)
        {
            if (*(ptr_skel + i_ncols + j) != 0)
            {
                // std::cout << static_cast<int>(*(ptr_skel + i_ncols + j)) << std::endl;
                points_x_.push_back(j);
                points_y_.push_back(i);
            }
        }
    }
    // std::cout << points_x_.size() << std::endl;
    // if (points_x_.size() == 0)
    // {
    //     return;
    // }

    if (flag_cam_stream_==true)
    {
        cv::imshow("img kel", skel);
        cv::waitKey(5);
    }
    
    if (points_x_.size() > n_row * n_col *0.5)
    {
        this->reset_vector();
        return;
    }

    // std::cout << points_x_.size() <<std::endl;
    // std::cout << points_y_.size() <<std::endl;
    
    int n_points = points_x_.size();
    // bool mask_inlier[n_points];
    // std::vector<int> points_x_tmp_;
    // std::vector<int> points_y_tmp_;

    points_x_tmp_.resize(0);
    points_y_tmp_.resize(0);
    // std::copy(points_x_.begin(), points_x_.end(), points_x_tmp_.begin());
    // std::copy(points_y_.begin(), points_y_.end(), points_y_tmp_.begin());
    for (int i = 0; i < n_points; ++i)
    {
        points_x_tmp_.push_back(points_x_[i]);
        points_y_tmp_.push_back(points_y_[i]);
    }
    // for (int i=0; i<points_x_tmp_.size(); ++i)
    // {
    //     std::cout<< "points_x_tmp_: " << points_x_tmp_[i]<<std::endl;
    // }

    // std::vector<int> points_x_tmp2_;
    // std::vector<int> points_y_tmp2_;

    //     if (lines.size() == 0)
    // {
    //     ROS_INFO_STREAM("no line detected hoho");
    // }

    // 4 line detection
    for (int i = 0; i < 4; ++i)
    {
        points_x_tmp2_.resize(0);
        points_y_tmp2_.resize(0);
        inlier_result_x_.resize(0);
        inlier_result_y_.resize(0);

        int n_pts_tmp = points_x_tmp_.size();
        std::cout << "# of remaining pts: " << n_pts_tmp << std::endl;
        bool mask_inlier[n_pts_tmp];
        for (int q = 0; q < n_pts_tmp; ++q)
        {
            mask_inlier[q] = false;
        }
        int param_RANSAC_iter = 25;
        int param_RANSAC_thr = 3;
        int param_RANSAC_mini_inlier = 30;
        ROS_INFO_STREAM("no line detected hoho iter: "<<i);
        // std::cout << points_x_tmp_.size() << " " << points_y_tmp_.size() << std::endl;
        if (n_pts_tmp > n_row * n_col)
        {
            ROS_INFO_STREAM("n_pts > n_row * n_col");
            flag_line_detect = false;
            exit(0);
            break;
        }
        ransacLine(points_x_tmp_, points_y_tmp_, /*output*/ mask_inlier, line_a_, line_b_, inlier_result_x_, inlier_result_y_);
        // std::cout<< "line_a_: " << line_a_[0]<< "line_b_: " << line_b_[0]<<std::endl;
        ROS_INFO_STREAM("no line detected haha iter: "<<i);

        ///////// visualization (from)
        // slope inf에 대한 대처
        if (std::abs(line_b_[i]) > 1e4)
        {
            float points_y_tmp_min = std::min_element(inlier_result_y_.begin(), inlier_result_y_.end()) - inlier_result_y_.begin();
            float points_y_tmp_max = std::max_element(inlier_result_y_.begin(), inlier_result_y_.end()) - inlier_result_y_.begin();
            cv::Point2f p0(inlier_result_x_[points_y_tmp_min], inlier_result_y_[points_y_tmp_min]);
            cv::Point2f p1(inlier_result_x_[points_y_tmp_max], inlier_result_y_[points_y_tmp_max]);
            cv::line(img_visual, p0, p1, cv::Scalar(255, 0, 255), 1);
        }
        else
        {
            float points_x_tmp_min = *std::min_element(inlier_result_x_.begin(), inlier_result_x_.end());
            float points_x_tmp_max = *std::max_element(inlier_result_x_.begin(), inlier_result_x_.end());
            float points_y_tmp_min = line_a_[i] * points_x_tmp_min + line_b_[i];
            float points_y_tmp_max = line_a_[i] * points_x_tmp_max + line_b_[i];
            cv::Point2f p0(points_x_tmp_min, points_y_tmp_min);
            cv::Point2f p1(points_x_tmp_max, points_y_tmp_max);
            cv::line(img_visual, p0, p1, cv::Scalar(255, 0, 255), 1);
        }
        ///////// visualization (to)

        for (int q = 0; q < n_pts_tmp; ++q)
        {
            if (mask_inlier[q] == false)
            {
                points_x_tmp2_.push_back(points_x_tmp_[q]);
                points_y_tmp2_.push_back(points_y_tmp_[q]);
            }
        }
        points_x_tmp_.resize(0);
        points_y_tmp_.resize(0);
        for (int i = 0; i < points_x_tmp2_.size(); ++i)
        {
            points_x_tmp_.push_back(points_x_tmp2_[i]);
            points_y_tmp_.push_back(points_y_tmp2_[i]);
        }

        flag_line_detect = true;

        if (points_x_tmp_.size() < 30 && i < 3)
        {
            flag_line_detect = false;
            break;
        }
    }

    if (flag_line_detect == true)
    {
        for (int i = 0; i < line_a_.size(); ++i)
        {
            std::cout << "line_a_[" << i << "]: " << line_a_[i] << "  "
                      << "line_b_[" << i << "]: " << line_b_[i] << std::endl;
        }

        std::vector<float> line_a_abs;
        for (int i = 0; i < line_a_.size(); ++i)
        {
            line_a_abs.push_back(SQUARE(line_a_[i]));
        }

        // intersection points
        int min_a_abs_idx = std::min_element(line_a_abs.begin(), line_a_abs.end()) - line_a_abs.begin();
        
        ROS_INFO_STREAM("min_a_abs_idx: " << min_a_abs_idx);

        std::vector<float> diff_b;
        float b_min_a_idx = line_b_[min_a_abs_idx];

        for (int i = 0; i < line_b_.size(); ++i)
        {
            if (i == min_a_abs_idx)
            {
                diff_b.push_back(1e7);
            }
            else
            {
                diff_b.push_back(std::abs(line_b_[i] - b_min_a_idx));
            }
            std::cout << diff_b[i] << std::endl;
        }
        int min_diff_b_idx = std::min_element(diff_b.begin(), diff_b.end()) - diff_b.begin();
        std::cout << min_diff_b_idx << std::endl;

        std::vector<float> dir1_line_a;
        std::vector<float> dir1_line_b;

        std::vector<float> dir2_line_a;
        std::vector<float> dir2_line_b;

        for (int i = 0; i < line_a_.size(); ++i)
        {
            if (i == min_a_abs_idx)
            {
                dir1_line_a.push_back(line_a_[i]);
                dir1_line_b.push_back(line_b_[i]);
            }
            else if (i == min_diff_b_idx)
            {
                dir1_line_a.push_back(line_a_[i]);
                dir1_line_b.push_back(line_b_[i]);
            }
            else
            {
                dir2_line_a.push_back(line_a_[i]);
                dir2_line_b.push_back(line_b_[i]);
            }
        }
        
        for (int i = 0; i < dir1_line_a.size(); ++i)
        {
            float dir1_a = dir1_line_a[i];
            float dir1_b = dir1_line_b[i];

            for (int j = 0; j < dir2_line_a.size(); ++j)
            {
                float dir2_a = dir2_line_a[j];
                float dir2_b = dir2_line_b[j];
                float px_tmp = 0.0;
                float py_tmp = 0.0;
                calcLineIntersection(dir1_a, dir1_b, dir2_a, dir2_b, px_tmp, py_tmp);
                cv::Point2f points_tmp(px_tmp, py_tmp);
                next_feat_.push_back(points_tmp);
                // px_.push_back(px_tmp);
                // py_.push_back(py_tmp);
                std::cout << "dir1_a: " << dir1_a << ", "
                          << "dir1_b: " << dir1_b << std::endl;
                std::cout << "dir2_a: " << dir2_a << ", "
                          << "dir2_b: " << dir2_b << std::endl;
                std::cout << "points_tmp: "
                          << "(" << points_tmp.x << ", " << points_tmp.y << ")" << std::endl;
            }
        }
        // for (int i = 0; i < next_feat_.size(); ++i)
        // {
        //     cv::circle(img_visual, next_feat_[i], 10, cv::Scalar(255, 0, 255), 1, 8, 0);
        // }

        if (flag_init_ == false)
        {
            flag_init_ = true;

            std::cout << "next_feat_[0]" << " " << next_feat_[0] << std::endl;
            std::cout << "next_feat_[1]" << " " << next_feat_[1] << std::endl;
            std::cout << "next_feat_[2]" << " " << next_feat_[2] << std::endl;
            std::cout << "next_feat_[3]" << " " << next_feat_[3] << std::endl;

            // 왼쪽 위 -> 왼쪽 아래 -> 오른쪽 아래 -> 오른쪽 위 : 순서대로 id0 id1 id2 id3
            std::vector<float> pts_x;
            std::vector<float> pts_y;
            for (int i=0; i<4; ++i)
            {
                pts_x.push_back(next_feat_[i].x);
                pts_y.push_back(next_feat_[i].y);
            }
            int pts_x_max_idx = std::max_element(pts_x.begin(),pts_x.end()) - pts_x.begin();
            pts_x.erase(pts_x.begin()+pts_x_max_idx);
            pts_y.erase(pts_y.begin()+pts_x_max_idx);
            pts_x_max_idx = std::max_element(pts_x.begin(),pts_x.end()) - pts_x.begin();
            pts_x.erase(pts_x.begin()+pts_x_max_idx);
            pts_y.erase(pts_y.begin()+pts_x_max_idx);

            int pts_y_min_idx = std::min_element(pts_y.begin(),pts_y.end()) - pts_y.begin();
            cv::Point2f pts_id0_tmp(pts_x[pts_y_min_idx], pts_y[pts_y_min_idx]);
            help_feat_.push_back(pts_id0_tmp);

            int pts_y_max_idx = std::max_element(pts_y.begin(),pts_y.end()) - pts_y.begin();
            cv::Point2f pts_id1_tmp(pts_x[pts_y_max_idx], pts_y[pts_y_max_idx]);
            help_feat_.push_back(pts_id1_tmp);

            pts_x.resize(0);
            pts_y.resize(0);
            for (int i=0; i<4; ++i)
            {
                pts_x.push_back(next_feat_[i].x);
                pts_y.push_back(next_feat_[i].y);
            }

            int pts_x_min_idx = std::min_element(pts_x.begin(),pts_x.end()) - pts_x.begin();
            pts_x.erase(pts_x.begin()+pts_x_min_idx);
            pts_y.erase(pts_y.begin()+pts_x_min_idx);
            pts_x_min_idx = std::min_element(pts_x.begin(),pts_x.end()) - pts_x.begin();
            pts_x.erase(pts_x.begin()+pts_x_min_idx);
            pts_y.erase(pts_y.begin()+pts_x_min_idx);

            pts_y_max_idx = std::max_element(pts_y.begin(),pts_y.end()) - pts_y.begin();
            cv::Point2f pts_id2_tmp(pts_x[pts_y_max_idx], pts_y[pts_y_max_idx]);
            help_feat_.push_back(pts_id2_tmp);

            pts_y_min_idx = std::min_element(pts_y.begin(),pts_y.end()) - pts_y.begin();
            cv::Point2f pts_id3_tmp(pts_x[pts_y_min_idx], pts_y[pts_y_min_idx]);
            help_feat_.push_back(pts_id3_tmp);

            std::cout << "help_feat_[0]" << " " << help_feat_[0] << std::endl;
            std::cout << "help_feat_[1]" << " " << help_feat_[1] << std::endl;
            std::cout << "help_feat_[2]" << " " << help_feat_[2] << std::endl;
            std::cout << "help_feat_[3]" << " " << help_feat_[3] << std::endl;

            next_feat_.resize(0);
            next_feat_.push_back(help_feat_[0]);
            next_feat_.push_back(help_feat_[1]);
            next_feat_.push_back(help_feat_[2]);
            next_feat_.push_back(help_feat_[3]);
            ///////////////////////////////////////////////////////////////////
            
            img_gray_original.copyTo(img0_);
            prev_feat_.resize(0);
            for (int i = 0; i < next_feat_.size(); ++i)
            {
                prev_feat_.push_back(next_feat_[i]);
            }
            ROS_INFO_STREAM(">>>>>>>>>> Initialization Complete <<<<<<<<<");

            // cv::imshow("img input", img_visual);
            // cv::imshow("img input", img_gray);
            // cv::waitKey(0);
        }
        else
        {
            
            help_feat_.resize(0);
            // check prev_feat vs. help_feat
            
            std::cout << "next_feat_[0]" << " " << next_feat_[0] << std::endl;
            std::cout << "next_feat_[1]" << " " << next_feat_[1] << std::endl;
            std::cout << "next_feat_[2]" << " " << next_feat_[2] << std::endl;
            std::cout << "next_feat_[3]" << " " << next_feat_[3] << std::endl;
            // 왼쪽 위 -> 왼쪽 아래 -> 오른쪽 아래 -> 오른쪽 위 : 순서대로 id0 id1 id2 id3
            std::vector<float> pts_x;
            std::vector<float> pts_y;
            for (int i=0; i<4; ++i)
            {
                pts_x.push_back(next_feat_[i].x);
                pts_y.push_back(next_feat_[i].y);
            }
            int pts_x_max_idx = std::max_element(pts_x.begin(),pts_x.end()) - pts_x.begin();
            pts_x.erase(pts_x.begin()+pts_x_max_idx);
            pts_y.erase(pts_y.begin()+pts_x_max_idx);
            pts_x_max_idx = std::max_element(pts_x.begin(),pts_x.end()) - pts_x.begin();
            pts_x.erase(pts_x.begin()+pts_x_max_idx);
            pts_y.erase(pts_y.begin()+pts_x_max_idx);

            int pts_y_min_idx = std::min_element(pts_y.begin(),pts_y.end()) - pts_y.begin();
            cv::Point2f pts_id0_tmp(pts_x[pts_y_min_idx], pts_y[pts_y_min_idx]);
            help_feat_.push_back(pts_id0_tmp);

            int pts_y_max_idx = std::max_element(pts_y.begin(),pts_y.end()) - pts_y.begin();
            cv::Point2f pts_id1_tmp(pts_x[pts_y_max_idx], pts_y[pts_y_max_idx]);
            help_feat_.push_back(pts_id1_tmp);

            pts_x.resize(0);
            pts_y.resize(0);
            for (int i=0; i<4; ++i)
            {
                pts_x.push_back(next_feat_[i].x);
                pts_y.push_back(next_feat_[i].y);
            }

            int pts_x_min_idx = std::min_element(pts_x.begin(),pts_x.end()) - pts_x.begin();
            pts_x.erase(pts_x.begin()+pts_x_min_idx);
            pts_y.erase(pts_y.begin()+pts_x_min_idx);
            pts_x_min_idx = std::min_element(pts_x.begin(),pts_x.end()) - pts_x.begin();
            pts_x.erase(pts_x.begin()+pts_x_min_idx);
            pts_y.erase(pts_y.begin()+pts_x_min_idx);

            pts_y_max_idx = std::max_element(pts_y.begin(),pts_y.end()) - pts_y.begin();
            cv::Point2f pts_id2_tmp(pts_x[pts_y_max_idx], pts_y[pts_y_max_idx]);
            help_feat_.push_back(pts_id2_tmp);

            pts_y_min_idx = std::min_element(pts_y.begin(),pts_y.end()) - pts_y.begin();
            cv::Point2f pts_id3_tmp(pts_x[pts_y_min_idx], pts_y[pts_y_min_idx]);
            help_feat_.push_back(pts_id3_tmp);

            std::cout << "help_feat_[0]" << " " << help_feat_[0] << std::endl;
            std::cout << "help_feat_[1]" << " " << help_feat_[1] << std::endl;
            std::cout << "help_feat_[2]" << " " << help_feat_[2] << std::endl;
            std::cout << "help_feat_[3]" << " " << help_feat_[3] << std::endl;

            next_feat_.resize(0);
            next_feat_.push_back(help_feat_[0]);
            next_feat_.push_back(help_feat_[1]);
            next_feat_.push_back(help_feat_[2]);
            next_feat_.push_back(help_feat_[3]);
            // ///////////////////////////////////////////////////////////////////

            // help_feat_.resize(0);

            // std::vector<float> diff_prev_and_help;
            
            // // 이건 matching 문제가 있다.
            // // for (int i = 0; i < prev_feat_.size(); ++i)
            // // {
            // //     diff_prev_and_help.resize(0);
            // //     for (int j = 0; j < next_feat_.size(); ++j)
            // //     {
            // //         float diff_norm = std::sqrt( SQUARE(prev_feat_[i].x - next_feat_[j].x) + SQUARE(prev_feat_[i].y - next_feat_[j].y) );
            // //         diff_prev_and_help.push_back(diff_norm);
            // //         std::cout << diff_norm << " ";
            // //     }
            // //     std::cout << std::endl;
            // //     int diff_min_idx = std::min_element(diff_prev_and_help.begin(), diff_prev_and_help.end()) - diff_prev_and_help.begin();
            // //     //check 할당도 안했는데 [~]로 대입해주려고 하면 안됨.
            // //     help_feat_.push_back(next_feat_[diff_min_idx]);
            // //     std::cout << diff_min_idx <<std::endl;
            // //     std::cout << next_feat_[diff_min_idx] <<std::endl;
            // //     std::cout << help_feat_[i] <<std::endl;
            // // }


            // std::vector<float> diff_norm;
            
            // for (int k = 0; k < perm_.size(); ++k)
            // {
            //     float diff_norm_sum_tmp = 0;
            //     for (int p=0; p<4 ; ++p)
            //     {
            //         int perm_k_p = perm_[k][p];
            //         diff_norm_sum_tmp += std::sqrt( SQUARE(prev_feat_[p].x - next_feat_[perm_k_p].x) + SQUARE(prev_feat_[p].y - next_feat_[perm_k_p].y) );
            //     }
            //     diff_norm.push_back(diff_norm_sum_tmp);
            // }
            // int diff_min_idx = std::min_element(diff_norm.begin(), diff_norm.end()) - diff_norm.begin();

            // for (int i=0; i<4; ++i)
            // {
            //     help_feat_.push_back(next_feat_[perm_[diff_min_idx][i]]);
            // }
            // exit(0);
            

            std::vector<unsigned char> status;
            std::vector<float> err;
            ROS_INFO_STREAM("before tracking");
            // std::cout << prev_feat_.size() << " " << help_feat_.size() << std::endl;
            std::cout << "help_feat_[0]" << " " << help_feat_[0] << std::endl;
            std::cout << "help_feat_[1]" << " " << help_feat_[1] << std::endl;
            std::cout << "help_feat_[2]" << " " << help_feat_[2] << std::endl;
            std::cout << "help_feat_[3]" << " " << help_feat_[3] << std::endl;

            int size_rec_half1 = 5;
            int size_rec_half2 = 10;
            int size_rec_half3 = 15;
            int size_rec_half4 = 20;

            cv::rectangle(img_visual, cv::Rect(cv::Point(prev_feat_[0].x-size_rec_half1, prev_feat_[0].y-size_rec_half1), cv::Point(prev_feat_[0].x+size_rec_half1, prev_feat_[0].y+size_rec_half1)), cv::Scalar(255, 0, 255), 1, 8, 0);
            cv::rectangle(img_visual, cv::Rect(cv::Point(prev_feat_[1].x-size_rec_half2, prev_feat_[1].y-size_rec_half2), cv::Point(prev_feat_[1].x+size_rec_half2, prev_feat_[1].y+size_rec_half2)), cv::Scalar(255, 0, 255), 1, 8, 0);
            cv::rectangle(img_visual, cv::Rect(cv::Point(prev_feat_[2].x-size_rec_half3, prev_feat_[2].y-size_rec_half3), cv::Point(prev_feat_[2].x+size_rec_half3, prev_feat_[2].y+size_rec_half3)), cv::Scalar(255, 0, 255), 1, 8, 0);
            cv::rectangle(img_visual, cv::Rect(cv::Point(prev_feat_[3].x-size_rec_half4, prev_feat_[3].y-size_rec_half4), cv::Point(prev_feat_[3].x+size_rec_half4, prev_feat_[3].y+size_rec_half4)), cv::Scalar(255, 0, 255), 1, 8, 0);


            cv::calcOpticalFlowPyrLK(img0_, img_gray_original, prev_feat_, help_feat_, status, err, cv::Size(51, 51), 0,
                                     {}, 4); // use OPTFLOW_USE_INITIAL_FLOW

            std::cout << "help_feat_[0]" << " " << help_feat_[0] << std::endl;
            std::cout << "help_feat_[1]" << " " << help_feat_[1] << std::endl;
            std::cout << "help_feat_[2]" << " " << help_feat_[2] << std::endl;
            std::cout << "help_feat_[3]" << " " << help_feat_[3] << std::endl;

            // help_feat_ == next_feat_

            // cv::imshow("img input", img_visual);
            // // cv::imshow("img input", img_gray);
            // cv::waitKey(0);

            img_gray_original.copyTo(img0_);
            prev_feat_.resize(0);
            for (int i = 0; i < help_feat_.size(); ++i)
            {
                prev_feat_.push_back(help_feat_[i]);
            }

            // ROS_INFO_STREAM(">>>>>>>>>> " << "Iter " << image_seq <<" Complete <<<<<<<<<");
        }
    }
    else if (flag_line_detect == false)
    {
        ROS_INFO_STREAM("4 line detection fail");

        flag_init_ = false;
    }

    // cv::imshow("img input", img_visual);
    // // cv::imshow("img input", img_gray);
    // cv::waitKey(0);

    double *ptr_cameraMatrix = cameraMatrix_.ptr<double>(0);

    geometry_msgs::PoseArray  posearray;
    posearray.header.stamp = ros::Time::now(); // timestamp of creation of the msg
    posearray.header.frame_id = "map"; // frame id in which the array is published
    geometry_msgs::Pose p; // one pose to put in the array

    for (int i = 0; i < 4; ++i)
    {
        p.position.x = ( (help_feat_[i].x) - (*(ptr_cameraMatrix + 2)) ) / *(ptr_cameraMatrix + 0); // u = (x - cx)/fx
        p.position.y = ( (help_feat_[i].y) - (*(ptr_cameraMatrix + 5)) ) / *(ptr_cameraMatrix + 4); // v = (y - cy)/fy
        p.position.z = i;
        // push in array
        posearray.poses.push_back(p);
    }

    pub_projected_points_.publish(posearray);

    double dt_toc = timer::toc(1); // milliseconds
    ROS_INFO_STREAM("total time :" << dt_toc << " [ms]");

    // if (dt_toc>30)
    // {
    //     cv::imshow("img input", skel);
    //     cv::waitKey(0);
    //     // exit(0);
    // }

    /* for calibration (save image)
    std::string image_folder = "/home/junhakim/mono_calibration/data3/";
    std::string image_name = std::to_string(image_seq);
    std::string save_path = image_folder + image_name + ".png";
    std::cout << save_path << std::endl;
    cv::imwrite(save_path, img_distort);
    // */

    ROS_INFO_STREAM(">>>>>>>>>> "
                    << "Iter " << image_seq << " End <<<<<<<<<");
    image_seq += 1;
    // if (image_seq == n_test_data_)
    // {
    //     ROS_INFO_STREAM(" >>>>>>>>>> All data is done <<<<<<<<<");
    //     exit(0);
    // }

    // for (int i = 0; i < help_feat_.size(); ++i)
    // {
    cv::circle(img_visual, help_feat_[0], 5, cv::Scalar(255, 0, 255), 1, 8, 0);
    cv::circle(img_visual, help_feat_[1], 10, cv::Scalar(255, 0, 255), 1, 8, 0);
    cv::circle(img_visual, help_feat_[2], 15, cv::Scalar(255, 0, 255), 1, 8, 0);
    cv::circle(img_visual, help_feat_[3], 20, cv::Scalar(255, 0, 255), 1, 8, 0);
    // }

    if(flag_cam_stream_==true)
    {
        cv::imshow("img input", img_visual);
        cv::waitKey(5);
    }

    reset_vector();

    // Done.
};

void MonoLineDetectorROS::Thinning(cv::Mat input, int row, int col)
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

void MonoLineDetectorROS::permutation(std::vector<int>& input_vec, int depth, int n, int r)
{
    if (depth == r)
    {
        static int k = 0;
        for (int i = 0; i < r; i++)
        {   
            perm_[k].push_back(input_vec[i]);
            // std::cout << "depth: " << depth << " " << "i: " << i << " " << "input_vec[i]: " << input_vec[i] <<std::endl;
        }
        k += 1;
        return;
    }

    for (int i = depth; i < n; i++)
    {
        std::swap(input_vec[depth], input_vec[i]);         // 스왑
        permutation(input_vec, depth + 1, n, r); // 재귀
        std::swap(input_vec[depth], input_vec[i]);         // 다시 원래 위치로 되돌리기
    }
};