#include <stdio.h>

#include "opencv2/opencv.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "Detector.hpp"
#include "Distance.hpp"


class BuoyDetector : public Detector, Distance
{
private:
    typedef enum
    {
        Jiangshi,
        Aswang,
        Draugr,
        Vetalas
    } Buoy_t;
    bool found_buoy = false;
    u_int32_t distance_x;
    CameraInput& camera_input;
    Buoy_t buoy;
    u_int8_t min_match_count = 10;
    float ratio_thresh = 0.6f; // ratio for Lowe's ratio test
    cv::Rect buoy_rect; // rectangle around buoy
    
    struct Detector {
        cv::Mat buoy_img;
        cv::Ptr<cv::xfeatures2d::SIFT> sift;
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;
        cv::Ptr<cv::DescriptorMatcher> matcher;
    };  
    Detector detector;

public:
    BuoyDetector(CameraInput& input, std::string cascade_name);
    ~BuoyDetector();
    bool Update();
    cv::Rect GetRect();
};