#include <stdio.h>

#include "opencv2/opencv.hpp"
#include "opencv2/xfeatures2d.hpp"


class BuoyDetector
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
    cv::VideoCapture cap;
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
    BuoyDetector(const Buoy_t, const cv::VideoCapture);
    ~BuoyDetector();
    bool FindBuoy();
    cv::Rect GetRect();
    void Demo();
};