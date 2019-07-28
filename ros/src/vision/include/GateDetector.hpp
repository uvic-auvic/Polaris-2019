#ifndef GATEDETECTOR
#define GATEDETECTOR

#include "Detector.hpp"
#include "opencv2/xfeatures2d.hpp"

class GateDetector {
public:
    GateDetector();
    ~GateDetector();
    
    struct results{
        uint32_t distance_y;
        uint32_t distance_x;
        uint32_t distance_z;
    };

    GateDetector::results findGate(cv::Mat frame, uint8_t camera);
    cv::Point findGateDivider(cv::Mat frame);
    cv::Point avgPoint(std::vector<cv::Point2f> list);

private:


};
