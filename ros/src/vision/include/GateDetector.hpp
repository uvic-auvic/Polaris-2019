#ifndef GATEDETECTOR
#define GATEDETECTOR

#include "Detector.hpp"

class GateDetector {
public:
    GateDetector();
    ~GateDetector();

    cv::Rect findGate(cv::Mat frame);
    cv::Point findGateDivider(cv::Mat frame);
    cv::Point avgPoint(std::vector<cv::Point2f> list);

private:


};
