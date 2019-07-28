#ifndef GATEDETECTOR
#define GATEDETECTOR

#include "Detector.hpp"

class GateDetector : public Detector
{
public:
    GateDetector();
    ~GateDetector();

    bool Update(cv::Mat frame, uint8_t camera);

private:
    cv::Point findGateDivider(cv::Mat frame);
    cv::Point avgPoint(std::vector<cv::Point2f> list);

    CameraInput camera_input;
};
