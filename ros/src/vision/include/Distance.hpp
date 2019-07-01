#ifndef DISTANCE
#define DISTANCE

#define FRONT_FOCAL 380.65
#define TOP_FOCAL 481.81
#define BOTTOM_FOCAL 0

#include "opencv2/opencv.hpp"

class Distance
{
public:
    double getDistance(std::vector<cv::Rect> locations);

private:

};

#endif