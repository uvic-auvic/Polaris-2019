#ifndef DISTANCE
#define DISTANCE

#define FRONT_FOCAL 380.65
#define TOP_FOCAL 481.81
#define BOTTOM_FOCAL 0

#include "opencv2/opencv.hpp"

class Distance
{
public:
    u_int32_t getDistanceZ(cv::Rect object, double realWidth, u_int8_t camera);
    u_int32_t getDistanceX(cv::Rect object, double realWidth, cv::Mat frame);
    u_int32_t getDistanceY(cv::Rect object, double realHieght, cv::Mat frame);
private:

};

#endif