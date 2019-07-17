#ifndef DISTANCE
#define DISTANCE

#define FRONT_FOCAL 380.65
#define TOP_FOCAL 481.81
#define BOTTOM_FOCAL 0

#include "opencv2/opencv.hpp"

class Distance
{
public:
    uint32_t getDistanceZ(cv::Rect object, double realWidth, u_int8_t camera) {return z_distance; }
    uint32_t getDistanceX(cv::Rect object, double realWidth, cv::Mat frame) {return x_distance; }
    uint32_t getDistanceY(cv::Rect object, double realHieght, cv::Mat frame) {return y_distance; }

protected:
uint32_t z_distance = 0;
uint32_t x_distance = 0;
uint32_t y_distance = 0;

};

#endif