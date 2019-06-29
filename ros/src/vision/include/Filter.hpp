#ifndef FILTER
#define FILTER

#include "opencv2/opencv.hpp"
#include <cmath>

class Filter
{
public:
    void updateFilter(std::vector<cv::Rect> locations);
    uint16_t getBestX() const { return x; }
    uint16_t getBestY() const { return y; }

private:
    uint16_t x = 0;
    uint16_t y = 0;

    struct point_data {
        cv::Point p;
        uint8_t n;
    };

    std::vector<point_data> points;
    uint8_t iteration = 0;
    const uint16_t radius = 100;
    const uint8_t reset_time = 8;
    
    void updateBestPoint();
    void addPoint(cv::Point point);
    void removePoint();
};

#endif