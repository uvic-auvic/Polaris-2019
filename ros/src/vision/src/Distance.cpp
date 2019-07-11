#include "Distance.hpp"

u_int32_t Distance::getDistanceZ(cv::Rect object, double realWidth, u_int8_t camera)
{   
    double focal = 0;
    if (camera == 0){
        focal = TOP_FOCAL;
    } else if(camera == 1){
        focal = FRONT_FOCAL;
    } else {
        focal = BOTTOM_FOCAL;
    }
    
    u_int32_t distance = (realWidth*focal)/object.width;

    return distance;
}

u_int32_t Distance::getDistanceX(cv::Rect object, double realWidth, cv::Mat frame)
{   

    cv::Point center(object.x + object.width/2, object.y - object.height/2);
    
    u_int32_t scale_factor = object.width/realWidth;

    u_int32_t x_dis = abs(center.x - frame.cols/2)/scale_factor;
        if (center.x - frame.cols/2 < 0){
            x_dis = -x_dis;
        }

    return x_dis;
}

u_int32_t Distance::getDistanceY(cv::Rect object, double realHieght, cv::Mat frame)
{   
    cv::Point center(object.x + object.width/2, object.y - object.height/2);
    int scale_factor = object.height/realHieght;
    u_int32_t y_dis = abs(center.y - frame.rows/2)/scale_factor;
    if (center.y - frame.rows/2 > 0){
            y_dis = -y_dis;
      }
    return y_dis;
}