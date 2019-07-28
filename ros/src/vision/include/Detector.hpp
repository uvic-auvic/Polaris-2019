#include "opencv2/opencv.hpp"
#include "CameraInput.hpp"

class Detector
{
public:
    virtual bool update() = 0;

    uint16_t getXFront() const { return distance_x_front; }
    uint16_t getYFront() const { return distance_y_front; }
    uint16_t getZFront() const { return distance_z_front; }
    uint16_t getXBottom() const { return distance_x_bottom; }
    uint16_t getYBottom() const { return distance_y_bottom; }
    uint16_t getZBottom() const { return distance_z_front; }
    uint16_t getXTop() const { return distance_x_top; }
    uint16_t getYTop() const { return distance_y_top; }
    uint16_t getZTop() const { return distance_z_top; }
    // bool errorFound() const { return error_found; }
    // std::string getError() const { return error_type; }
    // void enable() { this->enabled = true; }
    // void disable() { this->enabled = false; }
    // bool isEnabled() const { return enabled; }

protected:

    cv::CascadeClassifier cascade;
    
    cv::Rect object_front;
    cv::Rect object_bottom;
    cv::Rect object_top;

    uint32_t distance_x_front = 0;
    uint32_t distance_y_front = 0;
    uint32_t distance_z_front = 0;
    uint32_t distance_x_bottom = 0;
    uint32_t distance_y_bottom = 0;
    uint32_t distance_z_bottom = 0;
    uint32_t distance_x_top = 0;
    uint32_t distance_y_top = 0;
    uint32_t distance_z_top = 0;

    bool object_found_front = false;
    bool object_found_bottom = false;
    bool object_found_top = false;
    // bool error_found = false;
    // std::string error_type;
    bool enabled_front = false;
    bool enabled_bottom = false;
    bool enabled_top = false;
};

