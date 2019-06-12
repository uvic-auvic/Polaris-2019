#include "opencv2/opencv.hpp"
#include "CameraInput.cpp"

class Detector
{
public:
    virtual bool update() = 0;

    // Only needs to be called by one object
    bool updateCameraInput() { camera_input.update(); }

    uint16_t getX() const { return x; }
    uint16_t getY() const { return y; }
    bool objectFound() const { return object_found; }
    bool errorFound() const { return error_found; }
    std::string getError() const { return error_type; }
    void enable() { this->enabled = true; }
    void disable() { this->enabled = false; }
    bool isEnabled() const { return enabled; }

protected:
    CameraInput camera_input;

    cv::CascadeClassifier cascade;
    uint16_t x;
    uint16_t y;

    bool object_found = false;
    bool error_found = false;
    std::string error_type = nullptr;
    bool enabled = true;
};

