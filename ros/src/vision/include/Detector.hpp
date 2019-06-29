#include "opencv2/opencv.hpp"
#include "CameraInput.hpp"

class Detector
{
public:
    virtual bool update() = 0;

    uint16_t getXF() const { return xf; }
    uint16_t getYF() const { return yf; }
    uint16_t getXB() const { return xb; }
    uint16_t getYB() const { return yb; }
    uint16_t getXT() const { return xt; }
    uint16_t getYT() const { return yt; }
    bool objectFound() const { return object_found; }
    bool errorFound() const { return error_found; }
    std::string getError() const { return error_type; }
    void enable() { this->enabled = true; }
    void disable() { this->enabled = false; }
    bool isEnabled() const { return enabled; }

protected:
    ~Detector() = default;

    cv::CascadeClassifier cascade;
    
    uint16_t xf;
    uint16_t yf;
    uint16_t xb;
    uint16_t yb;
    uint16_t xt;
    uint16_t yt;

    bool object_found = false;
    bool error_found = false;
    std::string error_type = nullptr;
    bool enabled = true;
};

