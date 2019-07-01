#include "opencv2/opencv.hpp"
#include "CameraInput.hpp"

class Detector
{
public:
    virtual bool update() = 0;

    uint16_t getXF() const { return xf; }
    uint16_t getYF() const { return yf; }
    uint16_t getZF() const { return zf; }
    uint16_t getXB() const { return xb; }
    uint16_t getYB() const { return yb; }
    uint16_t getZB() const { return zb; }
    uint16_t getXT() const { return xt; }
    uint16_t getYT() const { return yt; }
    uint16_t getZT() const { return zt; }
    bool objectFound() const { return object_found; }
    bool errorFound() const { return error_found; }
    std::string getError() const { return error_type; }
    void enable() { this->enabled = true; }
    void disable() { this->enabled = false; }
    bool isEnabled() const { return enabled; }

protected:

    cv::CascadeClassifier cascade;
    
    uint16_t xf = 0;
    uint16_t yf = 0;
    uint16_t zf = 0;
    uint16_t xb = 0;
    uint16_t yb = 0;
    uint16_t zb = 0;
    uint16_t xt = 0;
    uint16_t yt = 0;
    uint16_t zt = 0;

    bool object_found = false;
    bool error_found = false;
    std::string error_type;
    bool enabled = true;
};

