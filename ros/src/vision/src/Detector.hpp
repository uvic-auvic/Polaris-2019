#include "opencv2/opencv.hpp"
#include "CameraInput.cpp"

class Detector
{
public:
    Detector();
    Detector(std::string cascade_name);
    ~Detector() = default;

    virtual bool update() = 0;
    virtual bool error_found() = 0;
    virtual std::string get_error() = 0;

    void enable() { this->enabled = true; }
    void disable() { this->enabled = false; }
    bool is_enabled() const { return enabled; }

protected:
    bool enabled;
};