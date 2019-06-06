#include"opencv2/opencv.hpp"

class Detector
{
public:
    Detector() = delete;
    Detector(std::string cascade_name);
    ~Detector() = default;

    virtual bool update() = 0;
    virtual uint16_t get_x() = 0;
    virtual uint16_t get_y() = 0;
    virtual bool error_found() = 0;
    virtual std::string get_error() = 0;
    void enable() { this->enabled = true; }
    void disable() { this->enabled = false; }
    bool is_enabled() const { return enabled; }

protected:
    cv::Mat frame;
    bool enabled;
    const uint8_t camera_id_1 = 0;
    const uint8_t camera_id_2 = 0;
    const uint8_t camera_id_3 = 0;
};