#include "opencv2/opencv.hpp"

class CameraInput
{
public:
    CameraInput();
    ~CameraInput() = default;

    bool update();

    cv::Mat getFrameFront();
    cv::Mat getFrameBottom();
    cv::Mat getFrameTop();

private:
    cv::Mat frame_front;
    cv::Mat frame_bottom;
    cv::Mat frame_top;

    cv::VideoCapture input_front;
    cv::VideoCapture input_bottom;
    cv::VideoCapture input_top;
};

CameraInput::CameraInput() : input_front(0), input_bottom(0), input_top(0)
{}

bool CameraInput::update()
{
    if(!input_front.read(frame_front)) {
        return false;
    }
    if(!input_bottom.read(frame_bottom)) {
        return false;
    }
    if(!input_top.read(frame_top)) {
        return false;
    }
    return true;
}

cv::Mat CameraInput::getFrameFront() { return frame_front; }

cv::Mat CameraInput::getFrameBottom() { return frame_bottom; }

cv::Mat CameraInput::getFrameTop() { return frame_top; }