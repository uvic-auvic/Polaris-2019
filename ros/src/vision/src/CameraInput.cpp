#include"opencv2/opencv.hpp"

class CameraInput
{
public:
    CameraInput();
    ~CameraInput();

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
{
    input_front.read(frame_front);
    input_bottom.read(frame_bottom);
    input_top.read(frame_top);
}

CameraInput::~CameraInput()
{}

bool CameraInput::update()
{
    if(!input_front.read(frame_front)) {
        // throw error
    }
    if(!input_bottom.read(frame_bottom)) {
        // throw error
    }
    if(!input_top.read(frame_top)) {
        // throw error
    }
}

cv::Mat CameraInput::getFrameFront() { return frame_front; }

cv::Mat CameraInput::getFrameBottom() { return frame_bottom; }

cv::Mat CameraInput::getFrameTop() { return frame_top; }