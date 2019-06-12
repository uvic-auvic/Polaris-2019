#include "Detector.hpp"
#include "Filter.cpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"



//-------------------------------------------------------------------------------------------------
//
// THIS IS A TEMPLATE
//
//-------------------------------------------------------------------------------------------------

class DerivedDetector : public Detector, Filter
{
public:
    DerivedDetector::DerivedDetector(std::string cascade_name);
    bool update();
    // add functions as needed

private:
    // add functions/variables as needed
};

DerivedDetector::DerivedDetector(std::string cascade_name)
{
    cascade.load(cascade_name);
}

bool DerivedDetector::update()
{
    cv::Mat frame_gray;
    cv::cvtColor(camera_input.getFrameFront(), frame_gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(frame_gray, frame_gray);

    // Detects objects and stores their location in locations
    std::vector<cv::Rect> locations;
    cascade.detectMultiScale(frame_gray, locations, 1.1, 2, 0|cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
    if(locations.empty()) return false;

    updateFilter(locations);
    x = getBestX();
    
    return true;
}