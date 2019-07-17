#include "Detector.hpp"
#include "GateDetector.hpp"


//Update to return Detector Result Struct
cv::Rect findGate(cv::Mat frame){
    cv::Rect Gate;
    cv::Mat frame_gray;
    cv::CascadeClassifier object_cascade; 
    object_cascade.load("gate_cascade.xml");//Insert Cascade Classifier Here

    cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(frame_gray, frame_gray);
    // Detects objects and stores their location in object
    std::vector<cv::Rect> object;
    object_cascade.detectMultiScale(frame_gray, object, 1.1, 2, 0|cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

    //if object is found in frame
    if(!object.empty()) {
        Gate = object[0];  
    }
    return Gate;
}















