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

cv::Point findGateDivider(cv::Mat frame){
    cv::Mat img1 = frame;
    cv::Mat img2 = cv::imread("img_matches/gate_divide.PNG");

    int minHessian = 400;
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create( minHessian );
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;
    detector->detectAndCompute( img1, cv::noArray(), keypoints1, descriptors1 );
    detector->detectAndCompute( img2, cv::noArray(), keypoints2, descriptors2 );
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<cv::DMatch> > knn_matches;
    matcher->knnMatch( descriptors1, descriptors2, knn_matches, 2 );
    //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.7f;
    std::vector<cv::DMatch> good_matches;
    std::vector<cv::Point2f> feature_points;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
            feature_points.push_back(keypoints2.at(knn_matches[i][0].queryIdx).pt);
        }
    }
    return avgPoint(feature_points);
}

cv::Point avgPoint(std::vector<cv::Point2f> list){
    int x = 0;
    int y = 0;
    int points = list.size();
    for (int i = 0; i < points; i++){
        x += list.at(i).x;
        y += list.at(i).y;
    }
    int x_avg = x/points;
    int y_avg = y/points;
    cv::Point avg = cv::Point(x_avg,y_avg);
    return avg;
    }














