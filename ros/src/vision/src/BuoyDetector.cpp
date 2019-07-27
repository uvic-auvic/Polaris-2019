#include "BuoyDetector.hpp"
#include "Distance.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <stdio.h>

BuoyDetector::BuoyDetector(CameraInput& input, std::string cascade_name) : camera_input(input)
{
    cascade.load(cascade_name);
}

BuoyDetector::~BuoyDetector(){}


bool BuoyDetector::update()
{
    found_buoy = false;
    cv::Mat gray_frame;

    // Convert input.getFrameFront() to black and white
    cv::cvtColor(camera_input.getFrameFront(), gray_frame, cv::COLOR_BGR2GRAY);

    
    detector.sift->detectAndCompute(gray_frame, cv::noArray(), detector.keypoints2, detector.descriptors2);
    // matching the descriptor vectors with a FLANN based matcher
    std::vector< std::vector<cv::DMatch> > knn_matches;
    detector.matcher->knnMatch( detector.descriptors1, detector.descriptors2, knn_matches, 2);

    // Filter matches using the Lowe's ratio test
    std::vector<cv::DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++){
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance){
            good_matches.push_back(knn_matches[i][0]);
        }
    }
    cv::Mat img_matches;
    if( good_matches.size() >= min_match_count){
        std::vector<cv::Point2f> src_pts;
        std::vector<cv::Point2f> dst_pts;

        for( cv::DMatch x : good_matches){
            src_pts.push_back(detector.keypoints1[x.queryIdx].pt);
            dst_pts.push_back(detector.keypoints2[x.trainIdx].pt);
        }
        cv::Mat H = cv::findHomography(src_pts, dst_pts, cv::RANSAC);

        // get the corners from the jiangshi image
        std::vector<cv::Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( detector.buoy_img.cols, 0 );
        obj_corners[2] = cvPoint(detector.buoy_img.cols, detector.buoy_img.rows); 
        obj_corners[3] = cvPoint(0, detector.buoy_img.rows);
        std::vector<cv::Point2f> scene_corners(4);
        
        if ( !H.empty() ){
            cv::perspectiveTransform( obj_corners, scene_corners, H);
            cv::Rect rect(scene_corners[0], scene_corners[2]);
            buoy_rect = rect;
            found_buoy = true;
            
            distance_x_front = Distance::getDistanceX(buoy_rect, buoy_width, camera_input.getFrameFront());
            distance_y_front = Distance::getDistanceY(buoy_rect, buoy_height, camera_input.getFrameFront());
            distance_z_front = Distance::getDistanceZ(buoy_rect, buoy_width, 1);
        }
    }
    return found_buoy;
}
