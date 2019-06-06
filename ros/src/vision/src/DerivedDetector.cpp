#include "Detector.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <cmath>

class DerivedDetector : public Detector
{
public:
    DerivedDetector(std::string cascade_name);
    // ~DerivedDetector();
    bool update();
    bool camera_detected();
    uint16_t get_x();
    uint16_t get_y();

private:
    struct point_data {
        cv::Point p;
        uint8_t n;
    };

    cv::Point find_best_point();
    void add_point(cv::Point point);
    void remove_point();


    cv::Point best_point;
    std::vector<point_data> points;
    cv::Mat frame;
    cv::VideoCapture src;
    cv::CascadeClassifier cascade;

    bool camera_found;
    const uint16_t radius;
    const uint8_t reset_time;
    uint8_t iteration;
};

DerivedDetector::DerivedDetector(std::string cascade_name) : radius(100), reset_time(6)
{
    // if(src.isOpened()) camera_found = true;
    // else camera_found = false;

    cascade.load(cascade_name);
    iteration = 0;
}

// DerivedDetector::~DerivedDetector() // TODO
// {}

bool DerivedDetector::update()
{
    if(!camera_found) return false;
    if(!src.read(frame)) return false;

    cv::Mat frame_gray;
    cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(frame_gray, frame_gray);

    // Detects objects and stores their location in locations
    std::vector<cv::Rect> locations;
    cascade.detectMultiScale(frame_gray, locations, 1.1, 2, 0|cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
    if(locations.empty()) return false;

    for(uint8_t i=0; i<locations.size(); i++)
    {
        cv::Point curr_point(locations[0].x + locations[0].width/2, locations[0].y + locations[0].height/2);
        add_point(curr_point);
        best_point = find_best_point();
    }

    iteration++;
    if(iteration >= reset_time) {
        remove_point();
        iteration = 0;
    }
    
    return true;
}

bool DerivedDetector::camera_detected() { return camera_found; }

uint16_t DerivedDetector::get_x() { return best_point.x; }

uint16_t DerivedDetector::get_y() { return best_point.y; }

/**
 * Iterates through points to find the point with the largest n. If n is
 * not greater than 3, the point (0,0) is returned.
 */
cv::Point DerivedDetector::find_best_point()
{
    uint8_t max = 0;
    uint8_t max_index = 0;
    for(int i=0; i<points.size(); i++)
    {
        if(points.at(i).n > max) {
            max = points.at(i).n;
            max_index = i;
        }
    }
    if(max > 1) return points.at(max_index).p;
    return {0,0};
}

/**
 * Iterates through points, adding one to n if the current point is within
 * radius distance. P is then updated to the current point.
 * 
 * If the current point is not within radius of any point in points,
 * a new index is created for it.  
 */
void DerivedDetector::add_point(cv::Point point)
{
    for(int i=0; i<points.size(); i++)
    {
        if(sqrt(pow(point.x - points.at(i).p.x, 2) + pow(point.y - points.at(i).p.y, 2)) <= radius) {
            points.at(i).p.x = point.x;
            points.at(i).p.y = point.y;
            points.at(i).n = (points.at(i).n < 6) ? points.at(i).n + 1 : 6;
            return;
        }
    }
    points.push_back({point, 1});
}

/**
 * Iterates through points and removes one from every n in point_data.
 * If n == 0 in any point_data, remove the data from the vector points. 
 */
void DerivedDetector::remove_point()
{
    for(int i=0; i<points.size(); i++)
    {
        points.at(i).n = points.at(i).n - 1;
        if(points.at(i).n == 0) {
            points.erase(points.begin() + i);
            i--;
        }
    }
}