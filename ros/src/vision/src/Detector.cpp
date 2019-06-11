#include "opencv2/opencv.hpp"
#include "CameraInput.cpp"
#include <cmath>

class Detector
{
public:
    virtual bool update() = 0;

    uint16_t getX() const { return x; }
    uint16_t getY() const { return y; }
    bool errorFound() const { return error_found; }
    std::string getError() const { return error_type; }
    void enable() { this->enabled = true; }
    void disable() { this->enabled = false; }
    bool isEnabled() const { return enabled; }

protected:
    Detector();
    ~Detector() = default;
    void filter();

    cv::CascadeClassifier cascade;
    uint16_t x;
    uint16_t y;

    bool error_found;
    std::string error_type;
    bool enabled;

// Everything below, exept for some of the constructor init's,
// is used to implement a false positive filter.
private:
    struct point_data {
        cv::Point p;
        uint8_t n;
    };

    std::vector<point_data> points;
    uint8_t iteration;
    const uint16_t radius;
    const uint8_t reset_time;
    
    void updateBestPoint();
    void addPoint(cv::Point point);
    void removePoint();
};

Detector::Detector() : radius(100), reset_time(8)
{
    enable = true;
    error_found = false;
    error_type = nullptr;

    iteration = 0;
}

/**
 * Must be called in update() every iteration to work properly.
 * 
 * Takes in the location data of every object found, filters out false positives,
 * then sets x and y to the best location found. 
 */
void Detector::filter(std::vector<cv::Rect> locations)
{
    for(uint8_t i=0; i<locations.size(); i++)
    {
        cv::Point curr_point(locations[i].x + locations[i].width/2, locations[i].y + locations[i].height/2);
        addPoint(curr_point);
        updateBestPoint();
    }

    iteration++;
    if(iteration >= reset_time) {
        removePoint();
        iteration = 0;
    }
}

/**
 * Iterates through points to find the point with the largest n. If n is
 * not greater than 3, x and y are set to 0.
 */
void Detector::updateBestPoint()
{
    uint8_t max = 0;
    uint8_t max_index = 0;
    for(uint8_t i=0; i<points.size(); i++)
    {
        if(points.at(i).n > max) {
            max = points.at(i).n;
            max_index = i;
        }
    }

    if(max > 3) {
        x = points.at(max_index).p.x;
        y = points.at(max_index).p.y;
    } 
    else {
        x = 0;
        y = 0;
    }
}

/**
 * Iterates through points, adding one to n if the current point is within
 * the set radius. P is then updated to the current point.
 * 
 * If the current point is not within the radius of any point in points,
 * a new index is created for it.  
 */
void Detector::addPoint(cv::Point point)
{
    for(uint8_t i=0; i<points.size(); i++)
    {
        if(sqrt(pow(point.x - points.at(i).p.x, 2) + pow(point.y - points.at(i).p.y, 2)) <= radius) {
            points.at(i).p.x = point.x;
            points.at(i).p.y = point.y;
            points.at(i).n = (points.at(i).n < 8) ? points.at(i).n + 1 : 8;
            return;
        }
    }
    points.push_back({point, 1});
}

/**
 * Iterates through points and removes one from every n in point_data.
 * If n == 0 in any point_data, remove the data from the vector points. 
 */
void Detector::removePoint()
{
    for(uint8_t i=0; i<points.size(); i++)
    {
        points.at(i).n = points.at(i).n - 1;
        if(points.at(i).n == 0) {
            points.erase(points.begin() + i);
            i--;
        }
    }
}