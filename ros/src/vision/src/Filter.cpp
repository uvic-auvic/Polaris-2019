#include "Filter.hpp"

/**
 * Must be called in every iteration to work properly.
 * 
 * Takes in the location data of every object found, filters out false positives,
 * then sets x and y to the best location found. 
 */
void Filter::updateFilter(std::vector<cv::Rect> locations)
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
void Filter::updateBestPoint()
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
void Filter::addPoint(cv::Point point)
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
void Filter::removePoint()
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