#include <iostream>
#include <numeric>
#include "projectileMotion.hpp"


BoxPoint::BoxPoint(double x, double yHeight, double yWidth, double z):
    x{x},
    yHeight{yHeight},
    yWidth{yWidth},
    z{z} {}

std::ostream& operator<<(std::ostream& os, const BoxPoint& bp){
    os << bp.x << " " << bp.yWidth << " " << bp.yHeight << " " << bp.z;
    return os;
} 

std::vector<BoxPoint> getCenters(std::vector<std::vector<cv::Point>> contours){
    // returns centers of contours as points in 3D space
    // y coordinate becomes z, depth (in/out of photo) is y coordinate
    // y calculated as the average 
    std::vector<BoxPoint> boxPoints;
    boxPoints.reserve(contours.size());
    
    for(auto& contour : contours){
        cv::Moments moment = cv::moments(contour);
        cv::Point2d center{moment.m10/moment.m00, moment.m01/moment.m00};
        
        cv::Rect boundingBox = cv::boundingRect(contour);

        boxPoints.emplace_back(
                BoxPoint{
                    center.x,
                    (double) boundingBox.width,
                    (double) boundingBox.height,
                    -1 * center.y});
    }
    return boxPoints;
}

BoxPoint projectileDisplacement(double t, BoxPoint initial, BoxPoint velocities, double g){
    // calculates the position of the BoxPoint after t frames
    return BoxPoint(
            initial.x + (velocities.x * t),
            initial.yWidth + (velocities.yWidth * t),
            initial.yHeight + (velocities.yHeight * t),
            initial.z + (velocities.z * t) + (0.5 * g * t * t));
}

double calculateGravity(std::vector<BoxPoint> points){
    // estimates acceleration of gravityusing the trajectory of the given points
    std::vector<double> gravities;
    gravities.reserve(points.size() - 2);
    for(size_t i=0; i<points.size()-2; i++){
        BoxPoint cur = points.at(i);
        BoxPoint next = points.at(i+1);
        BoxPoint last = points.at(i+2);

        // find v2, v1, g = v2-v1/deltat, deltat=1
        double v1 = next.z - cur.z;
        double v2 = last.z - next.z;
        gravities.emplace_back(v2-v1);

    }
    return std::accumulate(gravities.begin(), gravities.end(), 0.0)/gravities.size();
}

