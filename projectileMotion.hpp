#ifndef BOXPOINT
#define BOXPOINT

#include <opencv4/opencv2/opencv.hpp> 
#include <vector>

struct BoxPoint{
    double x;
    double yHeight;
    double yWidth;
    double z;
    BoxPoint(double x, double yHeight, double yWidth, double z);

};

std::ostream& operator<<(std::ostream& os, const BoxPoint& bp);

std::vector<BoxPoint> getCenters(std::vector<std::vector<cv::Point>> contours);


BoxPoint projectileDisplacement(double t, BoxPoint initial, BoxPoint velocities, double g=0);


double calculateGravity(std::vector<BoxPoint> points);



#endif
