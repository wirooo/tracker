#include "projectileMotion.hpp"


BoxPoint getVelocity(BoxPoint first, BoxPoint second, int t = 1);


std::vector<BoxPoint> getVelocities(std::vector<BoxPoint> points);


std::vector<BoxPoint> predictTrajectory(BoxPoint initial, BoxPoint velocity, int startingOffset, double tMax, double tIncrement, double g);


std::vector<BoxPoint> getPredictedTrajectory(std::vector<BoxPoint> points, double tMax, double tIncrement);

    
