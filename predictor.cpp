#include "predictor.hpp"
#include "projectileMotion.hpp"
#include <cmath>


BoxPoint getVelocity(BoxPoint first, BoxPoint second, int t){
    // returns average velocity from second to first in t frames
    return BoxPoint{
        (second.x-first.x)/t, 
        (second.yWidth-first.yWidth)/t,
        (second.yHeight-first.yHeight)/t,
        (second.z-first.z)/t
    };
}

std::vector<BoxPoint> getVelocities(std::vector<BoxPoint> points){
    // returns vector of velocities from points 
    // returned vector has size one less than points
    std::vector<BoxPoint> velocities;
    velocities.reserve(points.size()-1);

    for(size_t i=0; i<points.size()-1; i++){
        BoxPoint cur = points.at(i);
        BoxPoint next = points.at(i+1);
        
        BoxPoint velocity = getVelocity(cur, next, 1);

        velocities.emplace_back(velocity);
    }
    return velocities;
}


std::vector<BoxPoint> predictTrajectory(BoxPoint initial, BoxPoint velocity, int startingOffset, double tMax, double tIncrement, double g){
    // for each increment of time up to tMax, finds the predicted location of the projectile
    std::vector<BoxPoint> trajectory;
    trajectory.reserve(std::ceil(startingOffset/tIncrement));
    for(double t=startingOffset+(2*tIncrement); t<startingOffset+tMax; t+=tIncrement){
        BoxPoint location = projectileDisplacement(t, initial, velocity, g);
        trajectory.emplace_back(location); 
    } 
    return trajectory;
}


std::vector<BoxPoint> getPredictedTrajectory(std::vector<BoxPoint> points, double tMax, double tIncrement){
    // calculates expected trajectory by finding the average trajectory using every pair of adjacent points
    double g = calculateGravity(points);
    std::vector<BoxPoint> velocities = getVelocities(points);
    std::vector<std::vector<BoxPoint>> trajectories;
    trajectories.reserve(velocities.size());

    for(size_t i=0; i<velocities.size(); i++){
        BoxPoint velocity = velocities.at(i);
        BoxPoint initial = points.at(i);
        int predictionOffset = velocities.size() - i; 

        std::vector<BoxPoint> calculatedTrajectory = predictTrajectory(initial, velocity, predictionOffset, tMax, tIncrement, g);
        trajectories.emplace_back(calculatedTrajectory);
    }
    
    std::vector<BoxPoint> avgTrajectory; 
    if(trajectories.size() > 0){
        int numPoints = trajectories.at(0).size();
        int numTrajectories = trajectories.size();
        avgTrajectory.reserve(numPoints);
        for(size_t i=0; i<numPoints; i++){
            double xTot = 0;
            /* double yWTot = 0; */
            /* double yHTot = 0; */
            double zTot = 0;
            std::vector<double> yWs;
            std::vector<double> yHs;
            for(auto& traj : trajectories){
                BoxPoint point = traj.at(i);
                xTot += point.x;
                /* yWTot += point.yWidth; */
                /* yHTot += point.yHeight; */
                zTot += point.z;
                yWs.emplace_back(point.yWidth);
                yHs.emplace_back(point.yHeight);
            }

            std::sort(yWs.begin(), yWs.end());
            std::sort(yHs.begin(), yHs.end());
            double medyW, medyH;
            int medIdx = yWs.size() / 2;
            if(yWs.size() % 2 == 0){
                medyW = (yWs.at(medIdx) + yWs.at(medIdx)-1)/2;
                medyH = (yHs.at(medIdx) + yHs.at(medIdx)-1)/2;
            }else{
                medyW = yWs.at(medIdx);
                medyH = yHs.at(medIdx);
            }

            BoxPoint avgPoint = BoxPoint{
                xTot/numTrajectories,
                /* yWTot/numTrajectories, */
                /* yHTot/numTrajectories, */
                medyW,
                medyH,
                zTot/numTrajectories
            };
            avgTrajectory.emplace_back(avgPoint);
        }
    }
    return avgTrajectory;
}

