#include "projectileMotion.hpp"
#include "predictor.hpp"
#include <opencv2/imgproc.hpp>


int main(int argc, char** argv){
    
    if(argc <= 1){
        std::cerr << "Requires 1 Argument" << std::endl;
    }else{
        std::string videoFile = argv[1];
    
        int minContourArea = 300;
        cv::VideoCapture vid = cv::VideoCapture(videoFile);
        cv::Mat firstFrame, frame, grayFrame, frameDelta, thresh, lastFrame;
        std::vector<std::vector<cv::Point>> contours;
        vid.read(frame);
        lastFrame = frame.clone();
        cv::cvtColor(frame, firstFrame, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(firstFrame, firstFrame, cv::Size(21, 21), 0);


        std::vector<BoxPoint> initialTrajectory;
        
        while(vid.read(frame)){
            lastFrame = frame.clone();
            cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY); 
            cv::GaussianBlur(grayFrame, grayFrame, cv::Size(21, 21), 0);
            cv::absdiff(firstFrame, grayFrame, frameDelta);
            cv::threshold(frameDelta, thresh, 25, 255, cv::THRESH_BINARY);
            cv::dilate(thresh, thresh, cv::Mat(), cv::Point(-1, -1), 2);
            cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
            std::vector<cv::Rect> boundingRect{contours.size()};
            std::vector<BoxPoint> centers = getCenters(contours);
            if(centers.size() > 0){
                initialTrajectory.emplace_back(centers.at(0));
            };
            
            for(size_t i=0; i<contours.size(); i++){
                if(cv::contourArea(contours.at(i)) < minContourArea){ continue; }
          
                cv::Rect rect = cv::boundingRect(contours.at(i));
                cv::rectangle(frame, rect, cv::Scalar(0, 0, 255), 1);

                cv::circle(frame, cv::Point{(int) centers.at(i).x, (int) centers.at(i).z}, 2, cv::Scalar(0, 0, 255));
            }
            
            cv::imshow("Camera", frame);
            cv::waitKey(0);
            if(cv::waitKey(1) == 27){
                break;
            }

        }

        std::vector<BoxPoint> predictedTrajectory = getPredictedTrajectory(initialTrajectory, 25.0, 0.50);
        for(size_t i=0; i<initialTrajectory.size()+predictedTrajectory.size(); i++){
            BoxPoint point = i < initialTrajectory.size() ? initialTrajectory.at(i) : predictedTrajectory.at(i-initialTrajectory.size());
            cv::Scalar color = i < initialTrajectory.size() ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
                    
            cv::Point centerPoint{(int) point.x, -1 * (int) point.z};
            cv::Size boxSize = cv::Size{(int) point.yHeight, (int) point.yWidth};
            cv::Point boxStart{
                centerPoint.x - (int)(0.5*boxSize.width),
                centerPoint.y - (int)(0.5*boxSize.height)};
            cv::Rect box{boxStart, boxSize}; 

            cv::circle(lastFrame, centerPoint, 2, color);
            cv::rectangle(lastFrame, box, color);
        };
        cv::imshow("Predicted", lastFrame);
        cv::waitKey(0); 
        cv::imwrite("output.jpg", lastFrame);
    }
}

