#include "TargetTracker.hpp"

void TargetTracker::updateRegionOfInterest(int x, int y)
{
    doUpdateRegionOfInterest(x, y);
}

void TargetTracker::updateTarget(const cv::Mat& depthFrame, const cv::Mat& videoFrame)
{
    lastDepthFrame = depthFrame;
    lastVideoFrame = videoFrame;

    doUpdateTarget();
}

void TargetTracker::updateDisplayFrame(cv::Mat& displayFrame)
{
    // nothing to do by default
}

void TargetTracker::getCommandData(double& speed, double& angle)
{
    std::lock_guard<std::mutex> lock(commandMtx);
    speed = commandSpeed;
    angle = commandAngle;
}

TargetTracker::TargetTracker() :
    lastDepthFrame(), 
    lastVideoFrame(), 
    commandMtx(),
    commandSpeed(0),
    commandAngle(0)
{
}

TargetTracker::~TargetTracker()
{
}

void TargetTracker::setCommand(double speed, double angle)
{
    std::lock_guard<std::mutex> lock(commandMtx);
    commandSpeed = speed;
    commandAngle = angle;
}
