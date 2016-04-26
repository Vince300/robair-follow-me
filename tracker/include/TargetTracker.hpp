#ifndef _TARGETTRACKER_HPP_
#define _TARGETTRACKER_HPP_

#include <opencv2/core/mat.hpp>
#include <mutex>

class TargetTracker {
public:
    void updateRegionOfInterest(int x, int y);

    void updateTarget(const cv::Mat &depthFrame,
                      const cv::Mat &videoFrame);

    virtual void updateDisplayFrame(cv::Mat &displayFrame);

    void getCommandData(double &speed, double &angle);

protected:
    TargetTracker();
    virtual ~TargetTracker();

    virtual bool doUpdateTarget() = 0;
    virtual void doUpdateRegionOfInterest(int x, int y) = 0;

    double getCommandSpeed() const { return commandSpeed; }
    double getCommandAngle() const { return commandAngle; }
    void setCommand(double speed, double angle);

    cv::Mat lastDepthFrame;
    cv::Mat lastVideoFrame;

private:
    // Mutex to synchronize the access to the command speed and angle
    std::mutex commandMtx;
    // The speed to drive the engines, between -1.0 and 1.0
    double commandSpeed;
    // The angle relative to the current heading, in radians
    double commandAngle;
};

#endif
