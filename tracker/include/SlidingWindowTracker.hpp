#ifndef _SLIDINGWINDOWTRACKER_HPP_
#define _SLIDINGWINDOWTRACKER_HPP_

#include "TargetTracker.hpp"
#include "FeatureCapture.hpp"

class SlidingWindowTracker : public TargetTracker {
public:
    SlidingWindowTracker();
    ~SlidingWindowTracker();

    void updateDisplayFrame(cv::Mat &displayFrame) override;

protected:
    void doUpdateRegionOfInterest(int x, int y) override;
    bool doUpdateTarget() override;

    void doUpdateRegionOfInterest(int x, int y, cv::Scalar fallbackValue);

    // User tracking data
    FeatureCapture tracking;
};

#endif
