#ifndef _OBSTACLESBASICTRACKER_HPP_
#define _OBSTACLESBASICTRACKER_HPP_

#include "SlidingWindowTracker.hpp"

class ObstaclesBasicTracker : public SlidingWindowTracker {
public:
    ObstaclesBasicTracker();
    ~ObstaclesBasicTracker();

protected:
    bool doUpdateTarget() override;

    double headAngle;
};

#endif
