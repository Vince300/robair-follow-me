#ifndef _OBSTACLESBASICTRACKER_HPP_
#define _OBSTACLESBASICTRACKER_HPP_

#include "SlidingWindowTracker.hpp"

/**
 * \brief Work in progress, user tracker with obstacle avoidance.
 */
class ObstaclesBasicTracker : public SlidingWindowTracker {
public:
    /**
     * \brief Initializes a new instance of the \see ObstaclesBasicTracker class.
     */
    ObstaclesBasicTracker();
    /**
     * \brief Frees the resources used by this instance.
     */
    ~ObstaclesBasicTracker();

protected:
    /// Updates the target point of interest.
    bool doUpdateTarget() override;

    /// Angle of the robot's head.
    double headAngle;
};

#endif
