#ifndef _SLIDINGWINDOWTRACKER_HPP_
#define _SLIDINGWINDOWTRACKER_HPP_

#include "TargetTracker.hpp"
#include "FeatureCapture.hpp"

/**
 * \brief Implements a user tracking system using a sliding mean rectangular window.
 */
class SlidingWindowTracker : public TargetTracker {
public:
    /**
     * \brief Initializes a new instance of the \see SlidingWindowTracker class.
     */
    SlidingWindowTracker();
    /**
     * \brief Frees the resources used by this instance.
     */
    ~SlidingWindowTracker();

    /**
     * \brief Performs the transformations on the displayFrame to notify the user
     *        with the tracking data.
     *
     * \param displayFrame OpenCV image to modify.
     */
    void updateDisplayFrame(cv::Mat &displayFrame) override;

protected:
    /**
     * \brief Updates the region of interest of this tracker from the given coordinates.
     *
     * \param x Image point of interest X coordinate.
     * \param y Image point of interest Y coordinate.
     */
    void doUpdateRegionOfInterest(int x, int y) override;
    /**
     * \brief Updates the tracking target.
     *
     * \return true if the update was successful, false otherwise.
     */
    bool doUpdateTarget() override;
    /**
     * \brief Updates the region of interest using the fallbackValue.
     *
     * \param x
     * \param y
     * \param fallbackValue
     */
    void doUpdateRegionOfInterest(int x, int y, cv::Scalar fallbackValue);

    /// Tracking status.
    FeatureCapture tracking;
};

#endif
