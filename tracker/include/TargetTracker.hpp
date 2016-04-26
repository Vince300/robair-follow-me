#ifndef _TARGETTRACKER_HPP_
#define _TARGETTRACKER_HPP_

#include <opencv2/core/mat.hpp>
#include <mutex>

/**
 * \brief Represents a generic target tracker.
 *        A target tracker uses the successive frames returned by the
 *        OpenNI devices to update the tracked target position and
 *        the command data used to drive the motors.
 */
class TargetTracker {
public:
    /**
     * \brief Updates the region of interest of this tracker, based on
     *        the picked location from the frames.
     *
     * \param x X coordinate of the main point of interest.
     * \param y Y coordinate of the main point of interest.
     */
    void updateRegionOfInterest(int x, int y);

    /**
     * \brief Updates the tracking state using the new frames captured
     *        from the camera
     *
     * \param depthFrame Reference to the raw depth data frame.
     * \param videoFrame Reference to the raw video data frame.
     */
    void updateTarget(const cv::Mat &depthFrame,
                      const cv::Mat &videoFrame);

    /**
     * \brief Updates the given displayFrame to represent this tracker
     *        tracking state.
     *
     * \param displayFrame CV image to update.
     */
    virtual void updateDisplayFrame(cv::Mat &displayFrame);

    /**
     * \brief Obtains the command data from this tracker.
     *        This method is synchronized with the setCommand called by
     *        the tracker to update the current order, so (speed, angle)
     *        is always coherent.
     * 
     * \param speed Reference to receive the speed value.
     * \param angle Reference to receive the angle value.
     */
    void getCommandData(double &speed, double &angle);

protected:
    /**
     * \brief Initializes a new instance of the \see TargetTracker class.
     */
    TargetTracker();
    /**
     * \brief Frees the resources used by this instance.
     */
    virtual ~TargetTracker();

    /**
     * \brief Performs the actual target update, when implemented by a
     *        derived class.
     */
    virtual bool doUpdateTarget() = 0;
    
    /**
     * \brief Performs the actual region of interest update, when implemented by a
     *        derived class.
     */
    virtual void doUpdateRegionOfInterest(int x, int y) = 0;
    
    /**
     * \brief Gets the current command speed value.
     */
    double getCommandSpeed() const { return commandSpeed; }
    /**
     * \brief Gets the current command angle value.
     */
    double getCommandAngle() const { return commandAngle; }
    
    /**
     * \brief Sets the command values (synchronized call with \see getCommandData)
     *
     * \param speed Robot speed, between -1.0 and 1.0.
     * \param angle Robot angle, between -1.0 and 1.0.
     */
    void setCommand(double speed, double angle);

    /// Last depth frame used in an update.
    cv::Mat lastDepthFrame;
    /// Last video frame used in an update.
    cv::Mat lastVideoFrame;

private:
    /// Mutex to synchronize the access to the command speed and angle.
    std::mutex commandMtx;
    /// The speed to drive the engines, between -1.0 and 1.0.
    double commandSpeed;
    /// The angle relative to the current heading, in radians.
    double commandAngle;
};

#endif
