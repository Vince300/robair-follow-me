#ifndef _MOTORDRIVER_HPP_
#define _MOTORDRIVER_HPP_

#include <thread>
#include "TargetTracker.hpp"

/**
 * \brief Represents the motor driving interface and worker thread.
 */
class MotorDriver {
public:
    /**
     * \brief Initializes a new instance of the \see MotorDriver class.
     */
    MotorDriver();

    /**
     * \brief Stops the motor driver and exits the worker thread.
     */
    void stop();

    /**
     * \brief Sets the user tracker computing the motor drive values for this instance.
     *
     * \param value User tracker instance to use to drive the motors.
     */
    void setTargetTracker(const std::shared_ptr<TargetTracker> &value);

private:
    /// ROS thread callback.
    void threadCallback();
    /// ROS thread.
    std::thread workerThread;
    /// Current user tracker.
    std::shared_ptr<TargetTracker> targetTracker;
    /// Exit flag for the worker thread.
    bool doExit;
};

#endif
