#ifndef _MOTORDRIVER_HPP_
#define _MOTORDRIVER_HPP_

#include <thread>
#include "TargetTracker.hpp"
#include "PIDController.hpp"

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
	//Callback called by the start_suivi from the web interface
	void activationCallback(const std_msgs::Bool::ConstPtr& msg)
    /// ROS thread callback.
    void threadCallback();
    /// ROS thread.
    std::thread workerThread;
    /// Current user tracker.
    std::shared_ptr<TargetTracker> targetTracker;
    /// Exit flag for the worker thread.
    bool doExit;
    /// PID Controller for the speed value
    PIDController speedController;
    /// PID Controller for the angle value
    PIDController angleController;
	/// should robair stop following orders from the tracker
	bool pauseSuivi;
};

#endif
