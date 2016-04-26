#ifndef _PIDCONTROLLER_HPP_
#define _PIDCONTROLLER_HPP_

/**
 * \brief Implements a proportional-summation-difference (a discrete-time 
 *        variant of a PID controller).
 */
class PIDController {
public:
    /**
     * \brief Initializes a new instance of the \see PIDController class.
     * 
     * \param kP Proportional constant.
     * \param kI Integral constant.
     * \param kD Derivative constant.
     */
    PIDController(double kP, double kI, double kD);

    /**
     * \brief Computes the next time step of this controller.
     *
     * \return Current filtered output.
     */
    double timeStep();

    /**
     * \brief Sets the target value the controller must reach.
     *
     * \param targetValue New target value for the controller.
     */
    void setTargetValue(double targetValue);

private:
    /// Proportional constant.
    double kP;
    /// Integral constant.
    double kI;
    /// Derivative constant.
    double kD;

    /// Current value of the filter.
    double currentValue;
    /// Target value of the filter.
    double target;

    /// Current error integral value.
    double integral;
    /// Previous time-step error value.
    double prevError;
};

#endif
