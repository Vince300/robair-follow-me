#ifndef _PIDCONTROLLER_HPP_
#define _PIDCONTROLLER_HPP_

class PIDController {
public:
    PIDController(double kP, double kI, double kD);
    ~PIDController();

    double timeStep();
    void setTargetValue(double targetValue);

private:
    double kP;
    double kI;
    double kD;

    double currentValue;
    double target;

    double integral;
    double prevError;
};

#endif
