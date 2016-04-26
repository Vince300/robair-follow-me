#include "PIDController.hpp"

PIDController::PIDController(double kP, double kI, double kD)
    : kP(kP),
    kI(kI),
    kD(kD), 
    target(0), 
    prevError(0)
{
}

double PIDController::timeStep()
{
    // compute e[n]
    double error = target - currentValue;
    // compute de[n]/dn
    double derivative = error - prevError;
    // update integral
    integral += error;
    // update previous error
    prevError = error;
    // update current value
    currentValue = kP * error + kI * integral + kD * derivative;
    // return new value
    return currentValue;
}

void PIDController::setTargetValue(double targetValue)
{
    target = targetValue;
}


