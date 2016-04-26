#include <iostream>

#include "MotorDriver.hpp"

MotorDriver::MotorDriver()
    : workerThread(&MotorDriver::threadCallback, this),
    targetTracker(nullptr),
    doExit(false),
    /* PID initialization, kP, kI, kD */
    speedController(0.6, 0.15, 0.05),
    angleController(0.6, 0.01, 0.05)
{
}

void MotorDriver::stop()
{
    doExit = true;
    workerThread.join();
}

void MotorDriver::setTargetTracker(const std::shared_ptr<TargetTracker> &value)
{
    targetTracker = value;
}

void MotorDriver::threadCallback()
{
    while (!doExit)
    {
        /* Obtention de l'objet tracker */
        std::shared_ptr<TargetTracker> tracker = targetTracker;

        int16_t speed1, speed2;

        double angle = 0.0, speed = 0.0;
        int16_t scale = 127;
        int16_t scaleAngle = scale / 3;
        if (tracker)
        {
            tracker->getCommandData(speed, angle);
            
            speedController.setTargetValue(speed);
            angleController.setTargetValue(angle);

            angle = angleController.timeStep();

            if (std::abs(speed) < 0.01) {
                speed1 = angle * scaleAngle;
                speed2 = -angle * scaleAngle;
            } else {
                speed = speedController.timeStep();
                speed1 = speed * ((1.0 + angle) / 2.0 * scale);
                speed2 = speed * ((1.0 - angle) / 2.0 * scale);
            }
        }
        else
        {
            speed1 = 0;
            speed2 = 0;
        }
        
        std::cerr << "Command speed = " << speed << " angle = " << angle 
                  << " message speed1 = " << speed1 << " speed2 = " << speed2
                  << std::endl;
    }
}
