#ifndef _MOTORDRIVER_HPP_
#define _MOTORDRIVER_HPP_

#include <thread>
#include "TargetTracker.hpp"

class MotorDriver {
public:
    MotorDriver();

    void stop();
    void setTargetTracker(const std::shared_ptr<TargetTracker> &value);

private:
    void threadCallback();

    std::thread workerThread;
    std::shared_ptr<TargetTracker> targetTracker;

    bool doExit;
};

#endif
