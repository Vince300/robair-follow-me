#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <md49test/MotorCmd.h>

#include "MotorDriver.hpp"

MotorDriver::MotorDriver()
    : workerThread(&MotorDriver::threadCallback, this),
    targetTracker(nullptr),
    doExit(false),
    /* PID initialization, kP, kI, kD */
    speedController(0.6, 0.15, 0.05),
    angleController(0.6, 0.01, 0.05),
    pauseSuivi(true)
{
}

void MotorDriver::activationCallback(const std_msgs::Bool::ConstPtr& msg)
{
    pauseSuivi = !pauseSuivi;
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
    ros::NodeHandle npub;

    /* Cette commande crée un topic du nom cmdmotors et va publier un
       message de type md49test::MotorCmd dessus, 1000 messages dans
       le buffer */
    ros::Publisher chatter_pub = npub.advertise<md49test::MotorCmd>("cmdmotors", 1000);
    ros::Subscriber listenerActivationSuivi = npub.subscribe("start_suivi", 2, activationCallback);


    ros::Rate loop_rate(10);

    while (ros::ok() && !doExit)
    {
        /* Obtention de l'objet tracker */
        std::shared_ptr<TargetTracker> tracker = targetTracker;

        /* Crée un objet md49test::MotorCmd et remplit les attributs */
        md49test::MotorCmd msg;

        double angle = 0.0, speed = 0.0;

        int16_t scale = 127;
        int16_t scaleAngle = scale / 3;
        int16_t offset = 0;
        if (tracker)
        {
            // Smooth stopping of robert by using PIDs even if tracking
            // is not enabled.
            if (!pauseSuivi)
                tracker->getCommandData(speed, angle);
            
            speedController.setTargetValue(speed);
            angleController.setTargetValue(angle);

            angle = angleController.timeStep();

            if (std::abs(speed) < 0.01) {
                msg.speed1 = angle * scaleAngle;
                msg.speed2 = -angle * scaleAngle;
            } else {
                speed = speedController.timeStep();
                msg.speed1 = speed * ((1.0 + angle) / 2.0 * scale);
                msg.speed2 = speed * ((1.0 - angle) / 2.0 * scale);
            }
        }
        else
        {
            msg.speed1 = 0;
            msg.speed2 = 0;
        }
        
        std::cerr << "Command speed = " << speed << " angle = " << angle 
                  << " message speed1 = " << msg.speed1 << " speed2 = " << msg.speed2
                  << std::endl;

        // Clamp values
        if (msg.speed1 < -scale) msg.speed1 = -scale;
        if (msg.speed1 > scale) msg.speed1 = scale;
        if (msg.speed2 < -scale) msg.speed2 = -scale;
        if (msg.speed2 > scale) msg.speed2 = scale;

        /* Publie le message sur le topic */
        chatter_pub.publish(msg);

        /* Loop updates */
        ros::spinOnce();
        loop_rate.sleep();
    }
}
