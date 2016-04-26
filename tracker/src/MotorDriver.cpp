#include <ros/ros.h>
#include <std_msgs/String.h>
#include <md49test/MotorCmd.h>

#include "MotorDriver.hpp"

MotorDriver::MotorDriver()
    : workerThread(),
      targetTracker(nullptr),
      doExit(false)
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
    ros::NodeHandle npub;
    
    /* Cette commande crée un topic du nom cmdmotors et va publier un 
       message de type md49test::MotorCmd dessus, 1000 messages dans 
       le buffer */
    ros::Publisher chatter_pub = npub.advertise<md49test::MotorCmd>("cmdmotors", 1000);

    /*Nombre de messages par seconde*/
    ros::Rate loop_rate(10);

    while (ros::ok() && !doExit)
    {
        /* Obtention de l'objet tracker */
        std::shared_ptr<TargetTracker> tracker = targetTracker;

        /* Crée un objet md49test::MotorCmd et remplit les attributs */
        md49test::MotorCmd msg;

        double speed, angle;
        if (tracker)
        {
            double angle, speed;
            tracker->getCommandData(speed, angle);

            int16_t scale = 64;

            msg.speed1 = speed * scale;
            msg.speed2 = speed * scale;
        }
        else
        {
            msg.speed1 = 0;
            msg.speed2 = 0;
        }

        /* Publie le message sur le topic */
        chatter_pub.publish(msg);

        /* Loop updates */
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();
}
