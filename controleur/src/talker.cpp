#include "ros/ros.h"
/*md49test est un package qui contient un fichier msg/MotorCmd.msg, 
Il faut rajouter les dépendances dans package.xml et CMakeLists.txt 
Elles permettent la création d'objet c++/python de ce type de message.*/
#include "md49test/MotorCmd.h"

#include <sstream>

int main(int argc, char **argv)
{
/*Cette commande crée un noeud du nom de talker dans la package dans lequel il se trouve ( pour run , rosrun suivi talker )*/
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
/*Cette commande crée un topic du nom cmdmotors et va publier un message de type md49test::MotorCmd dessus, 1000 messages dans le buffer*/
  ros::Publisher chatter_pub = n.advertise<md49test::MotorCmd>("cmdmotors", 1000);
/*Nombre de messages par seconde*/
  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok())
  {
    /*crée un objet md49test::MotorCmd et rempli ces attributs*/
    md49test::MotorCmd msg;
    msg.speed1 = 5;
    msg.speed1 = -5;
    

    /*Publie le message sur le topic*/
    chatter_pub.publish(msg);
    ROS_INFO("message envoyé");

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
