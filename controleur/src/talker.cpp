#include "ros/ros.h"
/*md49test est un package qui contient un fichier msg/MotorCmd.msg, 
Il faut rajouter les dépendances dans package.xml et CMakeLists.txt 
Elles permettent la création d'objet c++/python de ce type de message.*/
#include "md49test/MotorCmd.h"

#include <sstream>

void chatterCallback(const std_msgs::String::ConstPtr& msglis)
{
  ROS_INFO("I heard: [%s]", msglis->data.c_str());
}

int main(int argc, char **argv)
{
/*Cette commande crée un noeud du nom de talker dans la package dans lequel il se trouve ( pour run , rosrun suivi talker )*/
  ros::init(argc, argv, "talker");
ros::init(argc, argv, "listener");

  ros::NodeHandle nlis;
ros::NodeHandle npub;
/*Cette commande crée un topic du nom cmdmotors et va publier un message de type md49test::MotorCmd dessus, 1000 messages dans le buffer*/
  ros::Publisher chatter_pub = npub.advertise<md49test::MotorCmd>("cmdmotors", 1000);
ros::Subscriber sub = nlis.subscribe("chatter", 1000, chatterCallback);
/*Nombre de messages par seconde*/
  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok())
  {
    /*crée un objet md49test::MotorCmd et rempli ces attributs*/
    md49test::MotorCmd msg;
    msg.speed1 = 5;
    msg.speed1 = -5;
    
	printf ("La compil a marché");
    /*Publie le message sur le topic*/
    chatter_pub.publish(msg);
    ROS_INFO("message envoyé");

    ros::spinOnce();

    loop_rate.sleep();
  }

ros::spin();
  return 0;
}
