#include "ros/ros.h"
/*md49test est un package qui contient un fichier msg/MotorCmd.msg, 
Il faut rajouter les dépendances dans package.xml et CMakeLists.txt 
Elles permettent la création d'objet c++/python de ce type de message.*/
#include "md49test/MotorCmd.h"

#include <sstream>

void chatterCallback(const image::String::ConstPtr& msglis)
{
  ROS_INFO("I heard: [%s]", msglis);//->data.c_str());
/*on recupere les valeurs*/
pos_horizontale = msglis->pos_horizontale;
distance = msglis->distance;
time = msglis-> time;
num_image = msglis->num_image;
}


int main(int argc, char **argv)
{

float pos-horizontale = 0;
float distance = 0;
float time = 0;
uint32 num-image = 0;

/*Cette commande crée un noeud du nom de talker dans la package dans lequel il se trouve ( pour run , rosrun suivi talker )*/
 ros::init(argc, argv, "listener");
 ros::init(argc, argv, "talker");

  ros::NodeHandle ntalk;
ros::NodeHandle nlis;
/*Cette commande crée un topic du nom cmdmotors et va publier un message de type md49test::MotorCmd dessus, 1000 messages dans le buffer*/
  ros::Publisher chatter_pub = ntalk.advertise<md49test::MotorCmd>("cmdmotors", 1000);

/*Nombre de messages par seconde*/
  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok())
  {



ros::Subscriber sub = nlis.subscribe("image", 1000, chatterCallback);

/*calcul de speed 1 et speed 2*/
//A FAIRE EN FCT DES VALEURS RECUPERES

    /*crée un objet md49test::MotorCmd et rempli ces attributs*/
    md49test::MotorCmd msgtalk;
    msgtalk.speed1 = speed1;
    msgtalk.speed1 = speed2;
    

    /*Publie le message sur le topic*/
    chatter_pub.publish(msgtalk);
    ROS_INFO("message envoyé");

    ros::spinOnce();
    //ros::spin();
    loop_rate.sleep();
  }


  return 0;
}
